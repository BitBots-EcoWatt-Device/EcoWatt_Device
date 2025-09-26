#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>
#include <Ticker.h>

#include "ESP8266Config.h"
#include "ESP8266Inverter.h"
#include "ESP8266DataTypes.h"
#include "ESP8266PollingConfig.h"

// Global objects
ESP8266Inverter inverter;
ESP8266DataBuffer dataBuffer(10); // Smaller buffer for ESP8266
ESP8266PollingConfig pollingConfig;

// Timers
Ticker pollTicker;
Ticker uploadTicker;

// Ticker flags (set from ISR-safe callbacks)
volatile bool pollRequested = false;
volatile bool uploadRequested = false;

// ISR-safe callback wrappers - only set flags
void IRAM_ATTR onPollTicker()
{
    pollRequested = true;
}

void IRAM_ATTR onUploadTicker()
{
    uploadRequested = true;
}

// Status variables
unsigned long startTime;
bool systemInitialized = false;

// Function prototypes
void setup();
void loop();
bool initializeSystem();
void pollSensors();
void uploadData();
void setupPollingConfig();
void printSystemStatus();
void handleSerialCommands();
bool uploadToServer(const std::vector<Sample> &samples);

// Task scheduling variables
unsigned long lastYield = 0;
const unsigned long YIELD_INTERVAL = 50; // Yield every 50ms
const unsigned long MIN_TASK_INTERVAL = 100; // Minimum time between tasks
unsigned long lastTaskTime = 0;

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("==================================");
    Serial.println("    BitBots EcoWatt ESP8266");
    Serial.println("==================================");

    // Disable all power saving and optimize for stability
    WiFi.persistent(false);       // Disable flash writes for WiFi
    system_update_cpu_freq(160);  // Set CPU to 160MHz for better stability
    
    // Increase WiFi TX power
    WiFi.setOutputPower(20.5);
    
    // Disable all sleep modes
    wifi_set_sleep_type(NONE_SLEEP_T);
    
    startTime = millis();
    systemInitialized = initializeSystem();

    if (systemInitialized)
    {
        setupPollingConfig();

        // Start polling and upload timers
        const DeviceConfig &deviceConfig = configManager.getDeviceConfig();
        // Attach ISR-safe wrappers that only set flags. Heavy work runs in loop().
        pollTicker.attach_ms(deviceConfig.poll_interval_ms, onPollTicker);
        uploadTicker.attach_ms(deviceConfig.upload_interval_ms, onUploadTicker);

        Serial.println("[MAIN] System initialized successfully");
        printSystemStatus();
    }
    else
    {
        Serial.println("[MAIN] System initialization failed!");
    }
}

void loop()
{
    static unsigned long lastYield = 0;
    static unsigned long lastWiFiCheck = 0;
    const unsigned long YIELD_INTERVAL = 25;    // More frequent yields
    const unsigned long WIFI_CHECK_INTERVAL = 5000; // Check WiFi every 5 seconds
    
    unsigned long currentMillis = millis();
    
    // More frequent yields for better stability
    if (currentMillis - lastYield >= YIELD_INTERVAL) {
        yield();
        ESP.wdtFeed();
        lastYield = currentMillis;
    }
    
    // Regular WiFi status checks
    if (currentMillis - lastWiFiCheck >= WIFI_CHECK_INTERVAL) {
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("[WiFi] Connection lost, reconnecting...");
            WiFi.disconnect();
            delay(100);
            WiFi.reconnect();
        }
        lastWiFiCheck = currentMillis;
    }
    // Handle serial commands for configuration
    handleSerialCommands();

    // Allow background tasks to run
    yield();

    // Handle scheduled work requested by ticker callbacks (safe context)
    if (pollRequested)
    {
        // clear flag as soon as possible
        pollRequested = false;
        pollSensors();
    }

    if (uploadRequested)
    {
        uploadRequested = false;
        uploadData();
    }

    // Basic watchdog - restart if system hangs
    static unsigned long lastLoopTime = 0;
    unsigned long currentTime = millis();

    if (currentTime - lastLoopTime > 60000)
    { // 1 minute timeout
        Serial.println("[WATCHDOG] Loop timeout - restarting");
        ESP.restart();
    }
    lastLoopTime = currentTime;

    // Small delay to prevent watchdog reset
    delay(100);
}

bool initializeSystem()
{
    Serial.println("[INIT] Starting system initialization...");

    // Initialize configuration manager
    if (!configManager.begin())
    {
        Serial.println("[INIT] Failed to load configuration, using defaults");
    }

    // Initialize inverter communication
    if (!inverter.begin())
    {
        Serial.println("[INIT] Failed to initialize inverter communication");
        return false;
    }

    Serial.println("[INIT] System initialization complete");
    return true;
}

void setupPollingConfig()
{
    Serial.println("[CONFIG] Setting up polling configuration...");

    // Configure to poll key parameters
    std::vector<ParameterType> params = {
        ParameterType::AC_VOLTAGE,
        ParameterType::AC_CURRENT,
        ParameterType::AC_FREQUENCY,
        ParameterType::TEMPERATURE,
        ParameterType::OUTPUT_POWER};

    pollingConfig.setParameters(params);
    pollingConfig.printEnabledParameters();
}

void pollSensors()
{
    if (!systemInitialized)
        return;

    unsigned long currentMillis = millis();
    
    // Ensure minimum time between tasks
    if (currentMillis - lastTaskTime < MIN_TASK_INTERVAL) {
        delay(10);
        yield();
        return;
    }
    lastTaskTime = currentMillis;

    Serial.println("[POLL] Starting sensor polling...");

    Sample sample;
    sample.timestamp = currentMillis - startTime;

    bool allSuccess = true;
    const auto &enabledParams = pollingConfig.getEnabledParameters();
    
    // Reset WiFi watchdog timer
    ESP.wdtFeed();

    for (ParameterType paramType : enabledParams)
    {
        const ParameterConfig &paramConfig = pollingConfig.getParameterConfig(paramType);
        float value;

        if (paramConfig.readFunction(inverter, value))
        {
            sample.setValue(paramType, value);
            Serial.print("[POLL] ");
            Serial.print(paramConfig.name);
            Serial.print(": ");
            Serial.print(value);
            Serial.println(paramConfig.unit);
        }
        else
        {
            Serial.print("[POLL] Failed to read ");
            Serial.println(paramConfig.name);
            allSuccess = false;
        }
    }

    if (allSuccess && dataBuffer.hasSpace())
    {
        dataBuffer.append(sample);
        Serial.print("[BUFFER] Sample added, buffer size: ");
        Serial.println(dataBuffer.size());
    }
    else if (!allSuccess)
    {
        Serial.println("[POLL] Poll failed for some parameters");
    }
    else
    {
        Serial.println("[BUFFER] Buffer full, sample discarded");
    }
}

void uploadData()
{
    if (!systemInitialized || dataBuffer.empty())
    {
        Serial.println("[UPLOAD] No data to upload");
        return;
    }

    Serial.println("[UPLOAD] Starting data upload...");

    auto samples = dataBuffer.flush();
    Serial.print("[UPLOAD] Uploading ");
    Serial.print(samples.size());
    Serial.println(" samples");

    if (uploadToServer(samples))
    {
        Serial.println("[UPLOAD] Upload successful");
    }
    else
    {
        Serial.println("[UPLOAD] Upload failed");
    }
}

bool uploadToServer(const std::vector<Sample> &samples)
{
    const APIConfig &apiConfig = configManager.getAPIConfig();

    HTTPClient httpClient;
    WiFiClient wifiClient;

    httpClient.begin(wifiClient, "http://10.63.73.102:5000/upload"); // Cloud dashboard upload URL
    httpClient.addHeader("Content-Type", "application/json");
    httpClient.setTimeout(apiConfig.timeout_ms);

    // Create JSON payload
    JsonDocument jsonDoc;
    JsonArray samplesArray = jsonDoc["samples"].to<JsonArray>();

    for (const Sample &sample : samples)
    {
        JsonObject sampleObj = samplesArray.add<JsonObject>();
        sampleObj["timestamp"] = sample.timestamp;

        JsonObject dataObj = sampleObj["data"].to<JsonObject>();
        for (ParameterType param : pollingConfig.getEnabledParameters())
        {
            if (sample.hasValue(param))
            {
                String paramName = parameterTypeToString(param);
                dataObj[paramName] = sample.getValue(param);
            }
        }
    }

    String payload;
    serializeJson(jsonDoc, payload);

    Serial.print("[HTTP] Payload size: ");
    Serial.println(payload.length());

    int httpResponseCode = httpClient.POST(payload);

    if (httpResponseCode > 0)
    {
        String response = httpClient.getString();
        Serial.print("[HTTP] Response code: ");
        Serial.println(httpResponseCode);

        httpClient.end();
        return httpResponseCode == HTTP_CODE_OK;
    }
    else
    {
        Serial.print("[HTTP] Error: ");
        Serial.println(httpClient.errorToString(httpResponseCode));
        httpClient.end();
        return false;
    }
}

void printSystemStatus()
{
    Serial.println("\n==== SYSTEM STATUS ====");

    // WiFi status
    Serial.print("WiFi Status: ");
    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.print("Connected to ");
        Serial.print(WiFi.SSID());
        Serial.print(" (");
        Serial.print(WiFi.localIP());
        Serial.println(")");
    }
    else
    {
        Serial.println("Not Connected");
    }

    // Memory status
    Serial.print("Free Heap: ");
    Serial.print(ESP.getFreeHeap());
    Serial.println(" bytes");

    // Buffer status
    Serial.print("Buffer Size: ");
    Serial.print(dataBuffer.size());
    Serial.print("/");
    Serial.println(configManager.getDeviceConfig().buffer_size);

    // Configuration
    Serial.print("Poll Interval: ");
    Serial.print(configManager.getDeviceConfig().poll_interval_ms);
    Serial.println(" ms");

    Serial.print("Upload Interval: ");
    Serial.print(configManager.getDeviceConfig().upload_interval_ms);
    Serial.println(" ms");

    Serial.println("========================\n");
}

void handleSerialCommands()
{
    if (Serial.available())
    {
        String command = Serial.readStringUntil('\n');
        command.trim();

        if (command == "status")
        {
            printSystemStatus();
        }
        else if (command == "restart")
        {
            Serial.println("[CMD] Restarting system...");
            ESP.restart();
        }
        else if (command == "test")
        {
            Serial.println("[CMD] Running test poll...");
            pollSensors();
        }
        else if (command == "upload")
        {
            Serial.println("[CMD] Triggering upload...");
            uploadData();
        }
        else if (command == "wifi")
        {
            Serial.print("[CMD] WiFi Status: ");
            Serial.println(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
            if (WiFi.status() == WL_CONNECTED)
            {
                Serial.print("IP: ");
                Serial.println(WiFi.localIP());
                Serial.print("RSSI: ");
                Serial.println(WiFi.RSSI());
            }
        }
        else if (command == "help")
        {
            Serial.println("[CMD] Available commands:");
            Serial.println("  status  - Show system status");
            Serial.println("  restart - Restart the system");
            Serial.println("  test    - Run test sensor poll");
            Serial.println("  upload  - Trigger data upload");
            Serial.println("  wifi    - Show WiFi status");
            Serial.println("  help    - Show this help");
        }
        else if (command.length() > 0)
        {
            Serial.println("[CMD] Unknown command. Type 'help' for available commands.");
        }
    }
}