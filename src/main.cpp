#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>
#include <Ticker.h>

#include "ESP8266Config.h"
#include "ESP8266Inverter.h"
#include "ESP8266DataTypes.h"
#include "ESP8266PollingConfig.h"
#include "ESP8266Compression.h"

// Global objects
ESP8266Inverter inverter;
ESP8266DataBuffer dataBuffer(10); // Smaller buffer for ESP8266
ESP8266PollingConfig pollingConfig;

// Timers
Ticker pollTicker;
Ticker uploadTicker;

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

// Simple CRC32 (polynomial 0xEDB88320) for MAC stub
static uint32_t crc32_calc(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j)
        {
            uint32_t mask = -(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

// Flags set from timer callbacks (ISR context) to defer work to loop()
volatile bool pollPending = false;
volatile bool uploadPending = false;

void IRAM_ATTR onPollTimer()
{
    pollPending = true;
}

void IRAM_ATTR onUploadTimer()
{
    uploadPending = true;
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("==================================");
    Serial.println("    BitBots EcoWatt ESP8266");
    Serial.println("==================================");

    startTime = millis();
    systemInitialized = initializeSystem();

    if (systemInitialized)
    {
        setupPollingConfig();

        // Start polling and upload timers
        const DeviceConfig &deviceConfig = configManager.getDeviceConfig();
    // Timer callbacks must be ISR-safe: set a flag and perform heavy work in loop()
    pollTicker.attach_ms(deviceConfig.poll_interval_ms, onPollTimer);
    uploadTicker.attach_ms(deviceConfig.upload_interval_ms, onUploadTimer);

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
    // Handle serial commands for configuration
    handleSerialCommands();

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
    // Check for deferred tasks set by timers
    if (pollPending)
    {
        // clear flag before executing to avoid missing new requests
        pollPending = false;
        pollSensors();
    }

    if (uploadPending)
    {
        uploadPending = false;
        uploadData();
    }

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

    Serial.println("[POLL] Starting sensor polling...");

    Sample sample;
    sample.timestamp = millis() - startTime;

    bool allSuccess = true;
    const auto &enabledParams = pollingConfig.getEnabledParameters();

    for (ParameterType paramType : enabledParams)
    {
        const ParameterConfig &paramConfig = pollingConfig.getParameterConfig(paramType);
        float value;

        if (paramConfig.readFunction(inverter, value))
        {
            sample.setValue(paramType, value);
            // Friendly name + unit with fallbacks
            String friendlyName = paramConfig.name.length() ? paramConfig.name : parameterTypeToString(paramType);
            String unit = paramConfig.unit;
            if (unit.length() == 0)
            {
                switch (paramType)
                {
                case ParameterType::AC_VOLTAGE:
                case ParameterType::PV1_VOLTAGE:
                case ParameterType::PV2_VOLTAGE:
                    unit = " V"; break;
                case ParameterType::AC_CURRENT:
                case ParameterType::PV1_CURRENT:
                case ParameterType::PV2_CURRENT:
                    unit = " A"; break;
                case ParameterType::AC_FREQUENCY:
                    unit = " Hz"; break;
                case ParameterType::TEMPERATURE:
                    unit = " °C"; break;
                case ParameterType::OUTPUT_POWER:
                    unit = " W"; break;
                case ParameterType::EXPORT_POWER_PERCENT:
                    unit = " %"; break;
                default:
                    unit = ""; break;
                }
            }
            Serial.print("[POLL] ");
            Serial.print(friendlyName);
            Serial.print(": ");
            Serial.print(value, 2);
            Serial.println(unit);
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

    static bool uploadInProgress = false;
    if (uploadInProgress)
    {
        Serial.println("[UPLOAD] Previous upload still in progress; skipping this tick");
        return;
    }
    uploadInProgress = true;

    Serial.println("[UPLOAD] Starting data upload...");

    // Non-destructive snapshot; clear only after ACK success
    auto samples = dataBuffer.snapshot();
    Serial.print("[UPLOAD] Uploading ");
    Serial.print(samples.size());
    Serial.println(" samples");

    if (uploadToServer(samples))
    {
        Serial.println("[UPLOAD] Upload successful");
        dataBuffer.clear();
    }
    else
    {
        Serial.println("[UPLOAD] Upload failed");
    }

    uploadInProgress = false;
}

bool uploadToServer(const std::vector<Sample> &samples)
{
    const APIConfig &apiConfig = configManager.getAPIConfig();
    HTTPClient httpClient;
    WiFiClient wifiClient;

    // Use upload_url for cloud ingestion
    const char *uploadUrl = apiConfig.upload_url[0] != '\0' ? apiConfig.upload_url : "http://10.63.73.102:5000/upload";
    httpClient.begin(wifiClient, uploadUrl);
    Serial.print("[HTTP] POST to: "); Serial.println(uploadUrl);
    httpClient.addHeader("Content-Type", "application/json");
    httpClient.setTimeout(apiConfig.timeout_ms);

    // Build payload expected by Flask app.py with compression + aggregation
    DynamicJsonDocument jsonDoc(8192);
    // Session/window metadata
    static uint32_t session_counter = 0;
    // Derive window timing from first/last sample timestamps
    jsonDoc["device_id"] = WiFi.hostname();
    jsonDoc["timestamp"] = millis() - startTime; // client send time
    jsonDoc["session_id"] = (uint32_t)(ESP.getChipId() ^ millis() ^ (++session_counter));
    jsonDoc["poll_count"] = (int)samples.size();

    // Accumulators for a single-document (non-chunked) build
    size_t totalOriginalBytes = 0;      // sum of 4 * n_samples per field
    size_t totalCompressedBytes = 0;    // sum of varint-encoded bytes_len per field
    float totalCpuMs = 0.0f;            // sum of cpu_time_ms per field
    bool verifyAll = true;              // AND of per-field verify_ok

    // Helper to append a single parameter field into a doc and accumulate totals
    auto appendParamField = [&](DynamicJsonDocument &doc, ParameterType param) {
        JsonObject fieldsObjLoc = doc["fields"].isNull() ? doc.createNestedObject("fields") : doc["fields"].as<JsonObject>();

        // Collect scaled integer series for this parameter
        std::vector<long> series;
        series.reserve(samples.size());
        for (const auto &s : samples)
        {
            if (!s.hasValue(param)) continue;
            float v = s.getValue(param);
            long scaled = 0;
            if (param == ParameterType::AC_VOLTAGE || param == ParameterType::AC_CURRENT || param == ParameterType::AC_FREQUENCY)
                scaled = (long)roundf(v * 1000.0f);
            else
                scaled = (long)roundf(v);
            series.push_back(scaled);
        }
        if (series.empty()) return; // nothing to add

        long minV = series[0], maxV = series[0];
        long sum = 0;
        for (long x : series) { if (x < minV) minV = x; if (x > maxV) maxV = x; sum += x; }
        float avg = (float)sum / (float)series.size();

        unsigned long t0 = micros();
        std::vector<int32_t> deltas;
        Compression::delta_compress(series, deltas);
        std::vector<uint8_t> varintBytes;
        Compression::encode_deltas_varint(deltas, varintBytes);
        bool verify_ok = true;
        {
            std::vector<int32_t> deltas2;
            if (!Compression::decode_deltas_varint(varintBytes, deltas2)) verify_ok = false;
            else {
                std::vector<long> recon;
                Compression::delta_decompress(deltas2, recon);
                if (recon.size() != series.size()) verify_ok = false;
                else {
                    for (size_t i = 0; i < recon.size(); ++i) { if (recon[i] != series[i]) { verify_ok = false; break; } }
                }
            }
        }
        unsigned long t1 = micros();
    float cpu_ms = (t1 - t0) / 1000.0f;
    size_t bytes_len = varintBytes.size();

        String paramName = parameterTypeToString(param);
        JsonObject field = fieldsObjLoc.createNestedObject(paramName.c_str());
        field["method"] = "Delta";
        field["param_id"] = static_cast<int>(param);
        field["n_samples"] = (int)series.size();
        field["bytes_len"] = (int)bytes_len;
        field["cpu_time_ms"] = cpu_ms;
        field["verify_ok"] = verify_ok;
    field["original_bytes"] = (int)(4 * series.size());

        JsonObject agg = field.createNestedObject("agg");
        agg["min"] = (long)minV;
        agg["avg"] = avg;
        agg["max"] = (long)maxV;

        JsonArray payloadArr = field.createNestedArray("payload");
        for (auto d : deltas) payloadArr.add((long)d);
        field["payload_varint_hex"] = Compression::hex_encode(varintBytes);

    };

    const auto enabledParams = pollingConfig.getEnabledParameters();

    // First, attempt to build a single-chunk payload and see if it fits
    for (ParameterType p : enabledParams) { appendParamField(jsonDoc, p); }

    const size_t PAYLOAD_THRESHOLD = 3500; // bytes; adjust if needed

    // Function to send a document with retry/backoff
    auto sendWithRetry = [&](DynamicJsonDocument &doc) -> bool {
        // Compute MAC over JSON without mac field
        String tmp;
        serializeJson(doc, tmp);
        uint32_t macLocal = crc32_calc(reinterpret_cast<const uint8_t *>(tmp.c_str()), tmp.length());
        doc["mac_crc32"] = macLocal;
        tmp = String();
        serializeJson(doc, tmp);

        Serial.print("[HTTP] Payload size: ");
        Serial.println(tmp.length());

        const int maxAttempts = 3;
        int attemptLocal = 0;
        while (attemptLocal < maxAttempts)
        {
            int code = httpClient.POST(tmp);
            if (code > 0)
            {
                String response = httpClient.getString();
                Serial.print("[HTTP] Response code: ");
                Serial.println(code);
                Serial.print("[HTTP] Response: ");
                Serial.println(response);

                if (code == HTTP_CODE_OK)
                {
                    StaticJsonDocument<1024> respDoc;
                    if (deserializeJson(respDoc, response) == DeserializationError::Ok)
                    {
                        const char *status = respDoc["status"] | "";
                        if (strcmp(status, "ok") == 0)
                        {
                            if (respDoc.containsKey("next_config"))
                            {
                                JsonObject cfg = respDoc["next_config"].as<JsonObject>();
                                Serial.print("[HTTP] next_config: ");
                                serializeJson(cfg, Serial);
                                Serial.println();
                            }
                            return true;
                        }
                    }
                }
            }
            else
            {
                Serial.print("[HTTP] Error: ");
                Serial.println(httpClient.errorToString(code));
            }
            ++attemptLocal;
            if (attemptLocal < maxAttempts)
            {
                uint32_t backoffMs = (1u << (attemptLocal - 1)) * 1000u;
                if (backoffMs > 4000u) backoffMs = 4000u;
                Serial.print("[HTTP] Retry attempt ");
                Serial.print(attemptLocal + 1);
                Serial.print(" in ");
                Serial.print(backoffMs);
                Serial.println(" ms");
                delay(backoffMs);
            }
        }
        return false;
    };

    // If the single doc is too large, chunk by fields
    String firstPayload;
    serializeJson(jsonDoc, firstPayload);
    if (firstPayload.length() <= PAYLOAD_THRESHOLD)
    {
        bool ok = sendWithRetry(jsonDoc);
        httpClient.end();
        return ok;
    }

    Serial.println("[UPLOAD] Payload exceeds threshold, chunking by fields");

    // Build per-field chunks
    struct FieldNameParam { String name; ParameterType param; };
    std::vector<FieldNameParam> fieldList;
    for (auto p : enabledParams)
    {
        FieldNameParam f{ parameterTypeToString(p), p };
        fieldList.push_back(f);
    }

    // Helper to append a single parameter into a chunk doc and accumulate chunk-level totals only
    auto appendParamFieldChunk = [&](DynamicJsonDocument &doc, ParameterType param,
                                     size_t &chunkOriginalBytes, size_t &chunkCompressedBytes,
                                     float &chunkCpuMs, bool &chunkVerifyAll) {
        JsonObject fieldsObjLoc = doc["fields"].isNull() ? doc.createNestedObject("fields") : doc["fields"].as<JsonObject>();

        // Collect scaled integer series for this parameter
        std::vector<long> series;
        series.reserve(samples.size());
        for (const auto &s : samples)
        {
            if (!s.hasValue(param)) continue;
            float v = s.getValue(param);
            long scaled = 0;
            if (param == ParameterType::AC_VOLTAGE || param == ParameterType::AC_CURRENT || param == ParameterType::AC_FREQUENCY)
                scaled = (long)roundf(v * 1000.0f);
            else
                scaled = (long)roundf(v);
            series.push_back(scaled);
        }
        if (series.empty()) return; // nothing to add

        long minV = series[0], maxV = series[0];
        long sum = 0;
        for (long x : series) { if (x < minV) minV = x; if (x > maxV) maxV = x; sum += x; }
        float avg = (float)sum / (float)series.size();

        unsigned long t0 = micros();
        std::vector<int32_t> deltas;
        Compression::delta_compress(series, deltas);
        std::vector<uint8_t> varintBytes;
        Compression::encode_deltas_varint(deltas, varintBytes);
        bool verify_ok = true;
        {
            std::vector<int32_t> deltas2;
            if (!Compression::decode_deltas_varint(varintBytes, deltas2)) verify_ok = false;
            else {
                std::vector<long> recon;
                Compression::delta_decompress(deltas2, recon);
                if (recon.size() != series.size()) verify_ok = false;
                else {
                    for (size_t i = 0; i < recon.size(); ++i) { if (recon[i] != series[i]) { verify_ok = false; break; } }
                }
            }
        }
        unsigned long t1 = micros();
        float cpu_ms = (t1 - t0) / 1000.0f;
        size_t bytes_len = varintBytes.size();

        String paramName = parameterTypeToString(param);
        JsonObject field = fieldsObjLoc.createNestedObject(paramName.c_str());
        field["method"] = "Delta";
        field["param_id"] = static_cast<int>(param);
        field["n_samples"] = (int)series.size();
        field["bytes_len"] = (int)bytes_len;
        field["cpu_time_ms"] = cpu_ms;
        field["verify_ok"] = verify_ok;
        field["original_bytes"] = (int)(4 * series.size());

        JsonObject agg = field.createNestedObject("agg");
        agg["min"] = (long)minV;
        agg["avg"] = avg;
        agg["max"] = (long)maxV;

        JsonArray payloadArr = field.createNestedArray("payload");
        for (auto d : deltas) payloadArr.add((long)d);
        field["payload_varint_hex"] = Compression::hex_encode(varintBytes);

        // Accumulate chunk-level totals only
        chunkOriginalBytes += (4 * series.size());
        chunkCompressedBytes += bytes_len;
        chunkCpuMs += cpu_ms;
        chunkVerifyAll = chunkVerifyAll && verify_ok;
    };

    // Try sequentially packing fields until threshold is reached, send chunk, then continue
    size_t total = 0;
    {
        // compute total chunks by simulating packing
        size_t countChunks = 0;
        size_t idx = 0;
        while (idx < fieldList.size())
        {
            DynamicJsonDocument docChunk(8192);
            // copy session/window metadata
            for (JsonPair kv : jsonDoc.as<JsonObject>())
            {
                if (String(kv.key().c_str()) == "fields") continue;
                docChunk[kv.key().c_str()] = kv.value();
            }
            JsonObject fieldsObjC = docChunk.createNestedObject("fields");
            size_t startIdx = idx;
            for (; idx < fieldList.size(); ++idx)
            {
                appendParamField(docChunk, fieldList[idx].param);
                String probe; serializeJson(docChunk, probe);
                if (probe.length() > PAYLOAD_THRESHOLD)
                {
                    // remove the last addition by rebuilding chunk without it
                    docChunk.remove("fields");
                    fieldsObjC = docChunk.createNestedObject("fields");
                    for (size_t j = startIdx; j < idx; ++j)
                        appendParamField(docChunk, fieldList[j].param);
                    break;
                }
            }
            countChunks++;
        }
        total = countChunks;
    }

    size_t seq = 1;
    size_t idx = 0;
    while (idx < fieldList.size())
    {
        DynamicJsonDocument docChunk(8192);
        for (JsonPair kv : jsonDoc.as<JsonObject>())
        {
            if (String(kv.key().c_str()) == "fields") continue;
            docChunk[kv.key().c_str()] = kv.value();
        }
        docChunk["chunk_seq"] = (int)seq;
        docChunk["chunk_total"] = (int)total;
        JsonObject fieldsObjC = docChunk.createNestedObject("fields");

        // Chunk accumulators
        size_t chunkOriginalBytes = 0;
        size_t chunkCompressedBytes = 0;
        float chunkCpuMs = 0.0f;
        bool chunkVerifyAll = true;

        size_t startIdx = idx;
        for (; idx < fieldList.size(); ++idx)
        {
            appendParamFieldChunk(docChunk, fieldList[idx].param, chunkOriginalBytes, chunkCompressedBytes, chunkCpuMs, chunkVerifyAll);
            String probe; serializeJson(docChunk, probe);
            if (probe.length() > PAYLOAD_THRESHOLD)
            {
                // rebuild without last
                docChunk.remove("fields");
                fieldsObjC = docChunk.createNestedObject("fields");
                for (size_t j = startIdx; j < idx; ++j)
                    appendParamFieldChunk(docChunk, fieldList[j].param, chunkOriginalBytes, chunkCompressedBytes, chunkCpuMs, chunkVerifyAll);
                break;
            }
        }

        // Add window totals and chunk subtotals
        docChunk["original_payload_size_bytes_total"] = (int)totalOriginalBytes;
        docChunk["compressed_payload_size_bytes_total"] = (int)totalCompressedBytes;
        docChunk["cpu_time_ms_total_window"] = totalCpuMs;
        docChunk["verify_ok_all_window"] = verifyAll;

        docChunk["original_payload_size_bytes_chunk"] = (int)chunkOriginalBytes;
        docChunk["compressed_payload_size_bytes_chunk"] = (int)chunkCompressedBytes;
        docChunk["cpu_time_ms_chunk"] = chunkCpuMs;
        docChunk["verify_ok_all_chunk"] = chunkVerifyAll;

        bool ok = sendWithRetry(docChunk);
        if (!ok)
        {
            httpClient.end();
            return false;
        }
        ++seq;
    }

    httpClient.end();
    return true;
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