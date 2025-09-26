#include "ESP8266ProtocolAdapter.h"
#include <Schedule.h>

ESP8266ProtocolAdapter::ESP8266ProtocolAdapter() : lastConnectionCheck_(0)
{
}

bool ESP8266ProtocolAdapter::begin()
{
    return connectWiFi();
}

bool ESP8266ProtocolAdapter::connectWiFi()
{
    const WiFiConfig &wifiConfig = configManager.getWiFiConfig();

    Serial.print("[WiFi] Connecting to ");
    Serial.println(wifiConfig.ssid);

    WiFi.persistent(false);  // Disable storing WiFi settings in flash
    WiFi.mode(WIFI_OFF);     // Turn off WiFi
    delay(100);              // Short delay to ensure WiFi is fully off
    
    // Configure WiFi for maximum stability
    WiFi.mode(WIFI_STA);     // Set station mode
    WiFi.setSleepMode(WIFI_NONE_SLEEP); // Disable all power saving
    WiFi.setAutoReconnect(true);  // Enable auto reconnect
    WiFi.setOutputPower(20.5);    // Set to maximum power
    
    WiFi.hostname(wifiConfig.hostname);
    WiFi.begin(wifiConfig.ssid, wifiConfig.password);
    
    // Disable modem sleep modes
    wifi_set_sleep_type(NONE_SLEEP_T);

    // Wait for connection with timeout
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < 15000)
    {
        delay(100); // Shorter delays
        yield();    // Explicit yield
        ESP.wdtFeed(); // Feed the watchdog
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println();
        Serial.print("[WiFi] Connected! IP address: ");
        Serial.println(WiFi.localIP());
        return true;
    }
    else
    {
        Serial.println();
        Serial.println("[WiFi] Connection failed!");
        return false;
    }
}

bool ESP8266ProtocolAdapter::sendReadRequest(const String &frameHex, String &outFrameHex)
{
    static unsigned long lastRequestTime = 0;
    const unsigned long MIN_REQUEST_INTERVAL = 100; // Minimum 100ms between requests
    
    // Ensure minimum time between requests
    unsigned long now = millis();
    if (now - lastRequestTime < MIN_REQUEST_INTERVAL) {
        delay(10);
        yield();
        return false;
    }
    lastRequestTime = now;
    
    // Check connection periodically
    unsigned long currentMillis = millis();
    if (currentMillis - lastConnectionCheck_ > CONNECTION_CHECK_INTERVAL)
    {
        lastConnectionCheck_ = currentMillis;
        if (WiFi.status() != WL_CONNECTED)
        {
            Serial.println("[WiFi] Connection lost, attempting to reconnect...");
            if (!connectWiFi())
            {
                return false;
            }
        }
        yield(); // Allow WiFi processing after connection check
    }

    if (!isConnected())
    {
        Serial.println("[HTTP] WiFi not connected");
        return false;
    }

    const APIConfig &apiConfig = configManager.getAPIConfig();
    return postJSON(apiConfig.read_url, frameHex, outFrameHex);
}

bool ESP8266ProtocolAdapter::sendWriteRequest(const String &frameHex, String &outFrameHex)
{
    if (!isConnected())
    {
        Serial.println("[HTTP] WiFi not connected");
        return false;
    }

    const APIConfig &apiConfig = configManager.getAPIConfig();
    return postJSON(apiConfig.write_url, frameHex, outFrameHex);
}

bool ESP8266ProtocolAdapter::postJSON(const String &url, const String &frameHex, String &outFrameHex)
{
    const APIConfig &apiConfig = configManager.getAPIConfig();

    yield(); // Allow WiFi processing
    if (!httpClient_.begin(wifiClient_, url)) {
        Serial.println("[HTTP] Failed to initialize HTTP client");
        return false;
    }
    setupHTTPHeaders(httpClient_);
    httpClient_.setTimeout(apiConfig.timeout_ms);

    // Create JSON payload
    JsonDocument jsonDoc;
    jsonDoc["frame"] = frameHex;

    String payload;
    serializeJson(jsonDoc, payload);

    Serial.print("[HTTP] POST to: ");
    Serial.println(url);
    Serial.print("[HTTP] Payload: ");
    Serial.println(payload);

    int httpResponseCode = httpClient_.POST(payload);

    if (httpResponseCode > 0)
    {
        String response = httpClient_.getString();
        Serial.print("[HTTP] Response code: ");
        Serial.println(httpResponseCode);
        Serial.print("[HTTP] Response: ");
        Serial.println(response);

        if (httpResponseCode == HTTP_CODE_OK)
        {
            // Parse JSON response
            JsonDocument responseDoc;
            DeserializationError error = deserializeJson(responseDoc, response);

            if (error)
            {
                Serial.print("[HTTP] JSON parsing failed: ");
                Serial.println(error.c_str());
                httpClient_.end();
                return false;
            }

            if (responseDoc["frame"].is<const char*>())
            {
                outFrameHex = responseDoc["frame"].as<String>();
                httpClient_.end();
                return true;
            }
            else
            {
                Serial.println("[HTTP] Response missing 'frame' field");
            }
        }
    }
    else
    {
        Serial.print("[HTTP] Error: ");
        Serial.println(httpClient_.errorToString(httpResponseCode));
    }

    httpClient_.end();
    return false;
}

void ESP8266ProtocolAdapter::setupHTTPHeaders(HTTPClient &client)
{
    const APIConfig &apiConfig = configManager.getAPIConfig();

    client.addHeader("Content-Type", "application/json");
    client.addHeader("Accept", "application/json");

    if (strlen(apiConfig.api_key) > 0)
    {
        String authHeader = "Bearer " + String(apiConfig.api_key);
        client.addHeader("Authorization", authHeader);
    }
}