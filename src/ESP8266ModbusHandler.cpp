#include "ESP8266ModbusHandler.h"

// Modbus CRC16 lookup table
const uint16_t ESP8266ModbusHandler::crc16_table[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040};

ESP8266ModbusHandler::ESP8266ModbusHandler() : lastOperationTime_(0)
{
}

bool ESP8266ModbusHandler::begin()
{
    return adapter_.begin();
}

bool ESP8266ModbusHandler::waitForOperationTimeout()
{
    unsigned long currentTime = millis();
    if (currentTime - lastOperationTime_ < MODBUS_OPERATION_TIMEOUT) {
        unsigned long waitTime = MODBUS_OPERATION_TIMEOUT - (currentTime - lastOperationTime_);
        delay(min(waitTime, (unsigned long)100)); // Break up long delays
        yield(); // Allow WiFi processing
        return true;
    }
    lastOperationTime_ = currentTime;
    return false;
}

bool ESP8266ModbusHandler::readRegisters(uint16_t startAddr, uint16_t numRegs, std::vector<uint16_t> &values, uint8_t slaveAddr)
{
    // Check if enough time has passed since last operation
    if (waitForOperationTimeout()) {
        return false; // Not enough time has passed
    }
    
    // Clear any previous values
    values.clear();
    
    String frameHex = buildReadFrame(slaveAddr, startAddr, numRegs);
    String responseHex;

    // Debug: print outgoing frame
    Serial.print("[MODBUS] Outgoing frame: ");
    Serial.println(frameHex);

    for (int retry = 0; retry < MODBUS_RETRY_COUNT; retry++) {
        yield(); // Allow WiFi processing
        
        if (adapter_.sendReadRequest(frameHex, responseHex)) {
            // Debug: print received raw hex from adapter
            Serial.print("[MODBUS] Received hex: ");
            Serial.println(responseHex);
            break;
        }
        
        if (retry < MODBUS_RETRY_COUNT - 1) {
            delay(MODBUS_RETRY_DELAY);
            yield();
            delay(10); // Additional short delay for stability
        } else {
            Serial.println("[MODBUS] Failed to send read request after retries");
            return false;
        }
    }

    // Parse response
    std::vector<uint8_t> response = hexToBytes(responseHex);

    Serial.print("[MODBUS] Response bytes: ");
    Serial.println(response.size());

    if (response.size() < 5)
    {
        Serial.println("[MODBUS] Response too short");
        return false;
    }

    // Check for exception
    if (response[1] & 0x80)
    {
        uint8_t exCode = response[2];
        Serial.print("[MODBUS] Exception code: 0x");
        if (exCode < 0x10) Serial.print('0');
        Serial.print(String(exCode, HEX));
        Serial.print(" (dec "); Serial.print(exCode); Serial.print(") -> ");
        Serial.println(modbusExceptionMessage(exCode));
        return false;
    }

    // Verify CRC
    uint16_t receivedCRC = (response[response.size() - 1] << 8) | response[response.size() - 2];
    std::vector<uint8_t> dataForCRC(response.begin(), response.end() - 2);
    uint16_t calculatedCRC = calculateCRC(dataForCRC);

    if (receivedCRC != calculatedCRC)
    {
        Serial.print("[MODBUS] CRC mismatch: received=0x");
        Serial.print(String(receivedCRC, HEX));
        Serial.print(" calculated=0x");
        Serial.println(String(calculatedCRC, HEX));
        return false;
    }

    // Extract register values
    size_t byteCount = static_cast<size_t>(response[2]);
    if (response.size() != byteCount + 5)
    {
        Serial.print("[MODBUS] Invalid byte count: expected=");
        Serial.print(byteCount + 5);
        Serial.print(" actual=");
        Serial.println(response.size());
        return false;
    }

    values.clear();
    for (int i = 0; i < numRegs; i++)
    {
        uint16_t value = (response[3 + i * 2] << 8) | response[4 + i * 2];
        values.push_back(value);
    }

    return true;
}

bool ESP8266ModbusHandler::writeRegister(uint16_t regAddr, uint16_t regValue, uint8_t slaveAddr)
{
    String frameHex = buildWriteFrame(slaveAddr, regAddr, regValue);
    String responseHex;

    if (!adapter_.sendWriteRequest(frameHex, responseHex))
    {
        Serial.println("[MODBUS] Failed to send write request");
        return false;
    }

    // Parse response
    std::vector<uint8_t> response = hexToBytes(responseHex);

    if (response.size() < 8)
    {
        Serial.println("[MODBUS] Write response too short");
        return false;
    }

    // Check for exception
    if (response[1] & 0x80)
    {
        Serial.print("[MODBUS] Write exception: ");
        Serial.println(modbusExceptionMessage(response[2]));
        return false;
    }

    // Verify CRC
    uint16_t receivedCRC = (response[response.size() - 1] << 8) | response[response.size() - 2];
    std::vector<uint8_t> dataForCRC(response.begin(), response.end() - 2);
    uint16_t calculatedCRC = calculateCRC(dataForCRC);

    if (receivedCRC != calculatedCRC)
    {
        Serial.println("[MODBUS] Write CRC mismatch");
        return false;
    }

    return true;
}

String ESP8266ModbusHandler::buildReadFrame(uint8_t slaveAddr, uint16_t startAddr, uint16_t numRegs)
{
    std::vector<uint8_t> frame;

    frame.push_back(slaveAddr);
    frame.push_back(0x03); // Function code: Read Holding Registers
    frame.push_back((startAddr >> 8) & 0xFF);
    frame.push_back(startAddr & 0xFF);
    frame.push_back((numRegs >> 8) & 0xFF);
    frame.push_back(numRegs & 0xFF);

    uint16_t crc = calculateCRC(frame);
    frame.push_back(crc & 0xFF);
    frame.push_back((crc >> 8) & 0xFF);

    return bytesToHex(frame);
}

String ESP8266ModbusHandler::buildWriteFrame(uint8_t slaveAddr, uint16_t regAddr, uint16_t regValue)
{
    std::vector<uint8_t> frame;

    frame.push_back(slaveAddr);
    frame.push_back(0x06); // Function code: Write Single Register
    frame.push_back((regAddr >> 8) & 0xFF);
    frame.push_back(regAddr & 0xFF);
    frame.push_back((regValue >> 8) & 0xFF);
    frame.push_back(regValue & 0xFF);

    uint16_t crc = calculateCRC(frame);
    frame.push_back(crc & 0xFF);
    frame.push_back((crc >> 8) & 0xFF);

    return bytesToHex(frame);
}

String ESP8266ModbusHandler::bytesToHex(const std::vector<uint8_t> &bytes)
{
    String hex = "";
    for (uint8_t byte : bytes)
    {
        if (byte < 0x10)
            hex += "0";
        hex += String(byte, HEX);
    }
    hex.toUpperCase();
    return hex;
}

std::vector<uint8_t> ESP8266ModbusHandler::hexToBytes(const String &hex)
{
    std::vector<uint8_t> bytes;
    for (size_t i = 0; i < hex.length(); i += 2)
    {
        String byteString = hex.substring(i, i + 2);
        bytes.push_back(static_cast<uint8_t>(strtol(byteString.c_str(), NULL, 16)));
    }
    return bytes;
}

uint16_t ESP8266ModbusHandler::calculateCRC(const std::vector<uint8_t> &data)
{
    uint16_t crc = 0xFFFF;

    for (uint8_t byte : data)
    {
        uint8_t index = (crc ^ byte) & 0xFF;
        crc = (crc >> 8) ^ crc16_table[index];
    }

    return crc;
}

String ESP8266ModbusHandler::modbusExceptionMessage(uint8_t code)
{
    switch (code)
    {
    case 0x01:
        return "Illegal Function";
    case 0x02:
        return "Illegal Data Address";
    case 0x03:
        return "Illegal Data Value";
    case 0x04:
        return "Slave Device Failure";
    case 0x05:
        return "Acknowledge";
    case 0x06:
        return "Slave Device Busy";
    case 0x08:
        return "Memory Parity Error";
    case 0x0A:
        return "Gateway Path Unavailable";
    case 0x0B:
        return "Gateway Target Device Failed to Respond";
    default:
        return "Unknown Exception Code: " + String(code);
    }
}