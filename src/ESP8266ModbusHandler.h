#ifndef ESP8266_MODBUS_HANDLER_H
#define ESP8266_MODBUS_HANDLER_H

#include <Arduino.h>
#include <vector>
#include "ESP8266ProtocolAdapter.h"

#define MODBUS_RETRY_COUNT 3
#define MODBUS_RETRY_DELAY 100
#define MODBUS_OPERATION_TIMEOUT 2000

class ESP8266ModbusHandler
{
public:
    ESP8266ModbusHandler();

    bool begin();
    bool readRegisters(uint16_t startAddr, uint16_t numRegs, std::vector<uint16_t> &values, uint8_t slaveAddr = 0x11);
    bool writeRegister(uint16_t regAddr, uint16_t regValue, uint8_t slaveAddr = 0x11);

    uint16_t calculateCRC(const std::vector<uint8_t> &data);
    String modbusExceptionMessage(uint8_t code);

private:
    ESP8266ProtocolAdapter adapter_;
    unsigned long lastOperationTime_;
    
    bool waitForOperationTimeout();
    String buildReadFrame(uint8_t slaveAddr, uint16_t startAddr, uint16_t numRegs);
    String buildWriteFrame(uint8_t slaveAddr, uint16_t regAddr, uint16_t regValue);
    String bytesToHex(const std::vector<uint8_t> &bytes);
    std::vector<uint8_t> hexToBytes(const String &hex);

    // CRC table for Modbus CRC16
    static const uint16_t crc16_table[256];
};

#endif // ESP8266_MODBUS_HANDLER_H