#include "ESP8266PollingConfig.h"
#include "ESP8266Inverter.h"

// Static member initialization
ParameterConfig ESP8266PollingConfig::parameterConfigs_[10];
bool ESP8266PollingConfig::configsInitialized_ = false;

ESP8266PollingConfig::ESP8266PollingConfig()
{
    // Do not initialize parameter configs during static/global initialization.
    // Some platforms (ESP8266) don't have the heap ready yet which causes
    // Arduino String constructions to produce empty strings. We rely on
    // lazy initialization in getParameterConfig()/printEnabledParameters().
}

void ESP8266PollingConfig::initializeParameterConfigs()
{
    // Initialize parameter configurations
    parameterConfigs_[static_cast<int>(ParameterType::AC_VOLTAGE)] =
        {"AC Voltage", " V", ParameterReaders::readACVoltage};
    parameterConfigs_[static_cast<int>(ParameterType::AC_CURRENT)] =
        {"AC Current", " A", ParameterReaders::readACCurrent};
    parameterConfigs_[static_cast<int>(ParameterType::AC_FREQUENCY)] =
        {"AC Frequency", " Hz", ParameterReaders::readACFrequency};
    parameterConfigs_[static_cast<int>(ParameterType::PV1_VOLTAGE)] =
        {"PV1 Voltage", " V", ParameterReaders::readPV1Voltage};
    parameterConfigs_[static_cast<int>(ParameterType::PV2_VOLTAGE)] =
        {"PV2 Voltage", " V", ParameterReaders::readPV2Voltage};
    parameterConfigs_[static_cast<int>(ParameterType::PV1_CURRENT)] =
        {"PV1 Current", " A", ParameterReaders::readPV1Current};
    parameterConfigs_[static_cast<int>(ParameterType::PV2_CURRENT)] =
        {"PV2 Current", " A", ParameterReaders::readPV2Current};
    parameterConfigs_[static_cast<int>(ParameterType::TEMPERATURE)] =
        {"Temperature", " °C", ParameterReaders::readTemperature};
    parameterConfigs_[static_cast<int>(ParameterType::EXPORT_POWER_PERCENT)] =
        {"Export Power Percent", " %", ParameterReaders::readExportPowerPercent};
    parameterConfigs_[static_cast<int>(ParameterType::OUTPUT_POWER)] =
        {"Output Power", " W", ParameterReaders::readOutputPower};

    // Mark initialized only after strings are constructed at runtime
    configsInitialized_ = true;
}

void ESP8266PollingConfig::setParameters(const std::vector<ParameterType> &params)
{
    enabledParameters_ = params;
}

const ParameterConfig &ESP8266PollingConfig::getParameterConfig(ParameterType param) const
{
    // Ensure configs are initialized (lazy init). This avoids static-init order
    // or heap-not-ready issues if constructors ran too early.
    if (!configsInitialized_)
    {
        // cast away const to initialize once
        const_cast<ESP8266PollingConfig *>(this)->initializeParameterConfigs();
        const_cast<bool &>(configsInitialized_) = true;
    }
    return parameterConfigs_[static_cast<int>(param)];
}

void ESP8266PollingConfig::printEnabledParameters() const
{
    // Ensure parameter configs are initialized before printing
    if (!configsInitialized_)
    {
        const_cast<ESP8266PollingConfig *>(this)->initializeParameterConfigs();
        const_cast<bool &>(configsInitialized_) = true;
    }
    Serial.println("[POLLING] Enabled parameters:");
    for (ParameterType param : enabledParameters_)
    {
        const ParameterConfig &config = getParameterConfig(param);
        // Debug: print numeric index and string lengths to diagnose empty names
        Serial.print("  - [");
        Serial.print(static_cast<int>(param));
        Serial.print("] ");
        Serial.print(config.name);
        Serial.print(" (len=");
        Serial.print(config.name.length());
        Serial.print(") unit(len=");
        Serial.print(config.unit.length());
        Serial.print(") ");
        Serial.println(config.unit);
    }
}

bool ESP8266PollingConfig::isParameterEnabled(ParameterType param) const
{
    for (ParameterType enabledParam : enabledParameters_)
    {
        if (enabledParam == param)
            return true;
    }
    return false;
}

// Parameter reader functions
namespace ParameterReaders
{
    bool readACVoltage(ESP8266Inverter &inverter, float &value)
    {
        return inverter.getACVoltage(value);
    }

    bool readACCurrent(ESP8266Inverter &inverter, float &value)
    {
        return inverter.getACCurrent(value);
    }

    bool readACFrequency(ESP8266Inverter &inverter, float &value)
    {
        return inverter.getACFrequency(value);
    }

    bool readPV1Voltage(ESP8266Inverter &inverter, float &value)
    {
        return inverter.getPV1Voltage(value);
    }

    bool readPV2Voltage(ESP8266Inverter &inverter, float &value)
    {
        return inverter.getPV2Voltage(value);
    }

    bool readPV1Current(ESP8266Inverter &inverter, float &value)
    {
        return inverter.getPV1Current(value);
    }

    bool readPV2Current(ESP8266Inverter &inverter, float &value)
    {
        return inverter.getPV2Current(value);
    }

    bool readTemperature(ESP8266Inverter &inverter, float &value)
    {
        return inverter.getTemperature(value);
    }

    bool readExportPowerPercent(ESP8266Inverter &inverter, float &value)
    {
        int intValue;
        if (inverter.getExportPowerPercent(intValue))
        {
            value = static_cast<float>(intValue);
            return true;
        }
        return false;
    }

    bool readOutputPower(ESP8266Inverter &inverter, float &value)
    {
        int intValue;
        if (inverter.getOutputPower(intValue))
        {
            value = static_cast<float>(intValue);
            return true;
        }
        return false;
    }
}