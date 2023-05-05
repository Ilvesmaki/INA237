/**
 * @file INA237.cpp
 * @author Wilho-Pekka Ilvesmäki (wilho.ilvesmaki@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-25
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "INA237.h"

INA237::INA237(TwoWire* i2c_handler, uint8_t addr):_i2c(i2c_handler),_device_address(addr)
{
    
}

INA237::~INA237()
{
}

void INA237::reset(void)
{
    _writeRegister(INA237_REG_CONFIG, ((uint16_t)1<<15));
    _current_lsb = 0;
    _adc_range = INA237_ADC_RANGE_163_84mV;
}

void INA237::calibrate(double res, double max_current, bool rounded)
{
    const float COEFF = 819.2E6F;
    double new_current_lsb = (max_current != 0.0F) ? ldexp(abs(max_current), -15) : 0.0F;
    if(rounded)
    {
        uint32_t multiplier = 1;
        double rounded_current_lsb = new_current_lsb;
        while (rounded_current_lsb < 1.0F)
        {
            multiplier *= 10;
            rounded_current_lsb = new_current_lsb * multiplier;
        }

        new_current_lsb = (ceil(rounded_current_lsb) / multiplier);
    }
    _current_lsb = new_current_lsb;
    uint16_t shunt_cal = (uint16_t)(COEFF * new_current_lsb * abs(res) * (_adc_range?4:1));
    _writeRegister(INA237_REG_SHUNT_CAL, shunt_cal);
}

bool INA237::configADC(ina237_adc_config_t config)
{
    if(config.mode > 0xF || config.vbusct > 7 || config.vshct > 7 || \
    config.vtct > 7 || config.avg > 7)
    {
        return false;
    }
    uint16_t data = (config.mode << 12) | (config.vbusct << 9) | \
                    (config.vshct << 6) | (config.vtct << 3) | (config.avg);
    
    _writeRegister(INA237_REG_ADC_CONFIG, data);

    return true;
}

void INA237::configAlert(ina237_alrt_config_t config)
{
    uint8_t reg[2];
    uint16_t regval = 0;
    _readRegister(INA237_REG_DIAG_ALRT, 2, &reg[0]);
    regval = 0x2FD & ((reg[1]<<8) | reg[0]);
    regval |= (config.alatch << 15) | (config.cnvr << 14) | \
            (config.slowalert << 13) | (config.apol << 12);
    _writeRegister(INA237_REG_DIAG_ALRT, regval);
}

void INA237::setConversionDelay(uint8_t delay)
{
    uint8_t val[2];
    uint16_t new_val;
    _readRegister(INA237_REG_CONFIG, 2, &val[0]);

    new_val = ((uint16_t)(0x3F & val[1])<<8) | (0x3F & val[0]);
    new_val |= (delay<<6);

    _writeRegister(INA237_REG_CONFIG, new_val);
}

void INA237::setADCRange(uint8_t range)
{
    if(range != INA237_ADC_RANGE_40_96mV || range != INA237_ADC_RANGE_163_84mV)
    {
        return;
    }

    _adc_range = range;

    uint8_t val[2];
    uint16_t new_val;
    _readRegister(INA237_REG_CONFIG, 2, &val[0]);
    
    new_val = ((uint16_t)(0xFF & val[1])<<8) | (0xEF & val[0]);
    new_val |= (range<<4);

    _writeRegister(INA237_REG_CONFIG, new_val);
}

void INA237::setShuntOvervoltageTreshold(double voltage)
{
    double value = voltage / INA237_VSHUNT_LSB_RES[_adc_range];
    int16_t regval = (value > INT16_MAX ? INT16_MAX : \
                     (value < INT16_MIN ? INT16_MIN : (int16_t)value));

    _writeRegister(INA237_REG_SOVL, regval);
}

void INA237::setShuntUndervoltageTreshold(double voltage)
{
    double value = voltage / INA237_VSHUNT_LSB_RES[_adc_range];
    int16_t regval = (value > INT16_MAX ? INT16_MAX : \
                     (value < INT16_MIN ? INT16_MIN : (int16_t)value));

    _writeRegister(INA237_REG_SUVL, regval);
}

void INA237::setBusOvervoltageTreshold(double voltage)
{
    double value = voltage / INA237_VBUS_LSB_RES;
    uint16_t regval = (value > 0x7FFF ? 0x7FFF : \
                     (value < 0 ? 0 : (uint16_t)value));

    _writeRegister(INA237_REG_BOVL, regval);
}

void INA237::setBusUndervoltageTreshold(double voltage)
{
    double value = voltage / INA237_VBUS_LSB_RES;
    uint16_t regval = (value > 0x7FFF ? 0x7FFF : \
                     (value < 0 ? 0 : (uint16_t)value));

    _writeRegister(INA237_REG_BUVL, regval);
}

void INA237::setTempOverlimitTreshold(double temp)
{
    temp = temp > 255 ? 255 : (temp < -255 ? -255 : temp);
    double value = temp / INA237_TEMP_LSB_RES;
    int16_t regval = ((int16_t)round(value) << 4) & 0xFFF0;
    
    _writeRegister(INA237_REG_TEMP_LIMIT, (uint16_t)regval);
}

void INA237::setPowerOverlimitTreshold(double power)
{
    double divider = (_current_lsb * 0.2 * 256);
    double value = divider != 0.0 ? ( power / divider) : 0.0;
    uint16_t regval = (value > UINT16_MAX ? UINT16_MAX : \
                     (value < 0 ? 0 : (uint16_t)round(value)));

    _writeRegister(INA237_REG_PWR_LIMIT, (uint16_t)regval);
}

uint16_t INA237::getAlertFlag(void)
{
    uint8_t reg[2];
    uint16_t regval;
    _readRegister(INA237_REG_DIAG_ALRT, 2, &reg[0]);
    regval = 0x2FF & ((reg[1]<<8) | reg[0]);
    return regval;
}

double INA237::getBusVoltage(void)
{
    uint8_t arr[2];
    int16_t voltage = 0;
    _readRegister(INA237_REG_VBUS, 2, &arr[0]);
    voltage = (int16_t)(((uint16_t)arr[1] << 8) | ((uint16_t)arr[0]));
    
    return voltage * INA237_VBUS_LSB_RES;
}

double INA237::getShuntVoltage(void)
{
    uint8_t arr[2];
    int16_t voltage = 0;
    _readRegister(INA237_REG_VSHUNT, 2, &arr[0]);
    voltage = (int16_t)(((uint16_t)arr[1] << 8) | ((uint16_t)arr[0]));
    
    return voltage * INA237_VSHUNT_LSB_RES[_adc_range];
}

double INA237::getCurrent(void)
{
    uint8_t arr[2];
    int16_t current = 0;
    _readRegister(INA237_REG_CURRENT, 2, &arr[0]);
    current = (int16_t)(((uint16_t)arr[1] << 8) | ((uint16_t)arr[0]));

    return current * _current_lsb;
}

double INA237::getPower(void)
{
    uint8_t arr[3];
    uint32_t power = 0;
    _readRegister(INA237_REG_POWER, 3, &arr[0]);
    power = ((uint32_t)arr[2] << 16) | ((uint32_t)arr[1] << 8) | ((uint32_t)arr[0]);

    return (double)power * _current_lsb * 0.2;
}

double INA237::getTemp(void)
{
    uint8_t arr[2];
    int16_t temp = 0;
    _readRegister(INA237_REG_DIETEMP, 2, &arr[0]);
    temp = (int16_t)(((uint16_t)arr[1] << 8) | ((uint16_t)arr[0]));
    temp >>= 4;

    return (double)temp * INA237_TEMP_LSB_RES;
}

double INA237::getShuntOvervoltageTreshold(void)
{
    uint8_t arr[2];
    int16_t value = 0;
    _readRegister(INA237_REG_SOVL, 2, &arr[0]);
    value = (int16_t)(((uint16_t)arr[1] << 8) | ((uint16_t)arr[0]));

    return (double)value * INA237_VSHUNT_LSB_RES[_adc_range];
}

double INA237::getShuntUndervoltageTreshold(void)
{
    uint8_t arr[2];
    int16_t value = 0;
    _readRegister(INA237_REG_SUVL, 2, &arr[0]);
    value = (int16_t)(((uint16_t)arr[1] << 8) | ((uint16_t)arr[0]));

    return (double)value * INA237_VSHUNT_LSB_RES[_adc_range];
}

double INA237::getBusOvervoltageTreshold(void)
{
    uint8_t arr[2];
    uint16_t value = 0;
    _readRegister(INA237_REG_BOVL, 2, &arr[0]);
    value = (((uint16_t)arr[1] << 8) | ((uint16_t)arr[0]));

    return (double)value * INA237_VBUS_LSB_RES;
}

double INA237::getBusUndervoltageTreshold(void)
{
    uint8_t arr[2];
    uint16_t value = 0;
    _readRegister(INA237_REG_BUVL, 2, &arr[0]);
    value = (((uint16_t)arr[1] << 8) | ((uint16_t)arr[0]));

    return (double)value * INA237_VBUS_LSB_RES;
}

double INA237::getTempOverlimitTreshold(void)
{
    uint8_t arr[2];
    int16_t value = 0;
    _readRegister(INA237_REG_TEMP_LIMIT, 2, &arr[0]);
    value = (int16_t)(((uint16_t)arr[1] << 8) | ((uint16_t)arr[0]));
    value >>= 4;

    return (double)value * INA237_TEMP_LSB_RES;
}

double INA237::getPowerOverlimitTreshold(void)
{
    uint8_t arr[2];
    uint32_t value = 0;
    _readRegister(INA237_REG_PWR_LIMIT, 2, &arr[0]);
    value = (((uint32_t)arr[1] << 8) | ((uint32_t)arr[0]));
    value <<= 8; // multiply by 256

    return (double)value * 0.2 * _current_lsb;
}

uint16_t INA237::getManufacturerID(void)
{
    uint8_t arr[2];
    uint16_t value = 0;
    _readRegister(INA237_REG_MANUFACTURER_ID, 2, &arr[0]);
    value = (((uint16_t)arr[1] << 8) | ((uint16_t)arr[0]));

    return value;
}

/*----------PRIVATE FUNCTIONS-------------------------------------------------*/

void INA237::_readRegister(uint8_t reg, uint8_t cnt, uint8_t *data)
{
    /* set device's register pointer to correct register before read */
    _i2c->beginTransmission(_device_address);
    _i2c->write(reg);
    _i2c->endTransmission(true);

    uint8_t read = _i2c->requestFrom(_device_address, cnt, true);
    if(read != cnt)
    {
        return;
    }
    while(cnt > _i2c->available());

    for(int i = (cnt - 1); i >= 0; i--)
    {
        int val = _i2c->read();
        if(val == -1)
        {
            // failed
            return;
        }
        data[i] = (uint8_t)val;
    }
}

void INA237::_writeRegister(uint8_t reg, uint16_t data)
{
    _i2c->beginTransmission(_device_address);
    _i2c->write(reg);
    _i2c->write((uint8_t)(data>>8));
    _i2c->write((uint8_t)data);
    _i2c->endTransmission(true);
}

