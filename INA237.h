/**
 * @file INA237.h
 * @author Wilho-Pekka Ilvesmäki (wilho.ilvesmaki@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-25
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __INA237__H__
#define __INA237__H__

#include <stdint.h>
#include <math.h>
#include "Wire.h"

/* define register addresses
All registers hold 2 octets of data except power register that holds 3 octets*/
#define INA237_REG_CONFIG                       0x00
#define INA237_REG_ADC_CONFIG                   0x01
#define INA237_REG_SHUNT_CAL                    0x02
#define INA237_REG_VSHUNT                       0x04
#define INA237_REG_VBUS                         0x05
#define INA237_REG_DIETEMP                      0x06
#define INA237_REG_CURRENT                      0x07
#define INA237_REG_POWER                        0x08
#define INA237_REG_DIAG_ALRT                    0x0B
#define INA237_REG_SOVL                         0x0C
#define INA237_REG_SUVL                         0x0D
#define INA237_REG_BOVL                         0x0E
#define INA237_REG_BUVL                         0x0F
#define INA237_REG_TEMP_LIMIT                   0x10
#define INA237_REG_PWR_LIMIT                    0x11
#define INA237_REG_MANUFACTURER_ID              0x3E
/*----------CONFIG DEFINES----------------------------------------------------*/
#define INA237_ADC_RANGE_163_84mV               0x0
#define INA237_ADC_RANGE_40_96mV                0x1

const double INA237_VSHUNT_LSB_RES[2] = {5E-6, 1.25E-6};
const double INA237_VBUS_LSB_RES = 3.125E-3;
const double INA237_TEMP_LSB_RES = 125E-3;

/*----------ADC CONFIG DEFINES------------------------------------------------*/
/* define operating modes*/
#define INA237_MODE_SHUTDOWN_1              0x0
#define INA237_MODE_TRIG_VBUS               0x1
#define INA237_MODE_TRIG_VSHUNT             0x2
#define INA237_MODE_TRIG_VBUS_VSHUNT        0x3
#define INA237_MODE_TRIG_TEMP               0x4
#define INA237_MODE_TRIG_TEMP_VBUS          0x5
#define INA237_MODE_TRIG_TEMP_VSHUNT        0x6
#define INA237_MODE_TRIG_TEMP_VBUS_VSHUNT   0x7
#define INA237_MODE_SHUTDOWN_2              0x8
#define INA237_MODE_CONT_VBUS               0x9
#define INA237_MODE_CONT_VSHUNT             0xA
#define INA237_MODE_CONT_VBUS_VSHUNT        0xB
#define INA237_MODE_CONT_TEMP               0xC
#define INA237_MODE_CONT_TEMP_VBUS          0xD
#define INA237_MODE_CONT_TEMP_VSHUNT        0xE
#define INA237_MODE_CONT_TEMP_VBUS_VSHUNT   0xF

/* define conversion times for VBUS, VSHUNT and TEMP */
#define INA237_CT_50US                      0x0
#define INA237_CT_84US                      0x1
#define INA237_CT_150US                     0x2
#define INA237_CT_280US                     0x3
#define INA237_CT_540US                     0x4
#define INA237_CT_1052US                    0x5
#define INA237_CT_2074US                    0x6
#define INA237_CT_4120US                    0x7

/* define AVG counts */
// defines for avg samples
#define INA237_AVG_1                        0x0
#define INA237_AVG_4                        0x1
#define INA237_AVG_16                       0x2
#define INA237_AVG_64                       0x3
#define INA237_AVG_128                      0x4
#define INA237_AVG_256                      0x5
#define INA237_AVG_512                      0x6
#define INA237_AVG_1024                     0x7

typedef struct
{
    uint16_t mode;
    uint16_t vbusct;
    uint16_t vshct;
    uint16_t vtct;
    uint16_t avg;
}ina237_adc_config_t;

const ina237_adc_config_t INA237_DEFAULT_ADC_CONFIG = 
{INA237_MODE_CONT_TEMP_VBUS_VSHUNT,
 INA237_CT_1052US,
 INA237_CT_1052US,
 INA237_CT_1052US,
 INA237_AVG_1};

/*----------DIAG_ALRT CONFIGS-------------------------------------------------*/

#define INA237_ALERT_MATH_OF    0x200
#define INA237_ALERT_TEMP_OL    0x80
#define INA237_ALERT_SHUNT_OL   0x40
#define INA237_ALERT_SHUNT_UL   0x20
#define INA237_ALERT_BUS_OL     0x10
#define INA237_ALERT_BUS_UL     0x8
#define INA237_ALERT_POW_OL     0x4
#define INA237_ALERT_CNVRF      0x2
#define INA237_ALERT_MEMSTAT    0x1

typedef struct
{
    bool alatch;
    bool cnvr;
    bool slowalert;
    bool apol;
}ina237_alrt_config_t;

class INA237
{
public:
    INA237(TwoWire* i2c_handler, uint8_t addr = 0x40);
    ~INA237();

    void reset(void);

    void calibrate(double res, double max_current, bool rounded = true);
    
    bool configADC(ina237_adc_config_t config = INA237_DEFAULT_ADC_CONFIG);
    void configAlert(ina237_alrt_config_t config);

    /* setters */
    void setConversionDelay(uint8_t delay);
    void setADCRange(uint8_t);
    void setShuntOvervoltageTreshold(double voltage);
    void setShuntUndervoltageTreshold(double voltage);
    void setBusOvervoltageTreshold(double voltage);
    void setBusUndervoltageTreshold(double voltage);
    void setTempOverlimitTreshold(double temp);
    void setPowerOverlimitTreshold(double power);

    /* getters */
    uint16_t getAlertFlag(void);
    double getBusVoltage(void);
    double getShuntVoltage(void);
    double getCurrent(void);
    double getPower(void);
    double getTemp(void);
    double getShuntOvervoltageTreshold(void);
    double getShuntUndervoltageTreshold(void);
    double getBusOvervoltageTreshold(void);
    double getBusUndervoltageTreshold(void);
    double getTempOverlimitTreshold(void);
    double getPowerOverlimitTreshold(void);
    uint16_t getManufacturerID(void);

private:
    void _readRegister(uint8_t reg, uint8_t cnt, uint8_t *data);
    void _writeRegister(uint8_t reg, uint16_t data);
    void _writeRegister(uint8_t reg, uint8_t cnt, uint8_t *data);

    /* Private variables */
    uint8_t _device_address = 0x40;
    uint8_t _adc_range = INA237_ADC_RANGE_163_84mV;
    double _current_lsb;
    Wire* _i2c = NULL;
};


#endif  //!__INA237__H__