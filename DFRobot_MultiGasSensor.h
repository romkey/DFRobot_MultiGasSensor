/*!
  * @file  DFRobot_MultiGasSensor.h
  * @brief This is a header file of the library for the sensor that can detect gas concentration in the air.
  * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  * @license     The MIT License (MIT)
  * @author      PengKaixing(kaixing.peng@dfrobot.com)
  * @version     V2.0.0
  * @date        2021-09-26
  * @url         https://github.com/DFRobot/DFRobot_MultiGasSensor
*/
#ifndef __DFRobot_GAS_H__
#define __DFRobot_GAS_H__

#include "Arduino.h"
#include <Wire.h>

#if (!defined ARDUINO_ESP32_DEV) && (!defined __SAMD21G18A__)
#include "SoftwareSerial.h"
#else
#include "HardwareSerial.h"
#endif

#define CMD_CHANGE_GET_METHOD          0X78
#define CMD_GET_GAS_CONCENTRATION      0X86
#define CMD_GET_TEMP                   0X87
#define CMD_GET_ALL_DTTA               0X88
#define CMD_SET_THRESHOLD_ALARMS       0X89
#define CMD_IIC_AVAILABLE              0X90
#define CMD_SENSOR_VOLTAGE             0X91
#define CMD_CHANGE_IIC_ADDR            0X92

#define DFGAS_PROTOCOL_LEN             9
#define DFGAS_PAYLOAD_LEN              6

// Uncomment to enable serial debug output from the library
//#define ENABLE_DBG

#ifdef ENABLE_DBG
#define DBG(...) {Serial.print("[");Serial.print(__FUNCTION__); Serial.print("(): "); Serial.print(__LINE__); Serial.print(" ] 0x"); Serial.println(__VA_ARGS__,HEX);}
#else
#define DBG(...)
#endif

/**
 * @struct sProtocol_t
 * @brief Data protocol package for communication
 */
typedef struct
{
  uint8_t head;
  uint8_t addr;
  uint8_t data[DFGAS_PAYLOAD_LEN];
  uint8_t check;
} sProtocol_t;

/**
 * @struct sAllData_t
 * @brief The struct used when getting all the data
 */
typedef struct
{
  uint8_t head;
  uint8_t cmd;
  uint8_t gasconcentration_h;
  uint8_t gasconcentration_l;
  uint8_t gastype;
  uint8_t gasconcentration_decimals;
  uint8_t temp_h;
  uint8_t temp_l;
  uint8_t check;
} sAllData_t;

/**
 * @struct sAllDataAnalysis_t
 * @brief All the parsed data
 */
typedef struct
{
  float gasconcentration;
  char gastype[8];
  float temp;
} sAllDataAnalysis_t;

class DFRobot_GAS
{
#ifdef DFGAS_HOST_TEST
  friend struct DFGasTestAccess;
#endif
public:
  /**
   * @enum eMethod_t
   * @brief Type of the data the sensor uploads
   */
  typedef enum
  {
    INITIATIVE = 0x03,
    PASSIVITY = 0x04
  } eMethod_t;

  /**
   * @enum eType_t
   * @brief Gas Type
   */
  typedef enum
  {
    O2 = 0x05,
    CO = 0x04,
    H2S = 0x03,
    NO2 = 0x2C,
    O3 = 0x2A,
    CL2 = 0x31,
    NH3 = 0x02,
    H2 = 0x06,
    HCL = 0X2E,
    SO2 = 0X2B,
    HF = 0x33,
    _PH3 = 0x45
  } eType_t;

  /**
   * @enum eSwitch_t
   * @brief Whether to enable ALA alarm function
   */
  typedef enum
  {
    SWITCH_ON = 0x01,
    SWITCH_OFF = 0x00
  } eSwitch_t;

  /**
   * @enum eALA_t
   * @brief High and low ALA alarm function
   */
  typedef enum
  {
    LOW_THRESHOLD_ALA = 0x00,
    HIGH_THRESHOLD_ALA = 0x01
  } eALA_t;

  DFRobot_GAS(void);
  ~DFRobot_GAS(void){};

  /**
   * @fn begin
   * @brief Parent class init, I2C or UART init is performed in subclass function
   * @return bool type, indicating whether init succeed
   */
  virtual bool begin(void) = 0;

  /**
   * @fn setResponseDelayMs
   * @brief Minimum wait before reading a command response
   */
  void setResponseDelayMs(uint16_t delayMs);

  /**
   * @fn changeAcquireMode
   * @brief Change the mode of acquiring sensor data
   */
  bool changeAcquireMode(eMethod_t mode);

  /**
   * @fn readGasConcentrationPPM
   * @brief Get gas concentration from sensor, unit PPM
   */
  float readGasConcentrationPPM(void);

  /**
   * @fn queryGasType
   * @brief Query gas type
   */
  String queryGasType(void);

  /**
   * @fn setThresholdAlarm
   * @brief Set sensor alarm threshold using a gas type string
   */
  bool setThresholdAlarm(eSwitch_t switchof, uint16_t threshold, eALA_t alamethod, const char *gasType);

  /**
   * @fn setThresholdAlarm
   * @brief Set sensor alarm threshold using a gas type enum
   */
  bool setThresholdAlarm(eSwitch_t switchof, uint16_t threshold, eALA_t alamethod, eType_t gasType);

  /**
   * @fn setThresholdAlarm
   * @brief Set sensor alarm threshold using an Arduino String
   */
  bool setThresholdAlarm(eSwitch_t switchof, uint16_t threshold, eALA_t alamethod, String gasType);

  /**
   * @fn readTempC
   * @brief Get sensor onboard temperature
   */
  float readTempC(void);

  /**
   * @fn setTempCompensation
   * @brief Set whether to turn on temperature compensation
   */
  void setTempCompensation(eSwitch_t tempswitch);

  /**
   * @fn pack
   * @brief Pack the protocol data for easy transmission
   */
  sProtocol_t pack(uint8_t *pBuf, uint8_t len);

  /**
   * @fn getSensorVoltage
   * @brief Get voltage output by sensor probe
   */
  float getSensorVoltage(void);

  /**
   * @fn dataIsAvailable
   * @brief Call this function in active mode to determine the presence of data on data line
   */
  virtual bool dataIsAvailable(void) = 0;

  /**
   * @fn changeI2cAddrGroup
   * @brief Change I2C address group
   */
  bool changeI2cAddrGroup(uint8_t group);

  /**
   * @fn getGasConcentration
   * @brief Parsed gas concentration from the last initiative-mode packet
   */
  float getGasConcentration(void) const;

  /**
   * @fn getGasType
   * @brief Parsed gas type from the last initiative-mode packet
   */
  const char *getGasType(void) const;

  /**
   * @fn getSensorTemperature
   * @brief Parsed temperature from the last initiative-mode packet
   */
  float getSensorTemperature(void) const;

protected:
  virtual void writeData(uint8_t Reg, void *Data, uint8_t len) = 0;
  virtual int16_t readData(uint8_t Reg, uint8_t *Data, uint8_t len) = 0;

  bool readResponse(uint8_t *recvbuf, uint8_t len);
  bool responseChecksumValid(const uint8_t *recvbuf, uint8_t len) const;
  bool readInitiativePacket(uint8_t *recvbuf, uint8_t len);
  bool storeAndAnalyzeInitiativePacket(const uint8_t *recvbuf, uint8_t len);
  void analysisAllData(void);

private:
  static const char *gasTypeFromCode(uint8_t code);
  static bool gasTypeUsesTenths(eType_t gasType);
  static void scaleThresholdForGasType(uint16_t &threshold, eType_t gasType);
  float applyTempCompensation(float concentration, uint8_t gasType, float temp) const;
  float adcToTempC(uint16_t tempAdc) const;

  sAllData_t _allData;
  sAllDataAnalysis_t _allDataAnalysis;
  bool _tempswitch;
  float _temp;
  uint16_t _responseDelayMs;
};

#ifdef DFGAS_HOST_TEST
struct DFGasTestAccess
{
  static uint8_t computeChecksum(uint8_t *data, uint8_t ln);
  static sProtocol_t pack(DFRobot_GAS &gas, uint8_t *pBuf, uint8_t len);
  static bool responseChecksumValid(const DFRobot_GAS &gas, const uint8_t *recvbuf, uint8_t len);
  static float applyTempCompensation(const DFRobot_GAS &gas, float concentration, uint8_t gasType, float temp);
  static float adcToTempC(const DFRobot_GAS &gas, uint16_t tempAdc);
  static const char *gasTypeFromCode(uint8_t code);
  static bool gasTypeUsesTenths(DFRobot_GAS::eType_t gasType);
  static void scaleThresholdForGasType(uint16_t &threshold, DFRobot_GAS::eType_t gasType);
  static bool storeAndAnalyzeInitiativePacket(DFRobot_GAS &gas, const uint8_t *recvbuf, uint8_t len);
  static bool readResponse(DFRobot_GAS &gas, uint8_t *recvbuf, uint8_t len);
  static float getGasConcentration(const DFRobot_GAS &gas);
  static const char *getGasType(const DFRobot_GAS &gas);
  static float getSensorTemperature(const DFRobot_GAS &gas);
};
#endif

class DFRobot_GAS_I2C : public DFRobot_GAS
{
  public:
    DFRobot_GAS_I2C(TwoWire *pWire=&Wire,uint8_t addr=0x74);
    ~DFRobot_GAS_I2C(void){};
    bool begin(void);
    void setI2cAddr(uint8_t addr);
    bool dataIsAvailable(void);
  protected:
    void writeData(uint8_t Reg ,void *Data ,uint8_t len);
    int16_t readData(uint8_t Reg ,uint8_t *Data ,uint8_t len);
  private:
    TwoWire* _pWire;
    uint8_t _I2C_addr;
};

#if (!defined ARDUINO_ESP32_DEV) && (!defined __SAMD21G18A__)
class DFRobot_GAS_SoftWareUart : public DFRobot_GAS
{
  public:
    DFRobot_GAS_SoftWareUart(SoftwareSerial *psoftUart);
    ~DFRobot_GAS_SoftWareUart(void){};
    bool begin(void);
    bool dataIsAvailable(void);
  protected:
    void writeData(uint8_t Reg ,void *Data ,uint8_t len);
    int16_t readData(uint8_t Reg ,uint8_t *Data ,uint8_t len);
  private:
    SoftwareSerial *_psoftUart;
};
#else
class DFRobot_GAS_HardWareUart : public DFRobot_GAS
{
  public:
    DFRobot_GAS_HardWareUart(HardwareSerial *phardUart);
    ~DFRobot_GAS_HardWareUart(void){};
    bool begin(void);
    bool dataIsAvailable(void);
  protected:
    void writeData(uint8_t Reg, void *Data, uint8_t len);
    int16_t readData(uint8_t Reg, uint8_t *Data, uint8_t len);
  private:
    HardwareSerial *_pharduart;
};

#endif
#endif
