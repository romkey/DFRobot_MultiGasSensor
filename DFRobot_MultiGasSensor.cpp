/*!
  * @file  DFRobot_MultiGasSensor.cpp
  * @brief This is function implementation .cpp file of a library for the sensor that can detect gas concentration in the air.
  * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  * @license     The MIT License (MIT)
  * @author      PengKaixing(kaixing.peng@dfrobot.com)
  * @version     V2.0.0
  * @date        2021-09-26
  * @url         https://github.com/DFRobot/DFRobot_MultiGasSensor
*/
#include "DFRobot_MultiGasSensor.h"
#include <string.h>

static uint8_t FucCheckSum(uint8_t *data, uint8_t ln)
{
  uint8_t sum = 0;
  data += 1;
  for (uint8_t j = 0; j < (ln - 2); j++)
  {
    sum += *data;
    data++;
  }
  return (~sum) + 1;
}

DFRobot_GAS::DFRobot_GAS(void)
  : _tempswitch(SWITCH_OFF), _temp(0.0f), _responseDelayMs(100)
{
  memset(&_allData, 0, sizeof(_allData));
  memset(&_allDataAnalysis, 0, sizeof(_allDataAnalysis));
}

void DFRobot_GAS::setResponseDelayMs(uint16_t delayMs)
{
  _responseDelayMs = delayMs;
}

bool DFRobot_GAS::readResponse(uint8_t *recvbuf, uint8_t len)
{
  if (_responseDelayMs > 0)
    delay(_responseDelayMs);
  return readData(0, recvbuf, len) == len;
}

bool DFRobot_GAS::responseChecksumValid(const uint8_t *recvbuf, uint8_t len) const
{
  if (len < DFGAS_PROTOCOL_LEN)
    return false;
  return FucCheckSum((uint8_t *)recvbuf, 8) == recvbuf[8];
}

bool DFRobot_GAS::readInitiativePacket(uint8_t *recvbuf, uint8_t len)
{
  if (readData(0, recvbuf, len) != len)
    return false;
  return responseChecksumValid(recvbuf, len);
}

bool DFRobot_GAS::storeAndAnalyzeInitiativePacket(const uint8_t *recvbuf, uint8_t len)
{
  if (!responseChecksumValid(recvbuf, len))
    return false;
  memcpy(&_allData, recvbuf, len);
  analysisAllData();
  return true;
}

float DFRobot_GAS::getGasConcentration(void) const
{
  return _allDataAnalysis.gasconcentration;
}

const char *DFRobot_GAS::getGasType(void) const
{
  return _allDataAnalysis.gastype;
}

float DFRobot_GAS::getSensorTemperature(void) const
{
  return _allDataAnalysis.temp;
}

const char *DFRobot_GAS::gasTypeFromCode(uint8_t code)
{
  switch (code)
  {
    case DFRobot_GAS::O2:   return "O2";
    case DFRobot_GAS::CO:   return "CO";
    case DFRobot_GAS::H2S:  return "H2S";
    case DFRobot_GAS::NO2:  return "NO2";
    case DFRobot_GAS::O3:   return "O3";
    case DFRobot_GAS::CL2:  return "CL2";
    case DFRobot_GAS::NH3:  return "NH3";
    case DFRobot_GAS::H2:   return "H2";
    case DFRobot_GAS::HCL:  return "HCL";
    case DFRobot_GAS::SO2:  return "SO2";
    case DFRobot_GAS::HF:   return "HF";
    case DFRobot_GAS::_PH3: return "PH3";
    default:                return "";
  }
}

bool DFRobot_GAS::gasTypeUsesTenths(eType_t gasType)
{
  switch (gasType)
  {
    case O2:
    case NO2:
    case O3:
    case CL2:
    case HCL:
    case SO2:
    case HF:
    case _PH3:
      return true;
    default:
      return false;
  }
}

void DFRobot_GAS::scaleThresholdForGasType(uint16_t &threshold, eType_t gasType)
{
  if (gasTypeUsesTenths(gasType))
    threshold *= 10;
}

float DFRobot_GAS::adcToTempC(uint16_t tempAdc) const
{
  if (tempAdc >= 1023)
    return 0.0f;

  float vpd3 = 3.0f * (float)tempAdc / 1024.0f;
  float denom = 3.0f - vpd3;
  if (denom <= 0.001f)
    return 0.0f;

  float rth = vpd3 * 10000.0f / denom;
  return 1.0f / (1.0f / (273.15f + 25.0f) + 1.0f / 3380.13f * log(rth / 10000.0f)) - 273.15f;
}

float DFRobot_GAS::applyTempCompensation(float concentration, uint8_t gasType, float temp) const
{
  float con = concentration;

  switch (gasType)
  {
    case O2:
    case HCL:
    case SO2:
      break;
    case CO:
      if (temp >= -20.0f && temp <= 20.0f)
        con = con / (0.005f * temp + 0.9f);
      else if (temp > 20.0f && temp <= 40.0f)
        con = con / (0.005f * temp + 0.9f) - (0.3f * temp - 6.0f);
      else
        con = 0.0f;
      break;
    case H2S:
      if (temp >= -20.0f && temp <= 20.0f)
        con = con / (0.006f * temp + 0.92f);
      else if (temp > 20.0f && temp <= 40.0f)
        con = con / (0.006f * temp + 0.92f) - (0.015f * temp + 2.4f);
      else
        con = 0.0f;
      break;
    case NO2:
      if (temp >= -20.0f && temp < 0.0f)
        con = con / (0.005f * temp + 0.9f) - (-0.0025f * temp);
      else if (temp >= 0.0f && temp <= 20.0f)
        con = con / (0.005f * temp + 0.9f) - (0.005f * temp + 0.005f);
      else if (temp > 20.0f && temp <= 40.0f)
        con = con / (0.005f * temp + 0.9f) - (0.0025f * temp + 0.1f);
      else
        con = 0.0f;
      break;
    case O3:
      if (temp >= -20.0f && temp < 0.0f)
        con = con / (0.015f * temp + 1.1f) - 0.05f;
      else if (temp >= 0.0f && temp <= 20.0f)
        con = con / 1.1f - (0.01f * temp);
      else if (temp > 20.0f && temp <= 40.0f)
        con = con / 1.1f - (-0.05f * temp + 0.3f);
      else
        con = 0.0f;
      break;
    case CL2:
      if (temp >= -20.0f && temp < 0.0f)
        con = con / (0.015f * temp + 1.1f) - (-0.0025f * temp);
      else if (temp >= 0.0f && temp <= 20.0f)
        con = con / 1.1f - 0.005f * temp;
      else if (temp > 20.0f && temp <= 40.0f)
        con = con / 1.1f - (0.06f * temp - 0.12f);
      else
        con = 0.0f;
      break;
    case NH3:
      if (temp >= -20.0f && temp <= 0.0f)
        con = con / (0.08f * temp + 3.98f) - (-0.005f * temp + 0.3f);
      else if (temp > 0.0f && temp <= 20.0f)
        con = con / (0.08f * temp + 3.98f) - (-0.005f * temp + 0.3f);
      else if (temp > 20.0f && temp <= 40.0f)
        con = con / (0.004f * temp + 1.08f) - (-0.1f * temp + 2.0f);
      else
        con = 0.0f;
      break;
    case H2:
      if (temp >= -20.0f && temp <= 40.0f)
        con = con / (0.74f * temp + 0.007f) - 5.0f;
      else
        con = 0.0f;
      break;
    case HF:
      if (temp >= -20.0f && temp < 0.0f)
        con = con - (-0.0025f * temp);
      else if (temp >= 0.0f && temp <= 20.0f)
        con = con + 0.1f;
      else if (temp > 20.0f && temp <= 40.0f)
        con = con - (0.0375f * temp - 0.85f);
      else
        con = 0.0f;
      break;
    case _PH3:
      if (temp >= -20.0f && temp <= 40.0f)
        con = con / (0.005f * temp + 0.9f);
      else
        con = 0.0f;
      break;
    default:
      break;
  }

  return (con >= 0.0f) ? con : 0.0f;
}

void DFRobot_GAS::analysisAllData(void)
{
  if (!responseChecksumValid((uint8_t *)&_allData, DFGAS_PROTOCOL_LEN))
    return;

  uint16_t rawCon = (_allData.gasconcentration_h << 8) + _allData.gasconcentration_l;
  switch (_allData.gasconcentration_decimals)
  {
    case 1:
      _allDataAnalysis.gasconcentration = 0.1f * rawCon;
      break;
    case 2:
      _allDataAnalysis.gasconcentration = 0.01f * rawCon;
      break;
    default:
      _allDataAnalysis.gasconcentration = rawCon;
      break;
  }

  _temp = adcToTempC((_allData.temp_h << 8) + _allData.temp_l);
  if (_tempswitch == SWITCH_ON)
    _allDataAnalysis.gasconcentration = applyTempCompensation(_allDataAnalysis.gasconcentration, _allData.gastype, _temp);

  strncpy(_allDataAnalysis.gastype, gasTypeFromCode(_allData.gastype), sizeof(_allDataAnalysis.gastype) - 1);
  _allDataAnalysis.gastype[sizeof(_allDataAnalysis.gastype) - 1] = '\0';
  _allDataAnalysis.temp = _temp;
}

sProtocol_t DFRobot_GAS::pack(uint8_t *pBuf, uint8_t len)
{
  sProtocol_t protocol;
  protocol.head = 0xff;
  protocol.addr = 0x01;
  memset(protocol.data, 0, sizeof(protocol.data));
  if (len > sizeof(protocol.data))
    len = sizeof(protocol.data);
  memcpy(protocol.data, pBuf, len);
  protocol.check = FucCheckSum((uint8_t *)&protocol, 8);
  return protocol;
}

bool DFRobot_GAS::changeAcquireMode(eMethod_t mode)
{
  uint8_t buf[DFGAS_PAYLOAD_LEN] = {0};
  uint8_t recvbuf[DFGAS_PROTOCOL_LEN] = {0};
  buf[0] = CMD_CHANGE_GET_METHOD;
  buf[1] = mode;
  sProtocol_t protocol = pack(buf, sizeof(buf));
  writeData(0, (uint8_t *)&protocol, sizeof(protocol));
  if (!readResponse(recvbuf, sizeof(recvbuf)))
    return false;
  if (!responseChecksumValid(recvbuf, sizeof(recvbuf)))
    return false;
  return recvbuf[2] == 1;
}

float DFRobot_GAS::readGasConcentrationPPM(void)
{
  uint8_t buf[DFGAS_PAYLOAD_LEN] = {0};
  uint8_t recvbuf[DFGAS_PROTOCOL_LEN] = {0};
  buf[0] = CMD_GET_GAS_CONCENTRATION;
  sProtocol_t protocol = pack(buf, sizeof(buf));
  writeData(0, (uint8_t *)&protocol, sizeof(protocol));
  if (!readResponse(recvbuf, sizeof(recvbuf)))
    return 0.0f;
  if (!responseChecksumValid(recvbuf, sizeof(recvbuf)))
    return 0.0f;

  float con = (float)((recvbuf[2] << 8) + recvbuf[3]);
  switch (recvbuf[5])
  {
    case 1:
      con *= 0.1f;
      break;
    case 2:
      con *= 0.01f;
      break;
    default:
      break;
  }

  if (_tempswitch == SWITCH_ON)
  {
    _temp = readTempC();
    con = applyTempCompensation(con, recvbuf[4], _temp);
  }

  return (con < 0.0f) ? 0.0f : con;
}

String DFRobot_GAS::queryGasType(void)
{
  uint8_t buf[DFGAS_PAYLOAD_LEN] = {0};
  uint8_t recvbuf[DFGAS_PROTOCOL_LEN] = {0};
  buf[0] = CMD_GET_GAS_CONCENTRATION;
  sProtocol_t protocol = pack(buf, sizeof(buf));
  writeData(0, (uint8_t *)&protocol, sizeof(protocol));
  if (!readResponse(recvbuf, sizeof(recvbuf)))
    return "NO GAS";
  if (!responseChecksumValid(recvbuf, sizeof(recvbuf)))
    return "NO GAS";

  const char *type = gasTypeFromCode(recvbuf[4]);
  if (type[0] == '\0')
    return "NO GAS";
  return String(type);
}

bool DFRobot_GAS::setThresholdAlarm(eSwitch_t switchof, uint16_t threshold, eALA_t alamethod, eType_t gasType)
{
  scaleThresholdForGasType(threshold, gasType);

  uint8_t buf[DFGAS_PAYLOAD_LEN] = {0};
  uint8_t recvbuf[DFGAS_PROTOCOL_LEN] = {0};
  buf[0] = CMD_SET_THRESHOLD_ALARMS;
  buf[1] = switchof;
  buf[2] = threshold >> 8;
  buf[3] = threshold;
  buf[4] = alamethod;
  sProtocol_t protocol = pack(buf, sizeof(buf));
  writeData(0, (uint8_t *)&protocol, sizeof(protocol));
  if (!readResponse(recvbuf, sizeof(recvbuf)))
    return false;
  if (!responseChecksumValid(recvbuf, sizeof(recvbuf)))
    return false;
  return recvbuf[2] == 1;
}

bool DFRobot_GAS::setThresholdAlarm(eSwitch_t switchof, uint16_t threshold, eALA_t alamethod, const char *gasType)
{
  if (gasType == NULL)
    return setThresholdAlarm(switchof, threshold, alamethod, (eType_t)0);

  if (strcmp(gasType, "O2") == 0)
    return setThresholdAlarm(switchof, threshold, alamethod, O2);
  if (strcmp(gasType, "CO") == 0)
    return setThresholdAlarm(switchof, threshold, alamethod, CO);
  if (strcmp(gasType, "H2S") == 0)
    return setThresholdAlarm(switchof, threshold, alamethod, H2S);
  if (strcmp(gasType, "NO2") == 0)
    return setThresholdAlarm(switchof, threshold, alamethod, NO2);
  if (strcmp(gasType, "O3") == 0)
    return setThresholdAlarm(switchof, threshold, alamethod, O3);
  if (strcmp(gasType, "CL2") == 0)
    return setThresholdAlarm(switchof, threshold, alamethod, CL2);
  if (strcmp(gasType, "NH3") == 0)
    return setThresholdAlarm(switchof, threshold, alamethod, NH3);
  if (strcmp(gasType, "H2") == 0)
    return setThresholdAlarm(switchof, threshold, alamethod, H2);
  if (strcmp(gasType, "HCL") == 0)
    return setThresholdAlarm(switchof, threshold, alamethod, HCL);
  if (strcmp(gasType, "SO2") == 0)
    return setThresholdAlarm(switchof, threshold, alamethod, SO2);
  if (strcmp(gasType, "HF") == 0)
    return setThresholdAlarm(switchof, threshold, alamethod, HF);
  if (strcmp(gasType, "PH3") == 0)
    return setThresholdAlarm(switchof, threshold, alamethod, _PH3);

  return setThresholdAlarm(switchof, threshold, alamethod, (eType_t)0);
}

bool DFRobot_GAS::setThresholdAlarm(eSwitch_t switchof, uint16_t threshold, eALA_t alamethod, String gasType)
{
  return setThresholdAlarm(switchof, threshold, alamethod, gasType.c_str());
}

float DFRobot_GAS::readTempC(void)
{
  uint8_t buf[DFGAS_PAYLOAD_LEN] = {0};
  uint8_t recvbuf[DFGAS_PROTOCOL_LEN] = {0};
  buf[0] = CMD_GET_TEMP;
  sProtocol_t protocol = pack(buf, sizeof(buf));
  writeData(0, (uint8_t *)&protocol, sizeof(protocol));
  if (!readResponse(recvbuf, sizeof(recvbuf)))
    return 0.0f;
  if (!responseChecksumValid(recvbuf, sizeof(recvbuf)))
    return 0.0f;

  return adcToTempC((recvbuf[2] << 8) + recvbuf[3]);
}

void DFRobot_GAS::setTempCompensation(eSwitch_t tempswitch)
{
  _tempswitch = tempswitch;
  _temp = readTempC();
}

float DFRobot_GAS::getSensorVoltage(void)
{
  uint8_t buf[DFGAS_PAYLOAD_LEN] = {0};
  uint8_t recvbuf[DFGAS_PROTOCOL_LEN] = {0};
  buf[0] = CMD_SENSOR_VOLTAGE;
  sProtocol_t protocol = pack(buf, sizeof(buf));
  writeData(0, (uint8_t *)&protocol, sizeof(protocol));
  if (!readResponse(recvbuf, sizeof(recvbuf)))
    return 0.0f;
  if (!responseChecksumValid(recvbuf, sizeof(recvbuf)))
    return 0.0f;

  return ((uint16_t)((recvbuf[2] << 8) + recvbuf[3]) * 3.0f / 1024.0f * 2.0f);
}

bool DFRobot_GAS::changeI2cAddrGroup(uint8_t group)
{
  uint8_t buf[DFGAS_PAYLOAD_LEN] = {0};
  uint8_t recvbuf[DFGAS_PROTOCOL_LEN] = {0};
  buf[0] = CMD_CHANGE_IIC_ADDR;
  buf[1] = group;
  sProtocol_t protocol = pack(buf, sizeof(buf));
  writeData(0, (uint8_t *)&protocol, sizeof(protocol));
  if (!readResponse(recvbuf, sizeof(recvbuf)))
    return false;
  if (!responseChecksumValid(recvbuf, sizeof(recvbuf)))
    return false;
  return recvbuf[2] == 1;
}

DFRobot_GAS_I2C::DFRobot_GAS_I2C(TwoWire *pWire, uint8_t addr)
{
  _pWire = pWire;
  _I2C_addr = addr;
}

bool DFRobot_GAS_I2C::begin(void)
{
  _pWire->begin();
  _pWire->beginTransmission(_I2C_addr);
  return _pWire->endTransmission() == 0;
}

void DFRobot_GAS_I2C::setI2cAddr(uint8_t addr)
{
  _I2C_addr = addr;
}

bool DFRobot_GAS_I2C::dataIsAvailable(void)
{
  uint8_t buf[DFGAS_PAYLOAD_LEN] = {0};
  uint8_t recvbuf[DFGAS_PROTOCOL_LEN] = {0};
  buf[0] = CMD_GET_ALL_DTTA;
  sProtocol_t protocol = pack(buf, sizeof(buf));
  writeData(0, (uint8_t *)&protocol, sizeof(protocol));
  if (!readResponse(recvbuf, sizeof(recvbuf)))
    return false;

  return storeAndAnalyzeInitiativePacket(recvbuf, sizeof(recvbuf));
}

void DFRobot_GAS_I2C::writeData(uint8_t Reg, void *pData, uint8_t len)
{
  uint8_t *data = (uint8_t *)pData;
  _pWire->beginTransmission(_I2C_addr);
  _pWire->write(Reg);
  for (uint8_t i = 0; i < len; i++)
    _pWire->write(data[i]);
  _pWire->endTransmission();
}

int16_t DFRobot_GAS_I2C::readData(uint8_t Reg, uint8_t *Data, uint8_t len)
{
  uint8_t i = 0;
  _pWire->beginTransmission(_I2C_addr);
  _pWire->write(Reg);
  if (_pWire->endTransmission() != 0)
    return -1;

  _pWire->requestFrom((uint8_t)_I2C_addr, (uint8_t)len);
  while (_pWire->available() && i < len)
    Data[i++] = _pWire->read();

  return (i == len) ? len : -1;
}

#if (!defined ARDUINO_ESP32_DEV) && (!defined __SAMD21G18A__)

DFRobot_GAS_SoftWareUart::DFRobot_GAS_SoftWareUart(SoftwareSerial *psoftUart)
{
  _psoftUart = psoftUart;
}

bool DFRobot_GAS_SoftWareUart::begin(void)
{
  _psoftUart->begin(9600);
  return true;
}

bool DFRobot_GAS_SoftWareUart::dataIsAvailable(void)
{
  if (_psoftUart->available() < DFGAS_PROTOCOL_LEN)
    return false;

  uint8_t recvbuf[DFGAS_PROTOCOL_LEN];
  if (!readInitiativePacket(recvbuf, sizeof(recvbuf)))
    return false;

  return storeAndAnalyzeInitiativePacket(recvbuf, sizeof(recvbuf));
}

void DFRobot_GAS_SoftWareUart::writeData(uint8_t Reg, void *pData, uint8_t len)
{
  (void)Reg;
  uint8_t *data = (uint8_t *)pData;
  _psoftUart->write(data, len);
}

int16_t DFRobot_GAS_SoftWareUart::readData(uint8_t Reg, uint8_t *Data, uint8_t len)
{
  (void)Reg;
  uint32_t start = millis();
  while ((millis() - start) < 3000)
  {
    if (_psoftUart->available() >= len)
      break;
  }

  if (_psoftUart->available() < len)
    return -1;

  for (uint8_t i = 0; i < len; i++)
    Data[i] = _psoftUart->read();

  return len;
}

#else

DFRobot_GAS_HardWareUart::DFRobot_GAS_HardWareUart(HardwareSerial *phardUart)
{
  _pharduart = phardUart;
}

bool DFRobot_GAS_HardWareUart::begin(void)
{
  _pharduart->begin(9600);
  return true;
}

bool DFRobot_GAS_HardWareUart::dataIsAvailable(void)
{
  if (_pharduart->available() < DFGAS_PROTOCOL_LEN)
    return false;

  uint8_t recvbuf[DFGAS_PROTOCOL_LEN];
  if (!readInitiativePacket(recvbuf, sizeof(recvbuf)))
    return false;

  return storeAndAnalyzeInitiativePacket(recvbuf, sizeof(recvbuf));
}

void DFRobot_GAS_HardWareUart::writeData(uint8_t Reg, void *pData, uint8_t len)
{
  (void)Reg;
  uint8_t *data = (uint8_t *)pData;
  _pharduart->write(data, len);
}

int16_t DFRobot_GAS_HardWareUart::readData(uint8_t Reg, uint8_t *Data, uint8_t len)
{
  (void)Reg;
  uint32_t start = millis();
  while ((millis() - start) < 3000)
  {
    if (_pharduart->available() >= len)
      break;
  }

  if (_pharduart->available() < len)
    return -1;

  for (uint8_t i = 0; i < len; i++)
    Data[i] = _pharduart->read();

  return len;
}
#endif

#ifdef DFGAS_HOST_TEST
uint8_t DFGasTestAccess::computeChecksum(uint8_t *data, uint8_t ln)
{
  return FucCheckSum(data, ln);
}

sProtocol_t DFGasTestAccess::pack(DFRobot_GAS &gas, uint8_t *pBuf, uint8_t len)
{
  return gas.pack(pBuf, len);
}

bool DFGasTestAccess::responseChecksumValid(const DFRobot_GAS &gas, const uint8_t *recvbuf, uint8_t len)
{
  return gas.responseChecksumValid(recvbuf, len);
}

float DFGasTestAccess::applyTempCompensation(const DFRobot_GAS &gas, float concentration, uint8_t gasType, float temp)
{
  return gas.applyTempCompensation(concentration, gasType, temp);
}

float DFGasTestAccess::adcToTempC(const DFRobot_GAS &gas, uint16_t tempAdc)
{
  return gas.adcToTempC(tempAdc);
}

const char *DFGasTestAccess::gasTypeFromCode(uint8_t code)
{
  return DFRobot_GAS::gasTypeFromCode(code);
}

bool DFGasTestAccess::gasTypeUsesTenths(DFRobot_GAS::eType_t gasType)
{
  return DFRobot_GAS::gasTypeUsesTenths(gasType);
}

void DFGasTestAccess::scaleThresholdForGasType(uint16_t &threshold, DFRobot_GAS::eType_t gasType)
{
  DFRobot_GAS::scaleThresholdForGasType(threshold, gasType);
}

bool DFGasTestAccess::storeAndAnalyzeInitiativePacket(DFRobot_GAS &gas, const uint8_t *recvbuf, uint8_t len)
{
  return gas.storeAndAnalyzeInitiativePacket(recvbuf, len);
}

bool DFGasTestAccess::readResponse(DFRobot_GAS &gas, uint8_t *recvbuf, uint8_t len)
{
  return gas.readResponse(recvbuf, len);
}

float DFGasTestAccess::getGasConcentration(const DFRobot_GAS &gas)
{
  return gas.getGasConcentration();
}

const char *DFGasTestAccess::getGasType(const DFRobot_GAS &gas)
{
  return gas.getGasType();
}

float DFGasTestAccess::getSensorTemperature(const DFRobot_GAS &gas)
{
  return gas.getSensorTemperature();
}
#endif
