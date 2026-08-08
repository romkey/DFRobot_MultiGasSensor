#ifndef DFGAS_HOST_TEST
#define DFGAS_HOST_TEST
#endif

#include "DFRobot_MultiGasSensor.h"
#include <string.h>

class MockGAS : public DFRobot_GAS
{
public:
  MockGAS()
  {
    setResponseDelayMs(0);
    memset(_response, 0, sizeof(_response));
    _responseLen = 9;
  }

  void setResponse(const uint8_t *data, uint8_t len)
  {
    memset(_response, 0, sizeof(_response));
    if (data != NULL && len > 0)
      memcpy(_response, data, len < sizeof(_response) ? len : sizeof(_response));
    _responseLen = len;
  }

  void setResponseLength(int16_t len)
  {
    _responseLen = len;
  }

  bool begin() override { return true; }
  bool dataIsAvailable() override { return false; }

protected:
  void writeData(uint8_t, void *, uint8_t) override {}

  int16_t readData(uint8_t, uint8_t *data, uint8_t len) override
  {
    if (_responseLen < 0)
      return _responseLen;

    if ((int16_t)len > _responseLen)
    {
      memcpy(data, _response, _responseLen);
      return _responseLen;
    }

    memcpy(data, _response, len);
    return len;
  }

private:
  uint8_t _response[9];
  int16_t _responseLen;
};
