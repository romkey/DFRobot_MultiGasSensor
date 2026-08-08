#include "mock_gas.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int g_failures = 0;

static void expectTrue(bool condition, const char *message)
{
  if (!condition)
  {
    fprintf(stderr, "FAIL: %s\n", message);
    g_failures++;
  }
}

static void expectFalse(bool condition, const char *message)
{
  expectTrue(!condition, message);
}

static void expectEqualInt(int expected, int actual, const char *message)
{
  if (expected != actual)
  {
    fprintf(stderr, "FAIL: %s (expected %d, got %d)\n", message, expected, actual);
    g_failures++;
  }
}

static void expectEqualStr(const char *expected, const char *actual, const char *message)
{
  if (strcmp(expected, actual) != 0)
  {
    fprintf(stderr, "FAIL: %s (expected '%s', got '%s')\n", message, expected, actual);
    g_failures++;
  }
}

static void expectNear(float expected, float actual, float epsilon, const char *message)
{
  if (fabsf(expected - actual) > epsilon)
  {
    fprintf(stderr, "FAIL: %s (expected %.6f, got %.6f)\n", message, expected, actual);
    g_failures++;
  }
}

static void finalizePacket(uint8_t *packet)
{
  packet[8] = DFGasTestAccess::computeChecksum(packet, 8);
}

static void buildGasConcentrationResponse(uint8_t *packet, uint16_t rawValue, uint8_t gasType, uint8_t decimals)
{
  memset(packet, 0, 9);
  packet[0] = 0xFF;
  packet[1] = 0x01;
  packet[2] = rawValue >> 8;
  packet[3] = rawValue & 0xFF;
  packet[4] = gasType;
  packet[5] = decimals;
  finalizePacket(packet);
}

static void buildInitiativePacket(uint8_t *packet, uint16_t rawValue, uint8_t gasType, uint8_t decimals, uint16_t tempAdc)
{
  memset(packet, 0, 9);
  packet[0] = 0xFF;
  packet[1] = CMD_GET_ALL_DTTA;
  packet[2] = rawValue >> 8;
  packet[3] = rawValue & 0xFF;
  packet[4] = gasType;
  packet[5] = decimals;
  packet[6] = tempAdc >> 8;
  packet[7] = tempAdc & 0xFF;
  finalizePacket(packet);
}

static void testChecksumRoundTrip()
{
  uint8_t packet[9] = {0xFF, 0x01, 0x86, 0x00, 0x04, 0x00, 0x01, 0x90, 0x00};
  finalizePacket(packet);
  MockGAS gas;
  expectTrue(DFGasTestAccess::responseChecksumValid(gas, packet, 9), "valid checksum accepted");
  packet[8] ^= 0xFF;
  expectFalse(DFGasTestAccess::responseChecksumValid(gas, packet, 9), "corrupted checksum rejected");
}

static void testPackGetGasConcentrationCommand()
{
  MockGAS gas;
  uint8_t payload[6] = {CMD_GET_GAS_CONCENTRATION, 0, 0, 0, 0, 0};
  sProtocol_t protocol = DFGasTestAccess::pack(gas, payload, sizeof(payload));

  expectEqualInt(0xFF, protocol.head, "pack sets header");
  expectEqualInt(0x01, protocol.addr, "pack sets address");
  expectEqualInt(CMD_GET_GAS_CONCENTRATION, protocol.data[0], "pack copies command byte");
  expectTrue(protocol.check != 0, "pack computes checksum");
}

static void testGasTypeFromCode()
{
  expectEqualStr("CO", DFGasTestAccess::gasTypeFromCode(DFRobot_GAS::CO), "CO gas type mapping");
  expectEqualStr("O2", DFGasTestAccess::gasTypeFromCode(DFRobot_GAS::O2), "O2 gas type mapping");
  expectEqualStr("", DFGasTestAccess::gasTypeFromCode(0x99), "unknown gas type mapping");
}

static void testThresholdScaling()
{
  expectTrue(DFGasTestAccess::gasTypeUsesTenths(DFRobot_GAS::O2), "O2 uses tenths");
  expectFalse(DFGasTestAccess::gasTypeUsesTenths(DFRobot_GAS::CO), "CO does not use tenths");

  uint16_t threshold = 200;
  DFGasTestAccess::scaleThresholdForGasType(threshold, DFRobot_GAS::O2);
  expectEqualInt(2000, threshold, "O2 threshold scaled by 10");

  threshold = 200;
  DFGasTestAccess::scaleThresholdForGasType(threshold, DFRobot_GAS::CO);
  expectEqualInt(200, threshold, "CO threshold unchanged");
}

static void testAdcToTempC()
{
  MockGAS gas;
  expectNear(0.0f, DFGasTestAccess::adcToTempC(gas, 1023), 0.001f, "saturated ADC returns 0");
  expectNear(0.0f, DFGasTestAccess::adcToTempC(gas, 1024), 0.001f, "overflow ADC returns 0");

  float roomTemp = DFGasTestAccess::adcToTempC(gas, 512);
  expectTrue(roomTemp > 10.0f && roomTemp < 40.0f, "mid-range ADC gives plausible temperature");
}

static void testTempCompensation()
{
  MockGAS gas;
  float compensated = DFGasTestAccess::applyTempCompensation(gas, 100.0f, DFRobot_GAS::CO, 10.0f);
  expectNear(100.0f / (0.005f * 10.0f + 0.9f), compensated, 0.001f, "CO compensation at 10C");

  compensated = DFGasTestAccess::applyTempCompensation(gas, 100.0f, DFRobot_GAS::CO, 25.0f);
  expectNear(100.0f / (0.005f * 25.0f + 0.9f) - (0.3f * 25.0f - 6.0f), compensated, 0.001f, "CO compensation at 25C");

  compensated = DFGasTestAccess::applyTempCompensation(gas, 100.0f, DFRobot_GAS::CO, 41.0f);
  expectNear(0.0f, compensated, 0.001f, "CO compensation out of range returns 0");

  compensated = DFGasTestAccess::applyTempCompensation(gas, 100.0f, DFRobot_GAS::NO2, 0.0f);
  expectNear(100.0f / 0.9f - 0.005f, compensated, 0.001f, "NO2 compensation at 0C boundary");
}

static void testMockReadGasConcentration()
{
  MockGAS gas;
  uint8_t response[9];
  buildGasConcentrationResponse(response, 1234, DFRobot_GAS::CO, 1);
  gas.setResponse(response, sizeof(response));

  expectNear(123.4f, gas.readGasConcentrationPPM(), 0.001f, "readGasConcentrationPPM parses decimal place");

  buildGasConcentrationResponse(response, 500, DFRobot_GAS::CO, 0);
  gas.setResponse(response, sizeof(response));
  expectNear(500.0f, gas.readGasConcentrationPPM(), 0.001f, "readGasConcentrationPPM parses integer ppm");
}

static void testMockReadGasConcentrationRejectsBadChecksum()
{
  MockGAS gas;
  uint8_t response[9];
  buildGasConcentrationResponse(response, 500, DFRobot_GAS::CO, 0);
  response[8] ^= 0xFF;
  gas.setResponse(response, sizeof(response));

  expectNear(0.0f, gas.readGasConcentrationPPM(), 0.001f, "bad checksum returns 0 ppm");
}

static void testMockChangeAcquireMode()
{
  MockGAS gas;
  uint8_t response[9];
  buildGasConcentrationResponse(response, 256, 0, 0);
  gas.setResponse(response, sizeof(response));
  expectTrue(gas.changeAcquireMode(DFRobot_GAS::PASSIVITY), "changeAcquireMode accepts valid ack");

  response[8] ^= 0xFF;
  gas.setResponse(response, sizeof(response));
  expectFalse(gas.changeAcquireMode(DFRobot_GAS::PASSIVITY), "changeAcquireMode rejects bad checksum");
}

static void testInitiativePacketParsing()
{
  MockGAS gas;
  uint8_t response[9];
  buildInitiativePacket(response, 250, DFRobot_GAS::CO, 0, 512);
  expectTrue(DFGasTestAccess::storeAndAnalyzeInitiativePacket(gas, response, sizeof(response)), "initiative packet ingested");
  expectEqualStr("CO", DFGasTestAccess::getGasType(gas), "initiative packet gas type parsed");
  expectNear(250.0f, DFGasTestAccess::getGasConcentration(gas), 0.001f, "initiative packet concentration parsed");
  expectTrue(DFGasTestAccess::getSensorTemperature(gas) > 10.0f, "initiative packet temperature parsed");
}

static void testReadResponseRequiresFullPacket()
{
  MockGAS gas;
  uint8_t response[9];
  buildGasConcentrationResponse(response, 100, DFRobot_GAS::CO, 0);
  gas.setResponse(response, 8);
  gas.setResponseLength(8);

  uint8_t buffer[9] = {0};
  expectFalse(DFGasTestAccess::readResponse(gas, buffer, sizeof(buffer)), "short read fails readResponse");
}

int main()
{
  testChecksumRoundTrip();
  testPackGetGasConcentrationCommand();
  testGasTypeFromCode();
  testThresholdScaling();
  testAdcToTempC();
  testTempCompensation();
  testMockReadGasConcentration();
  testMockReadGasConcentrationRejectsBadChecksum();
  testMockChangeAcquireMode();
  testInitiativePacketParsing();
  testReadResponseRequiresFullPacket();

  if (g_failures == 0)
  {
    printf("All native tests passed.\n");
    return 0;
  }

  fprintf(stderr, "%d native test(s) failed.\n", g_failures);
  return 1;
}
