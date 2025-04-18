#include "udpTelem.h"
WiFiUDP udpTelem;

void sendTelemetry(const char *signal, float value)
{
  static char buffer[256];
  uint32_t now_ms = millis();

  snprintf(buffer, sizeof(buffer), "%s:%3.3f",
      signal, value);
  udpTelem.beginPacket(TELEPLOT_URL, TELEPLOT_PORT);
  udpTelem.write((const uint8_t*)buffer, strlen(buffer));
  udpTelem.endPacket();
}

void sendTelemetry(const char *signal, int value)
{
  static char buffer[256];
  uint32_t now_ms = millis();

  snprintf(buffer, sizeof(buffer), "%s:%d",
      signal, value);
  udpTelem.beginPacket(TELEPLOT_URL, TELEPLOT_PORT);
  udpTelem.write((const uint8_t*)buffer, strlen(buffer));
  udpTelem.endPacket();
}