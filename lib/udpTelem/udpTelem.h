#ifndef UDPTELEM_H
#define UDPTELEM_H

#include <Arduino.h>
#include <WiFi.h>


//#define TELEPLOT_URL "192.168.1.22"
#define TELEPLOT_URL "192.168.1.52"
#define TELEPLOT_PORT 47269


void sendTelemetry(const char *, int);
void sendTelemetry(const char *, float);



#endif // UDPTELEM_H