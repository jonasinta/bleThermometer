/*
 * thermValues.h
 *
 *  Created on: 31/01/2022
 *      Author: jonas
 */

#ifndef THERMVALUES_H_
#define THERMVALUES_H_


#include <Adafruit_ADS1X15.h>
#include <Wire.h>
#include <esp_log.h>

#define Rseries 30000   //30k
#define ADCMax 32767
#define NOMINAL_RESISTANCE 30000 //thr4mistor at 25c
#define BCOEFFICIENT 3960 //magic thermistor number
#define NOMINAL_TEMPERATURE 25 //thermistor 30k at 25c
#define TRIM_TEMP_ADC0  0 // factor to add to temp C for adc1
#define TRIM_TEMP_ADC1  0 // factor to add to temp C for adc1

//sleep time
// got 7 days of battery with sleeptime 60 seconds
//#define sleepTime 1  //1 seconds sleeptime
#define sleepTime 2 //20secs sleeptime
//#define sleepTime 100 //10secs sleeptime
//#define sleepTime 300  //60 seconds x 5
//#define sleepTime 600  //60 seconds x 10
//#define sleepTime 900  //60 seconds x 15
#define looptimedelay 2    //5
#define SleeptimeUs (sleepTime*1000000)
#define WATCHDOGtimeout  20// WDT timeout 10 seconds

int Rtherm= 0;
//int ADCMax = 32767;
float ratio;
// external ADC
Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
//Adafruit_ADS1015 ads;     /* Use this for the 12-bit version */
//MAX Ox7fff




#endif /* THERMVALUES_H_ */
