#include "Arduino.h"
#include <esp_task_wdt.h>
#include <esp_system.h>
#include <rom/rtc.h>
#include <thermValues.h>



//circularBuffer include
#include <CircularBuffer.hpp>

#include <ArduinoJson.h>
//for BT address
#include "esp_bt_main.h"
#include "esp_bt_device.h"

//BLE setup includes and globals
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_poolFloater_UUID        "d586aca9-12be-4e03-8ad7-b0106ba6018f"
#define CHARACTERISTIC_tempC_UUID "42b5ebfe-a946-4f85-ba49-03495b0aa5ee"
#define CHARACTERISTIC_setEpoch "setEpochNotUUID"

BLECharacteristic *pCharacteristicValuesStructure; //global for the characterisy=tic, that way i can access it in loop

//time stuff library import and globals....
#include <ESP32Time.h>
ESP32Time rtc;
RTC_NOINIT_ATTR  unsigned  long epoch ;

bool isConnected = false;

int LED = 5; // LED connected to pin 2
float oldWeight = 0.000; // will use this variable to print over text in screen in backgound colour to blank text before next write

float newWeight = 0.000; //
long count = 0; //will use this for testing mqtt persistence
struct  DataSend {
	unsigned long epoch;
	float tempC;
	float tempCa;
	float battV;
}  boing;

/*struct RTC_NOINIT_ATTR DataSend {
	unsigned long epoch;
	float tempC;
	float tempCa;
	float battV;
}  boing;
*/

int reset_reason=0; //reste reason integer for reporting back to DB

// CircularBuffer<DataSend, 220> RTC_NOINIT_ATTR bufferCircle;
CircularBuffer<DataSend, 110> RTC_NOINIT_ATTR bufferCircle ;

//for BT address
const uint8_t* point = esp_bt_dev_get_address();

class ServersCallbacks: public BLEServerCallbacks {
	void onConnect(BLEServer *pServer) {
		isConnected = true;
		digitalWrite(LED, HIGH);
		log_i("*********");
		log_i("Co nected");

	}
	void onDisconnect(BLEServer *pServer) {
		isConnected = false;
		digitalWrite(LED, LOW);
		log_i("*********");
		log_i("Diss co nected: ");
	}

};

class MyCallbacks: public BLECharacteristicCallbacks {
	void onWrite(BLECharacteristic *pCharacteristic) {
		std::string value = pCharacteristic->getValue();
		BLEUUID gotUUID = pCharacteristic->getUUID();
		if (value.length() > 0) {
			log_i("*********");
			log_i("New value: ");
			for (int i = 0; i < value.length(); i++){
				log_d("uid bit %i;",value[i]);
				log_d("bit index %i;",i);
			}


			log_d("*********");
			String doing = value.c_str();
			int length = doing.length();
			log_i("epoch before trim %s",doing);
			doing.remove((length - 3), 3); // trim mS from the epoch
			log_i("epoch after trim %s",doing);
			epoch = doing.toInt();

			log_i("epochPrint %Ld",(long) epoch, 10);
			if (epoch < 2524611600L && epoch > 946688400L) {

				rtc.setTime(epoch, 0);
				log_i("epochPrint in epoch >150.... %Ld commer 10 means nothing %d",(long) epoch, 10);

			} //close if check
		}
		if (gotUUID.bitSize() > 0) {
			std::string valueUUID = gotUUID.toString();
			for (int i = 0; i < valueUUID.length(); i++){
				log_d("UUID; %d",valueUUID[i]);}

		}
log_i("hi there");
// nodered cant send giveTime when device is resetting all the tome, so under these circumstances, this function is useless
		if (value == "giveTime") {
			log_d("the time now is");
			log_d("time %s",rtc.getDateTime());
		}
	}
	void onRead(BLECharacteristic *pCharacteristic) {
		log_i(	"++++++++Read from client has just happened+++++++++++++++++++");
printBuffer();
	}
};

void printBuffer(void) {
log_i("in print buffer function");

	while (! bufferCircle.isEmpty()) {
		char tempString[200];
		struct DataSend temp = bufferCircle.pop();

		//jsonify

		JsonDocument doc;

		doc["count"] = temp.epoch;
		doc["adcvalue"] = temp.tempC;
		doc["adcvalue1"] = temp.tempCa;
		doc["battV"] = temp.battV;
		doc["reset_reason"] = reset_reason;

		serializeJson(doc, tempString);
		//serializeJson(doc, Serial);
		log_i("string tpo send to node3red %s",tempString);

		pCharacteristicValuesStructure->setValue(tempString);
		pCharacteristicValuesStructure->notify();
		log_i("just after characheristic set value and notify %s",tempString);
		delay(50);

	} //closew while

	//delay(2000);
} //close void printBuffer(void)

void messageReceived(String &topic, String &payload) {
  //blank unused function
}


int16_t readAdc(int counts, int channel) {
	int16_t avgAdc = 0.0;
	long adcAccumulater = 0;
	for (int var = 0; var < counts; ++var) {
		adcAccumulater += ads.readADC_SingleEnded(channel);
	}
	//log_w("adc total %d", adcAccumulater);
	avgAdc = adcAccumulater / counts;
//log_w("adc after averagation %d", avgAdc);
	return avgAdc;
}

float CalcSteinhart(float voltsVo0, float voltsVcc) {
	//compute resistance of thermistor (Rtherm)
	//Rtherm= (Vo x Rseries)/(Vcc-Vo)
	Rtherm = voltsVo0 * Rseries;
	Rtherm = Rtherm / (voltsVcc - voltsVo0);
	float steinhart;
	steinhart = Rtherm / (float) (NOMINAL_RESISTANCE); // (R/Ro)
	steinhart = log(steinhart); // ln(R/Ro)
	steinhart /= (float) (BCOEFFICIENT); // 1/B * ln(R/Ro)
	steinhart += 1.0 / (NOMINAL_TEMPERATURE + 273.15); // + (1/To)
	steinhart = 1.0 / steinhart; // Invert
	steinhart -= 273.15; // convert to C
	return steinhart;
}

float getTemperature(int count) {
	char counts = 10; //number of counts the adc should do
	int16_t adcVo0, adcVo1 ,adcVcc, adcBatt; // adc value for Vo, Vcc and Batt connected to inputs 0,1 and 3
	float voltsVo0, voltsVo1, voltsVcc, voltsBat;  // convert adc readings to voltages

	adcVo0 = readAdc(counts, 1); //read Vo circuit = ads.readADC_SingleEnded(0);//read Vo circuit
	adcVcc = readAdc(counts, 0);  //read Vo circuit
	adcVo1 = readAdc(counts, 2);  //read Vo1 circuit
	adcBatt = readAdc(counts, 3);  //read Vo circuit
// get voltages from adc Vo and Vcc both connected to adc0 and adc1- battery is adc3
	voltsVo0 = ads.computeVolts(adcVo0);  // get Vo
	voltsVo1 = ads.computeVolts(adcVo1);  // get Vo1
	voltsVcc = ads.computeVolts(adcVcc);  // get Vcc
	voltsBat = ads.computeVolts(adcBatt);  // getBattV
//double the batt voltage as it is halved in circuit
	voltsBat *= 2;
//compute resistance of thermistor (Rtherm)
//Rtherm= (Vo x Rseries)/(Vcc-Vo)
	float steinhartVo0  = CalcSteinhart(voltsVo0, voltsVcc);//convert the voltage measured to deg. C
	float steinhartVo1  = CalcSteinhart(voltsVo1, voltsVcc);//convert the voltage measured to deg. C
	steinhartVo0 += TRIM_TEMP_ADC0;
	steinhartVo1 += TRIM_TEMP_ADC1;
	log_d("voltsVo %fV, voltsVo1 %fV ,voltsVcc %fV.", voltsVo0, voltsVo1,  voltsVcc);
	log_i("Temperature top %f C, Temperature bot %f C, rtherm %d Ohm, batt volt %fV", steinhartVo0, steinhartVo1, Rtherm,
			voltsBat);
//put the battery voltage into the transmission structure
	boing.battV = voltsBat;
	boing.epoch = rtc.getEpoch();
	boing.tempC = steinhartVo0;
	boing.tempCa = steinhartVo1;

return steinhartVo0;
}
/// print out thew device BT address
void printDeviceAddress() {

  const uint8_t* point = esp_bt_dev_get_address();

  for (int i = 0; i < 6; i++) {

    char str[3];

    sprintf(str, "%02X", (int)point[i]);
    Serial.print(str);

    if (i < 5){
      Serial.print(":");
    }

  }
 // reset_reason = esp_reset_reason();
}

void setup() {
	//sleep setup//////watchdog Timer///////////////////////////////////////////////////////////
	uint64_t sleepies = (unsigned int)sleepTime * 1000000L;
	esp_sleep_enable_timer_wakeup(sleepies);


	pinMode(LED, OUTPUT);
	digitalWrite(LED, LOW); //connected indicator
	Serial.begin(115200);

	esp_task_wdt_init(WATCHDOGtimeout, true); //enable panic so ESP32 restarts
			esp_task_wdt_add(NULL); //add current thread to WDT watch


	//setup BLE setup BLE setup BLE setup BLE setup BLE setup BLE setup BLE setup BLE setup BLE setup BLE
	log_d("Starting BLE work!");
	BLEDevice::init("PoolThermometer");
	BLEServer *pServer = BLEDevice::createServer();
	pServer->setCallbacks(new ServersCallbacks);
	BLEService *pService = pServer->createService(SERVICE_poolFloater_UUID);
	pCharacteristicValuesStructure = pService->createCharacteristic(CHARACTERISTIC_tempC_UUID,
			BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
					| BLECharacteristic::PROPERTY_NOTIFY);
	//pCharacteristicValuesStructure->setNotifyProperty(true);
	//pCharacteristicValuesStructure->setValue("um");
	pCharacteristicValuesStructure->setCallbacks(new MyCallbacks());

	pService->start();
	BLEAdvertising *pAdvertising = pServer->getAdvertising(); // this still is working for backward compatibility
	//BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
	pAdvertising->addServiceUUID(SERVICE_poolFloater_UUID);
	pAdvertising->setScanResponse(true);
	pAdvertising->setMinPreferred(0x06); // functions that help with iPhone connections issue
	pAdvertising->setMinPreferred(0x12);
	//pAdvertising->setMaxInterval(1000);
	//pAdvertising->setMinInterval(2000);

	//set some addvertisement nice data-------------------
	//std::string manufacturerString = "IJ1968";
	//BLEAdvertisementData advertisementData;
	//advertisementData.setManufacturerData(manufacturerString);
	//advertisementData.setManufacturerData("77");

	//pAdvertising->setAdvertisementData(advertisementData);
	//set some addvertisement nice data-------------------------------------------------------
	////esp_task_wdt_reset();
	pAdvertising->start();
	log_d("BLE Advertisement start");
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//setup LCD setup LCD  setup LCD  setup LCD  setup LCD  setup LCD  setup LCD  setup LCD  setup LCD

	log_d("about to  setgain on ads and then enter loop");
	///////////////////////setup for the external adc and temperature///////////////////////////////////////////////////////////////////////////////////////////////////
	////esp_task_wdt_reset();
	//	ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
	ads.setGain(GAIN_ONE);    // 1x gain   +/- 4.096V  1 bit = 2mV      0.1250mV
	if (!ads.begin()) {
		log_d("Failed to initialize ADS.");

	}
	////esp_task_wdt_reset();
	printDeviceAddress();
	////////////////////////////////////////////////////////////////////////////////////////////////////////


}

void loop() {
	esp_task_wdt_reset();

	log_d("memory report Free Heap %d, Free Sketch Space %d", ESP.getFreeHeap(),ESP.getFreeSketchSpace());
//log_i("memory report Free Heap %d, Free Sketch Space %d", ESP.getFreeHeap(),ESP.getFreeSketchSpace());
	delay(looptimedelay * 1000);
	getTemperature(10);  //this function loads data into the structure from adc

	////esp_task_wdt_reset();
	//do not store data if epoch has not been yet set- i.e less than 1.5 billion equivalent of july 14 2017
	log_i("Just before if epoch > 2017 (1500000000) %Lu ", rtc.getEpoch());
			
	if (rtc.getEpoch() > 1500000000) {
		//if (1) {
		//if (epoch > 1) {
			
		bufferCircle.unshift(boing); //unshift will add data to the ringbuffer (boing)
		log_i("Just after if epoch > 2017 (1500000000) %Lu ", rtc.getEpoch());
	}

	//print time to serial port=======================================
	log_d("loop delay here is:  %d seconds\n", looptimedelay);

	log_d("date time printed in loop() %S \n",rtc.getDateTime());
	//log_d("date time printed in loop() %d \n",rtc.getDateTime());
	//==============================================================


	// test connected
	
	if (isConnected) {
		log_i("inside isConnected-printBuffer llop /n my storage buffer has %d units stored of %d", bufferCircle.size(), bufferCircle.capacity);
		printBuffer(); //routine to disharge buffer to BLE and client
	}
	//esp_bluedroid_disable;
	//esp_bluedroid_deinit();
	esp_deep_sleep_start();  //make it sleep deep
	//esp_light_sleep_start();  //make it sleep light
}
