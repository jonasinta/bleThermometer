#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

void connectToServer();
int sendData(const char* data);


#include <Arduino.h>


// Define the buffer size
constexpr size_t BUFFER_SIZE = 545; // Adjusted to fit within RTC slow memory constraints
//constexpr size_t BUFFER_SIZE = 582;
// 582 elements of buffer has been worked out by estimating then reducing by one at a time untill 
// compiler did not report slow memory overflow, which resuletd in 582 elements
//it was estimated that the 4 controlling variables used 13bytes and the buffer elements sets used 
// 14 bytes for each buffer size set 

// Declare buffer and related variables in RTC slow memory
extern RTC_DATA_ATTR uint16_t EpochBuff[BUFFER_SIZE];
extern RTC_DATA_ATTR float Float1[BUFFER_SIZE];
extern RTC_DATA_ATTR float Float2[BUFFER_SIZE];
extern RTC_DATA_ATTR float Float3[BUFFER_SIZE];
extern RTC_DATA_ATTR size_t head;
extern RTC_DATA_ATTR size_t tail;
extern RTC_DATA_ATTR size_t itemCount;
extern RTC_DATA_ATTR bool isBufferInitialized;

typedef struct DataSendStruct
{
	unsigned long epoch;
	float Float1;
	float Float2;
	float Float3;
} DATASEND;

// Function declarations
void initCircularBuffer();
bool pushToBuffer(DATASEND * value);
bool popFromBuffer(DATASEND * value);
void iterateBuffer(void (*callback)(DATASEND value));
bool isBufferFull();
bool isBufferEmpty();



#endif // CIRCULAR_BUFFER_H

 