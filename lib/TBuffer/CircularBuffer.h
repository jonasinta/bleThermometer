#pragma once
#include <Arduino.h>


// Define the buffer size
constexpr size_t BUFFER_SIZE = 10;

// Declare buffer and related variables in RTC slow memory
extern RTC_DATA_ATTR uint16_t circularBuffer[BUFFER_SIZE];
extern RTC_DATA_ATTR size_t head;
extern RTC_DATA_ATTR size_t tail;
extern RTC_DATA_ATTR size_t itemCount;
extern RTC_DATA_ATTR bool isBufferInitialized;

// Function declarations
void initCircularBuffer();
bool pushToBuffer(uint16_t value);
bool popFromBuffer(uint16_t* value);
void iterateBuffer(void (*callback)(uint16_t));
bool isBufferFull();
bool isBufferEmpty();