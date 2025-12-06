#include "CircularBuffer.h"

// RTC slow memory variables
RTC_DATA_ATTR uint16_t EpochBuff[BUFFER_SIZE] = {0};
RTC_DATA_ATTR float Float1[BUFFER_SIZE] = {0};
RTC_DATA_ATTR float Float2[BUFFER_SIZE] = {0};
RTC_DATA_ATTR float Float3[BUFFER_SIZE] = {0};
RTC_DATA_ATTR size_t head = 0;
RTC_DATA_ATTR size_t tail = 0;
RTC_DATA_ATTR size_t itemCount = 0;
RTC_DATA_ATTR bool isBufferInitialized = false;

// Initialize the circular buffer
void initCircularBuffer() {
    if (!isBufferInitialized) {
        for (size_t i = 0; i < BUFFER_SIZE; ++i) {
            EpochBuff[i] = 0;
            Float1[i] = 0.0f;
            Float2[i] = 0.0f;
            Float3[i] = 0.0f;
        }
        head = 0;
        tail = 0;
        itemCount = 0;
        isBufferInitialized = true;
    }
}

// Push a value into the buffer
bool pushToBuffer(DATASEND * value) {
    if (isBufferFull()) {
        return false; // Buffer is full
    }
    EpochBuff[head] = value ->epoch;
    Float1[head] = value ->Float1;
    Float2[head] = value ->Float2;
    Float3[head] = value ->Float3;
    head = (head + 1) % BUFFER_SIZE;
    itemCount++;
    return true;
}

// Pop a value from the buffer
bool popFromBuffer(DATASEND * value) {
    if (isBufferEmpty()) {
        return false; // Buffer is empty
    }
    value ->epoch= EpochBuff[tail];
    value ->Float1 = Float1[tail];
    value ->Float2 = Float2[tail];
    value ->Float3 = Float3[tail];
    tail = (tail + 1) % BUFFER_SIZE;
    itemCount--;
    return true;
}

// Iterate over the buffer and apply a callback function
void iterateBuffer(void (*callback)(DATASEND  value)) {
    size_t current = tail;
    for (size_t i = 0; i < itemCount; ++i) {
        DATASEND value;
        value.epoch = EpochBuff[current];
        value.Float1 = Float1[current];
        value.Float2 = Float2[current];
        value.Float3 = Float3[current];
        // Call the callback function with the current value
        
        callback(value);
        current = (current + 1) % BUFFER_SIZE;
    }
}

// Check if the buffer is full
bool isBufferFull() {
    return itemCount == BUFFER_SIZE;
}

// Check if the buffer is empty
bool isBufferEmpty() {
    return itemCount == 0;
}