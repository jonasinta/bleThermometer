#include "CircularBuffer.h"

// RTC slow memory variables
RTC_DATA_ATTR uint16_t circularBuffer[BUFFER_SIZE] = {0};
RTC_DATA_ATTR size_t head = 0;
RTC_DATA_ATTR size_t tail = 0;
RTC_DATA_ATTR size_t itemCount = 0;
RTC_DATA_ATTR bool isBufferInitialized = false;

// Initialize the circular buffer
void initCircularBuffer() {
    if (!isBufferInitialized) {
        for (size_t i = 0; i < BUFFER_SIZE; ++i) {
            circularBuffer[i] = 0;
        }
        head = 0;
        tail = 0;
        itemCount = 0;
        isBufferInitialized = true;
    }
}

// Push a value into the buffer
bool pushToBuffer(uint16_t value) {
    if (isBufferFull()) {
        return false; // Buffer is full
    }
    circularBuffer[head] = value;
    head = (head + 1) % BUFFER_SIZE;
    itemCount++;
    return true;
}

// Pop a value from the buffer
bool popFromBuffer(uint16_t* value) {
    if (isBufferEmpty()) {
        return false; // Buffer is empty
    }
    *value = circularBuffer[tail];
    tail = (tail + 1) % BUFFER_SIZE;
    itemCount--;
    return true;
}

// Iterate over the buffer and apply a callback function
void iterateBuffer(void (*callback)(uint16_t)) {
    size_t current = tail;
    for (size_t i = 0; i < itemCount; ++i) {
        callback(circularBuffer[current]);
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