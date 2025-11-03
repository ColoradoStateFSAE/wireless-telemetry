#include "Arduino.h"
#include <cstdarg>

// Global mock time
unsigned long mock_millis_value = 0;

unsigned long millis() {
    return mock_millis_value;
}

void delay(unsigned long ms) {
    mock_millis_value += ms;
}

void pinMode(int pin, int mode) {
    // Mock implementation - do nothing
}

void digitalWrite(int pin, int value) {
    // Mock implementation - do nothing
}

int digitalRead(int pin) {
    // Mock implementation - return LOW
    return LOW;
}

int analogRead(int pin) {
    // Mock implementation - return mid-range value
    return 512;
}

void analogWrite(int pin, int value) {
    // Mock implementation - do nothing
}

// Mock Serial implementations
MockSerial Serial;
MockSerial Serial1;
MockSerial Serial2;

void MockSerial::begin(unsigned long baud) {
    // Mock implementation
}

void MockSerial::end() {
    // Mock implementation
}

int MockSerial::available() {
    return read_data.size() - read_pos;
}

int MockSerial::read() {
    if (read_pos < read_data.size()) {
        return read_data[read_pos++];
    }
    return -1;
}

size_t MockSerial::write(uint8_t byte) {
    written_data += static_cast<char>(byte);
    return 1;
}

size_t MockSerial::write(const uint8_t *buffer, size_t size) {
    written_data.append(reinterpret_cast<const char*>(buffer), size);
    return size;
}

size_t MockSerial::print(const char *str) {
    written_data += str;
    return strlen(str);
}

size_t MockSerial::print(int val) {
    char buf[32];
    snprintf(buf, sizeof(buf), "%d", val);
    return print(buf);
}

size_t MockSerial::print(unsigned long val) {
    char buf[32];
    snprintf(buf, sizeof(buf), "%lu", val);
    return print(buf);
}

size_t MockSerial::print(float val) {
    char buf[32];
    snprintf(buf, sizeof(buf), "%f", val);
    return print(buf);
}

size_t MockSerial::print(double val) {
    char buf[32];
    snprintf(buf, sizeof(buf), "%f", val);
    return print(buf);
}

size_t MockSerial::println(const char *str) {
    size_t n = print(str);
    written_data += '\n';
    return n + 1;
}

size_t MockSerial::println(int val) {
    size_t n = print(val);
    written_data += '\n';
    return n + 1;
}

size_t MockSerial::println() {
    written_data += '\n';
    return 1;
}

size_t MockSerial::printf(const char *format, ...) {
    char buf[256];
    va_list args;
    va_start(args, format);
    int n = vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    written_data += buf;
    return n;
}

// Mock SPI implementation
MockSPI SPI1;

void MockSPI::begin() {
    // Mock implementation
}

void MockSPI::end() {
    // Mock implementation
}

void MockSPI::setSCK(int pin) {
    // Mock implementation
}

void MockSPI::setMISO(int pin) {
    // Mock implementation
}

void MockSPI::setMOSI(int pin) {
    // Mock implementation
}

uint8_t MockSPI::transfer(uint8_t data) {
    // Mock implementation - return echo
    return data;
}
