#ifndef ARDUINO_MOCK_H
#define ARDUINO_MOCK_H

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

// Basic Arduino types
typedef bool boolean;
typedef uint8_t byte;

// Pin modes
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2

// Digital values
#define HIGH 1
#define LOW 0

// Mock time
extern unsigned long mock_millis_value;
unsigned long millis();
void delay(unsigned long ms);

// Mock analog/digital I/O
void pinMode(int pin, int mode);
void digitalWrite(int pin, int value);
int digitalRead(int pin);
int analogRead(int pin);
void analogWrite(int pin, int value);

// Mock Serial class
class MockSerial {
public:
    void begin(unsigned long baud);
    void end();
    int available();
    int read();
    size_t write(uint8_t byte);
    size_t write(const uint8_t *buffer, size_t size);
    size_t print(const char *str);
    size_t print(int val);
    size_t print(float val);
    size_t println(const char *str);
    size_t println(int val);
    size_t println();
    size_t printf(const char *format, ...);

    // Test helpers
    std::string getWrittenData() const { return written_data; }
    void clearWrittenData() { written_data.clear(); }
    void setReadData(const std::string& data) { read_data = data; read_pos = 0; }

private:
    std::string written_data;
    std::string read_data;
    size_t read_pos = 0;
};

extern MockSerial Serial;
extern MockSerial Serial1;
extern MockSerial Serial2;

// Mock SPI class
class MockSPI {
public:
    void begin();
    void end();
    void setSCK(int pin);
    void setMISO(int pin);
    void setMOSI(int pin);
    uint8_t transfer(uint8_t data);
    void beginTransaction(void* settings) {}
    void endTransaction() {}
};

extern MockSPI SPI1;

#endif // ARDUINO_MOCK_H
