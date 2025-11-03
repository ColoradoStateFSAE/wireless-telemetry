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

// Mock Stream base class (needed for Arduino compatibility)
class Stream {
public:
    virtual ~Stream() = default;
    virtual int available() = 0;
    virtual int read() = 0;
    virtual size_t write(uint8_t byte) = 0;
    virtual size_t write(const uint8_t *buffer, size_t size) = 0;
    virtual size_t print(const char *str) = 0;
    virtual size_t print(int val) = 0;
    virtual size_t print(unsigned long val) = 0;
    virtual size_t print(float val) = 0;
    virtual size_t print(double val) = 0;
    virtual size_t println(const char *str) = 0;
    virtual size_t println(int val) = 0;
    virtual size_t println() = 0;
};

// Mock Serial class
class MockSerial : public Stream {
public:
    void begin(unsigned long baud);
    void end();
    int available() override;
    int read() override;
    size_t write(uint8_t byte) override;
    size_t write(const uint8_t *buffer, size_t size) override;
    size_t print(const char *str) override;
    size_t print(int val) override;
    size_t print(unsigned long val) override;
    size_t print(float val) override;
    size_t print(double val) override;
    size_t println(const char *str) override;
    size_t println(int val) override;
    size_t println() override;
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
