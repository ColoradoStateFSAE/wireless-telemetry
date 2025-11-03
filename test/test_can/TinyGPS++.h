#ifndef TINYGPSPLUS_MOCK_H
#define TINYGPSPLUS_MOCK_H

#include <cstdint>

class TinyGPSLocation {
public:
    bool isValid() const { return valid; }
    double lat() const { return latitude; }
    double lng() const { return longitude; }

    // Test helpers
    void setValid(bool v) { valid = v; }
    void setLat(double lat) { latitude = lat; }
    void setLng(double lng) { longitude = lng; }

private:
    bool valid = false;
    double latitude = 0.0;
    double longitude = 0.0;
};

class TinyGPSAltitude {
public:
    bool isValid() const { return valid; }
    double meters() const { return altitude_m; }

    // Test helpers
    void setValid(bool v) { valid = v; }
    void setMeters(double m) { altitude_m = m; }

private:
    bool valid = false;
    double altitude_m = 0.0;
};

class TinyGPSSpeed {
public:
    bool isValid() const { return valid; }
    double kmph() const { return speed_kmph; }

    // Test helpers
    void setValid(bool v) { valid = v; }
    void setKmph(double s) { speed_kmph = s; }

private:
    bool valid = false;
    double speed_kmph = 0.0;
};

class TinyGPSCourse {
public:
    bool isValid() const { return valid; }
    double deg() const { return course_deg; }

    // Test helpers
    void setValid(bool v) { valid = v; }
    void setDeg(double c) { course_deg = c; }

private:
    bool valid = false;
    double course_deg = 0.0;
};

class TinyGPSPlus {
public:
    TinyGPSLocation location;
    TinyGPSAltitude altitude;
    TinyGPSSpeed speed;
    TinyGPSCourse course;

    bool encode(char c) {
        // Mock implementation - just return true
        return true;
    }

    // Test helper to set all GPS data at once
    void setGPSData(double lat, double lng, double alt, double spd, double crs) {
        location.setValid(true);
        location.setLat(lat);
        location.setLng(lng);
        altitude.setValid(true);
        altitude.setMeters(alt);
        speed.setValid(true);
        speed.setKmph(spd);
        course.setValid(true);
        course.setDeg(crs);
    }

    void invalidateGPS() {
        location.setValid(false);
        altitude.setValid(false);
        speed.setValid(false);
        course.setValid(false);
    }
};

#endif // TINYGPSPLUS_MOCK_H
