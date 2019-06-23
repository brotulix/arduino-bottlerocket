// If using eg. Visual Studio Code with Arduino plugin, ARDUINO may not be set:
#ifndef ARDUINO
#define ARDUINO 185
#endif

#include <Wire.h>
#include <Adafruit_BMP085.h>

#define SET_BIT(a, b)   a |= (0x1 << b);
#define CLEAR_BIT(a, b) a &= ~(0x1 << b);


#define INTSRC_INTERRUPT        2
#define INTSRC_GYROMETER        -1
#define INTSRC_BAROMETER        4
#define INTSRC_THERMOMETER      5
#define INTSRC_ACCELEROMETER    6
#define INTSRC_MAGNETOMETER     7
#define I2C_SDA                 18
#define I2C_SCL                 19

#define STATUS_BAROMETER_DETECTED       0
#define STATUS_ACCELEROMETER_DETECTED   1
#define STATUS_GYROMETER_DETECTED       2
#define STATUS_MAGNETOMETER_DETECTED    3

Adafruit_BMP085 bmp;

byte sensorStatus = 0x0;

void setup()
{
    Serial.begin(115200);
    Serial.println("setup() begins");

    Serial.println("Setting up pins...");

    pinMode(INTSRC_INTERRUPT, INPUT);
    //pinMode(INTSRC_GYROMETER, INPUT);
    pinMode(INTSRC_BAROMETER, INPUT);
    pinMode(INTSRC_THERMOMETER, INPUT);
    pinMode(INTSRC_ACCELEROMETER, INPUT);
    pinMode(INTSRC_MAGNETOMETER, INPUT);

    if (!bmp.begin()) {
        Serial.println("ERROR: BMP085 not detected!");
        CLEAR_BIT(sensorStatus, STATUS_BAROMETER_DETECTED);
    }
    else
    {
        SET_BIT(sensorStatus, STATUS_BAROMETER_DETECTED);
    }

    Serial.print("Setup status byte: ");
    Serial.println(sensorStatus);

    Serial.println("setup() ends");
}



void loop()
{
    Serial.println("Iter");
    
    if(digitalRead(digitalPinToInterrupt(INTSRC_BAROMETER))) 
    {
        // Barometer ready
        Serial.println("Barometer data ready");
    }

    if(digitalRead(digitalPinToInterrupt(INTSRC_THERMOMETER))) 
    {
        // Thermometer ready
        Serial.println("Thermometer data ready");
    }

    if(digitalRead(digitalPinToInterrupt(INTSRC_ACCELEROMETER))) 
    {
        // Accelerometer ready
        Serial.println("Accelerometer data ready");
    }

    if(digitalRead(digitalPinToInterrupt(INTSRC_MAGNETOMETER))) 
    {
        // Magnetometer ready
        Serial.println("Magnetometer data ready");
    }

    Serial.print("Temperature = ");
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");
    
    Serial.print("Pressure = ");
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");
    
    // Calculate altitude assuming 'standard' barometric
    // pressure of 1013.25 millibar = 101325 Pascal
    Serial.print("Altitude = ");
    Serial.print(bmp.readAltitude());
    Serial.println(" meters");

    Serial.print("Pressure at sealevel (calculated) = ");
    Serial.print(bmp.readSealevelPressure());
    Serial.println(" Pa");




    delay(10000);
}
