// If using eg. Visual Studio Code with Arduino plugin, ARDUINO may not be set:
#ifndef ARDUINO
#define ARDUINO 185
#endif

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

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

#define STATE_GROUND_IDLE           0   // ### Ground, idle state
#define STATE_GROUND_IDLE_ON_PAD    1   // ### Ground, on-pad idle state
#define STATE_GROUND_ARMED          2   // ### Ground, armed state
#define STATE_AIRBORNE_OUTBOUND     3   // ### Airborne, outbound state
#define STATE_AIRBORNE_DEPLOYMENT   4   // ### Airborne, deployment state
#define STATE_AIRBORNE_INBOUND      5   // ### Airborne, inbound state
#define STATE_GROUND_RECOVERY       6   // ### Ground, recovery state
#define STATE_GROUND_DATADUMP       7   // ### Ground, data-dump state


Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

byte sensorStatus = 0x0;
volatile byte stateMachineState = STATE_GROUND_IDLE;

// Set a default sea level (= 0m ASL) pressure
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

void displaySensorDetails(void)
{
    sensor_t sensor;
    bmp.getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.print  ("Sensor:       "); Serial.println(sensor.name);
    Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
    Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
    Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" hPa");
    Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" hPa");
    Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" hPa");  
    Serial.println("------------------------------------");
    Serial.println("");
}

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
        displaySensorDetails();
        SET_BIT(sensorStatus, STATUS_BAROMETER_DETECTED);
    }

    Serial.print("Setup status byte: ");
    Serial.println(sensorStatus);

    Serial.println("setup() ends");
}



void loop()
{
    sensors_event_t event;

    Serial.print("State: ");
    Serial.println(stateMachineState);

    switch(stateMachineState)
    {
        case STATE_GROUND_IDLE_ON_PAD:
        {
            break;
        }
        
        case STATE_GROUND_ARMED:
        {
            break;
        }
        
        case STATE_AIRBORNE_OUTBOUND:
        {
            break;
        }
        
        case STATE_AIRBORNE_DEPLOYMENT:
        {
            break;
        }
        
        case STATE_AIRBORNE_INBOUND:
        {
            break;
        }
        
        case STATE_GROUND_RECOVERY:
        {
            break;
        }
        
        case STATE_GROUND_DATADUMP:
        {
            break;
        }

        default:
        case STATE_GROUND_IDLE:
        {
            break;
        }
    }

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
    
    bmp.getEvent(&event);

    if(event.pressure)
    {
        /* Display atmospheric pressue in hPa */
        Serial.print("Pressure:    ");
        Serial.print(event.pressure);
        Serial.println(" hPa");
        
        /* First we get the current temperature from the BMP085 */
        float temperature;
        bmp.getTemperature(&temperature);
        Serial.print("Temperature: ");
        Serial.print(temperature);
        Serial.println(" C");

        /* Then convert the atmospheric pressure, and SLP to altitude         */
        /* Update this next line with the current SLP for better results      */
        Serial.print("Altitude:    "); 
        Serial.print(bmp.pressureToAltitude(seaLevelPressure,
                                            event.pressure)); 
        Serial.println(" m");
        Serial.println("");
    }
    else
    {
        Serial.println("Sensor event error");
    }
    
    delay(10000);
}
