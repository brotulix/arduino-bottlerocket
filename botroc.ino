// If using eg. Visual Studio Code with Arduino plugin, ARDUINO may not be set:
#ifndef ARDUINO
#define ARDUINO 185
#endif

#include <Adafruit_Sensor.h>        // Adafruit Unified Sensor library
#include <Adafruit_BMP085_U.h>      // BMP085 Barometer
#include <Adafruit_ADXL345_U.h>     // ADXL345 Accelerometer
#include <Adafruit_HMC5883_U.h>     // HMC5883 Magnetometer
                                    // Gyro?

#define SET_BIT(a, b)   a |= (0x1 << b)
#define CLEAR_BIT(a, b) a &= ~(0x1 << b)
#define GET_BIT(a, b)   a & (0x1 << b)


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

#define COMMAND_LINE_BUFFER_SIZE                32
#define COMMAND_LINE_APPEND_THRESHOLD           ((COMMAND_LINE_BUFFER_SIZE / 4) * 3)

#define SENSORS_BAROMETER_NUM_VALUES            16
#define SENSORS_BAROMETER_SEALEVELHPA           SENSORS_PRESSURE_SEALEVELHPA
#define SENSORS_BAROMETER_FACTOR_SCALING        10

#define SENSORS_ACCELEROMETER_NUM_VALUES        16
#define SENSORS_ACCELEROMETER_FACTOR_SCALING    10

#define SENSORS_MAGNETOMETER_NUM_VALUES         16
#define SENSORS_MAGNETOMETER_FACTOR_SCALING     10



Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(24680);
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
Adafruit_ADXL345_Unified acc = Adafruit_ADXL345_Unified(13579);

byte sensorStatus = 0x0;

char cmdline[COMMAND_LINE_BUFFER_SIZE] = { 0 };
uint8_t cmdlength = 0;

uint32_t sensorBarometerValues[SENSORS_BAROMETER_NUM_VALUES] = { 0 };
uint16_t sensorAccelerometerValues[SENSORS_ACCELEROMETER_NUM_VALUES] = { 0 };
uint16_t sensorMagnetometerValues[SENSORS_MAGNETOMETER_NUM_VALUES] = { 0 };

uint32_t sensorBarometerAverage = 0;
uint16_t sensorAccelerometerAverage = 0;
uint16_t sensorMagnetometerAverage = 0;

uint8_t sensorBarometerReadings = 0;
uint8_t sensorBarometerNextReading = 0;

uint8_t sensorAccelerometerReadings = 0;
uint8_t sensorAccelerometerNextReading = 0;

uint8_t sensorMagnetometerReadings = 0;
uint8_t sensorMagnetometerNextReading = 0;

byte stateMachineState = STATE_GROUND_IDLE;

// Set a default sea level (= 0m ASL) pressure
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

float launchPadHeight = 0.0;

void sensorBarometerCalculateAverage()
{
    uint8_t i = 0;
    uint32_t sum = 0;
    float result = 0.0;
    unsigned long after = 0;
    unsigned long before = millis();

    for(i = 0; i < sensorBarometerReadings; i++)
    {
        sum += sensorBarometerValues[i];
    }
    result = sum / (i * 1.0);
    sensorBarometerAverage = (uint32_t)result;
    after = millis();
    Serial.print("Calculating barometer average took ");
    Serial.print(after-before);
    Serial.print("ms for ");
    Serial.print(i);
    Serial.println(" values:");

    for(i = 0; i < sensorBarometerReadings; i++)
    {
        Serial.print(i);
        Serial.print(": ");
        Serial.print(sensorBarometerValues[i]);
        Serial.println(" Pa");
    }

}

void sensorBarometerAppendValue()
{
    sensorBarometerValues[sensorBarometerNextReading] = sensorBarometerReadValue();
    sensorBarometerNextReading = (sensorBarometerNextReading + 1) & 0xF;
    if(sensorBarometerReadings < SENSORS_BAROMETER_NUM_VALUES)
    {
        sensorBarometerReadings++;
    }
}

void sensorAccelerometerCalculateAverage()
{
    uint8_t i = 0;
    uint32_t sum = 0;
    float result = 0.0;
    unsigned long after = 0;
    unsigned long before = millis();

    for(i = 0; i < sensorAccelerometerReadings; i++)
    {
        sum += sensorAccelerometerValues[i];
    }
    result = sum / (i * 1.0);
    sensorAccelerometerAverage = (uint16_t)result;
    after = millis();
    Serial.print("Calculating accelerometer average took ");
    Serial.print(after-before);
    Serial.print("ms for ");
    Serial.print(i);
    Serial.println(" values:");

    for(i = 0; i < sensorAccelerometerReadings; i++)
    {
        Serial.print(i);
        Serial.print(": ");
        Serial.print(sensorAccelerometerValues[i]);
        Serial.println(" m/s^2");
    }

}

void sensorAccelerometerAppendValue()
{
    sensorAccelerometerValues[sensorAccelerometerNextReading] = sensorAccelerometerReadValue();
    sensorAccelerometerNextReading = (sensorAccelerometerNextReading + 1) & 0xF;
    if(sensorAccelerometerReadings < SENSORS_BAROMETER_NUM_VALUES)
    {
        sensorAccelerometerReadings++;
    }
}

void sensorMagnetometerCalculateAverage()
{
    uint8_t i = 0;
    uint32_t sum = 0;
    float result = 0.0;
    unsigned long after = 0;
    unsigned long before = millis();

    for(i = 0; i < sensorMagnetometerReadings; i++)
    {
        sum += sensorMagnetometerValues[i];
    }
    result = sum / (i * 1.0);
    sensorMagnetometerAverage = (uint32_t)result;
    after = millis();
    Serial.print("Calculating magnetometer average took ");
    Serial.print(after-before);
    Serial.print("ms for ");
    Serial.print(i);
    Serial.println(" values:");

    for(i = 0; i < sensorMagnetometerReadings; i++)
    {
        Serial.print(i);
        Serial.print(": ");
        Serial.print(sensorMagnetometerValues[i]);
        Serial.println(" uT");
    }

}

void sensorMagnetometerAppendValue()
{
    sensorMagnetometerValues[sensorMagnetometerNextReading] = sensorMagnetometerReadValue();
    sensorMagnetometerNextReading = (sensorMagnetometerNextReading + 1) & 0xF;
    if(sensorMagnetometerReadings < SENSORS_BAROMETER_NUM_VALUES)
    {
        sensorMagnetometerReadings++;
    }
}

uint32_t sensorBarometerReadValue()
{
    sensors_event_t event;

    bmp.getEvent(&event);

    if(event.pressure)
    {
        Serial.print("Pressure as float: ");
        Serial.println(event.pressure);
        Serial.print("Pressure as uint32_t: ");
        Serial.println((uint32_t)event.pressure);
        Serial.print("Pressure as uint32_t in Pascal: ");
        Serial.println((uint32_t)(event.pressure * 100.0));
        return (uint32_t)(event.pressure * 100);
    }
    else
    {
        Serial.println("Sensor event error");
        return 0;
    }
}

uint16_t sensorAccelerometerReadValue() {
    sensors_event_t event;

    acc.getEvent(&event);

    if(event.acceleration.x)
    {
        Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
        Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
        Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");
        return (uint16_t)(
            sqrt(
                (event.acceleration.x*event.acceleration.x)
                +
                (event.acceleration.y*event.acceleration.y)
                +
                (event.acceleration.z*event.acceleration.z)
            )
            * SENSORS_ACCELEROMETER_FACTOR_SCALING
        );
    }
    else
    {
        Serial.println("Sensor event error");
        return 0;
    }
}

uint16_t sensorMagnetometerReadValue() {
    return 0;
}

void sensorBarometerGetHeightASL()
{
    sensors_event_t event;

    bmp.getEvent(&event);

    if(event.pressure)
    {
        /* Then convert the atmospheric pressure, and SLP to altitude         */
        /* Update this next line with the current SLP for better results      */
        Serial.print("Altitude:    "); 
        Serial.print(bmp.pressureToAltitude(seaLevelPressure, event.pressure)); 
        Serial.println(" m");
        Serial.println("");
    }
    else
    {
        Serial.println("Sensor event error");
    }
}

void sensorBarometerReadTemperature()
{
    sensors_event_t event;

    bmp.getEvent(&event);

    if(event.temperature)
    {
        /* First we get the current temperature from the BMP085 */
        float temperature;
        bmp.getTemperature(&temperature);
        Serial.print("Temperature: ");
        Serial.print(temperature);
        Serial.println(" C");
    }
    else
    {
        Serial.println("Sensor event error");
    }
}




uint8_t parseCommandConfiguration(uint8_t j)
{
    switch(cmdline[j++])
    {
        case 'A': // Averaging
        {
            Serial.println("Averaging");
            switch(cmdline[j++])
            {
                case 'A': // Accelerometer
                {
                    Serial.println("Accelerometer");
                    break;
                }

                case 'B': // Barometer
                {
                    Serial.println("Barometer");
                    break;
                }

                case 'M': // Magnetometer
                {
                    Serial.println("Magnetometer");
                    break;
                }

                default:
                    break;
            }
            break;
        }
        
        case 'G': // Launch pad height
        {
            Serial.println("Sea level pressure");
            break;
        }

        case 'H': // Launch pad height
        {
            Serial.println("Height");
            break;
        }

        case 'L': // Limits
        {
            Serial.println("Limits");

            switch(cmdline[j++])
            {
                case 'A': // Accelerometer
                {
                    Serial.println("Accelerometer");

                    switch(cmdline[j++])
                    {
                        case 'I': // Inbound
                        {
                            Serial.println("Inbound");
                            break;
                        }
                        
                        case 'O': // Outbound
                        {
                            Serial.println("Outbound");
                            break;
                        }

                        default:
                            break;

                    }
                    break;
                }

                case 'B': // Barometer
                {
                    Serial.println("Barometer");
                    break;
                }

                case 'M': // Magnetometer
                {
                    Serial.println("Magnetometer");
                    break;
                }

                default:
                    break;
            }
            break;
        }
        
        case 'T': // Timeouts
        {
            Serial.println("Timeouts");
            switch(cmdline[j++])
            {
                case 'I': // Inbound
                {
                    Serial.println("Inbound");
                    break;
                }

                case 'O': // Outbound
                {
                    Serial.println("Outbound");
                    break;
                }

                default:
                    break;
            }
            break;
        }
        
        default:
            break;
    }

    return j;
}



/** Remove all characters in array up to the specified character.
 * @param   i   The last character to remove (eg. linefeed).
 */
void stripCommand(uint8_t i)
{
    uint8_t j = 0;
    
    if(i == 0)
    {
        return;
    }

    if(i > COMMAND_LINE_BUFFER_SIZE)
    {
        // This is an error, but we "solve" it by clearing the whole buffer.
        i = COMMAND_LINE_BUFFER_SIZE;
    }
    else
    {
        // Shift the rest of the command buffer i characters to the left
        for(j = 0; j < COMMAND_LINE_BUFFER_SIZE; j++)
        {
            if(j < COMMAND_LINE_BUFFER_SIZE - (i + 1))
            {
                cmdline[j] = cmdline[j + i];
            }
            else
            {
                cmdline[j] = '\0';
            }
            
        }
    }
    
    // Decrement length counter
    if(i > cmdlength)
    {
        cmdlength = 0;
    }
    else
    {
        cmdlength -= i;
    }

    /*
    Serial.print("Command string: [");
    Serial.flush();
    for(j = 0; j < COMMAND_LINE_BUFFER_SIZE; j++)
    {
        Serial.print(cmdline[j] > 20 ? cmdline[j] : '.');
        Serial.flush();
    }
    Serial.println("]");
    Serial.flush();
    */
}



uint8_t parseCommand(void)
{
    uint8_t i = 0;
    uint8_t j = 0;
    
    // Find the first newline
    for(i = 0; i <= cmdlength; i++)
    {
        if(cmdline[i] == ';')
        {
            //Serial.print("Newline at position ");
            //Serial.println(i);
            //Serial.flush();
            break;
        }
    }

    if(i == 0)
    {
        return 1;
    }

    if(i > cmdlength)
    {
        //Serial.println("No newline found!");
        //Serial.flush();
        if(cmdlength <= COMMAND_LINE_APPEND_THRESHOLD)
        {
            // Not to worry, maybe a newline arrives shortly...
            return 0;
        }

        // We're above threshold for reading in new bytes,
        //   and there is no newline in our buffer!
        if(cmdlength > COMMAND_LINE_APPEND_THRESHOLD)
        {
            // Start shifting bytes to the left in search of a command
            return 1;
        }
    }

    if(i > 0)
    {
        // It can theoretically be a command at only 2 bytes ("P\n")
        switch(cmdline[j++])
        {
            Serial.print(millis());
            Serial.print(": ");

            case 'C': // Configuration
            {
                Serial.print("Configuration command ");
                if(
                    (stateMachineState == STATE_GROUND_IDLE)
                    ||
                    (stateMachineState == STATE_GROUND_IDLE_ON_PAD)
                )
                {
                    Serial.println("input");
                    j = parseCommandConfiguration(j);
                    if(j < i)
                    {
                        Serial.println("There were leftover characters!");
                    }
                    return j;
                }
                else
                {
                    Serial.println("in wrong mode!");
                    // Ignore the command
                    return i;
                }
                break;
            }

            case 'D': // Debug
            {
                Serial.print("Current state is S");
                Serial.println(stateMachineState);
                Serial.print("System time is ");
                Serial.println(millis());
                Serial.print("Command buffer length: ");
                Serial.println(cmdlength);

                sensorBarometerAppendValue();
                sensorBarometerCalculateAverage();
                Serial.print("Average: ");
                Serial.print(sensorBarometerAverage);
                Serial.println(" Pa");
                //sensorBarometerReadValue();
                sensorBarometerReadTemperature();
                //sensorBarometerGetHeightASL();


                sensorAccelerometerAppendValue();
                sensorAccelerometerCalculateAverage();
                Serial.print("Average: ");
                Serial.print(sensorAccelerometerAverage);
                Serial.println(" m/s^2");


                /*
                sensorMagnetometerAppendValue();
                sensorMagnetometerCalculateAverage();
                Serial.print("Average: ");
                Serial.print(sensorMagnetometerAverage);
                Serial.println(" uT");
                */

                return i;
                break;
            }

            case 'P': // Ping
            {
                Serial.println("P");
                return i;
                break;
            }
            
            case 'S': // State transition
            {
                if(j == i)
                {
                    // Report current state
                    Serial.print("S");
                    Serial.println(stateMachineState);
                    return i;
                }

                Serial.print("State change ");
                j = (uint8_t)cmdline[j++] - 0x30;
                if(j >= 8)
                {
                    // Error!
                    Serial.println("error!");
                    return i;
                    break;
                }
                Serial.print("from S");
                Serial.print(stateMachineState);
                Serial.print(" to S");
                Serial.println(j);
                stateMachineState = j;
                return i;
                break;
            }
            
            default:
            {
                Serial.println("E: Command not recognized.");
                return i;
                break;
            }
        } // switch(cmdline[0])
    } // if(i > 1)
} // parseCommand()



void displaySensorDetails(void)
{
    sensor_t sensor;

    if(!sensorStatus)
    {
        Serial.println("No sensors detected!");
        return;
    }
    
    if(GET_BIT(sensorStatus, STATUS_BAROMETER_DETECTED))
    {
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
        Serial.flush();
    }
    
    if(GET_BIT(sensorStatus, STATUS_MAGNETOMETER_DETECTED))
    {
        mag.getSensor(&sensor);
        Serial.println("------------------------------------");
        Serial.print  ("Sensor:       "); Serial.println(sensor.name);
        Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
        Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
        Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
        Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
        Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
        Serial.println("------------------------------------");
        Serial.println("");
        Serial.flush();
    }

    if(GET_BIT(sensorStatus, STATUS_ACCELEROMETER_DETECTED))
    {
        acc.getSensor(&sensor);
        Serial.println("------------------------------------");
        Serial.print  ("Sensor:       "); Serial.println(sensor.name);
        Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
        Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
        Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
        Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
        Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");  
        Serial.println("------------------------------------");
        Serial.println("");
        Serial.flush();
    }
}



void stateMachine()
{
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
            stateMachineState = STATE_AIRBORNE_INBOUND;
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
}


/** Check for bytes in UART receive buffer, and append them to our own command
 *    buffer, or at least as many as there is room for at the moment.
 */
void grabSerial()
{
    uint8_t cmdlen = 0;
    uint8_t i = 0;
    
    // Only get more data for command buffer if it's below threshold.
    if(cmdlength > COMMAND_LINE_APPEND_THRESHOLD)
    {
        return;
    }

    // Check for bytes in UART receive buffer
    cmdlen = Serial.available();
    if(cmdlen > 0)
    {
        // Don't try to overfill the command buffer
        if((cmdlength + cmdlen) > COMMAND_LINE_BUFFER_SIZE)
        {
            // Only fill it...
            cmdlen = COMMAND_LINE_BUFFER_SIZE - cmdlength;
        }

        i = Serial.readBytes((char*)(&cmdline[cmdlength]), cmdlen);
        
        //Serial.print("Read ");
        //Serial.print(i);
        //Serial.print(" of ");
        //Serial.print(cmdlen);
        //Serial.println(" bytes");
        
        cmdlength += i;

        // Don't try to terminate past end of buffer
        if(cmdlength == COMMAND_LINE_BUFFER_SIZE)
        {
            Serial.println("Command buffer at capacity!");
            return;
        }
        if(cmdlength > COMMAND_LINE_BUFFER_SIZE)
        {
            Serial.println("Command buffer capacity exceeded!");
            return;
        }
        
        cmdline[cmdlength] = '\0';

    }

}


void setup()
{
    Serial.begin(115200);
    Serial.println("setup() begins");

    Serial.print("UART RX Buffer size: ");
    Serial.println(SERIAL_RX_BUFFER_SIZE);

    Serial.print("Command line buffer size: ");
    Serial.println(COMMAND_LINE_BUFFER_SIZE);

    Serial.print("Command line fill threshold: ");
    Serial.println(COMMAND_LINE_APPEND_THRESHOLD);

    Serial.println("Clearing command buffer...");
    for(cmdlength = COMMAND_LINE_BUFFER_SIZE; cmdlength > 0; cmdlength--)
    {
        cmdline[cmdlength-1] = '\0';
    }
    
    Serial.print("Command line length: ");
    Serial.println(cmdlength);

    Serial.println("Setting up pins...");

    pinMode(INTSRC_INTERRUPT, INPUT);
    //pinMode(INTSRC_GYROMETER, INPUT);
    pinMode(INTSRC_BAROMETER, INPUT);
    pinMode(INTSRC_THERMOMETER, INPUT);
    pinMode(INTSRC_ACCELEROMETER, INPUT);
    pinMode(INTSRC_MAGNETOMETER, INPUT);

    Serial.println("Initializing sensors...");
    
    Serial.println("Barometer...");
    SET_BIT(sensorStatus, STATUS_BAROMETER_DETECTED);
    if (!bmp.begin()) {
        Serial.println("ERROR: BMP085 not detected!");
        CLEAR_BIT(sensorStatus, STATUS_BAROMETER_DETECTED);
    }

    Serial.println("Magnetometer...");
    SET_BIT(sensorStatus, STATUS_MAGNETOMETER_DETECTED);
    if(!mag.begin())
    {
        Serial.println("ERROR: HMC5883 not detected!");
        CLEAR_BIT(sensorStatus, STATUS_MAGNETOMETER_DETECTED);
    }

    Serial.println("Accelerometer...");
    SET_BIT(sensorStatus, STATUS_ACCELEROMETER_DETECTED);
    if(!acc.begin())
    {
        Serial.println("ERROR: ADXL345 not detected!");
        CLEAR_BIT(sensorStatus, STATUS_ACCELEROMETER_DETECTED);
    }

    // At least one sensor was initialized...
    displaySensorDetails();
    
    Serial.print("setup() ends at ");
    Serial.println(millis());
}



void loop()
{
    uint8_t i = 0;

    grabSerial();

    i = parseCommand();

    if(i > 0)
    {
        stripCommand(i);
    }

    stateMachine();

#if 0
#endif // if 0
    
    delay(10);
}
