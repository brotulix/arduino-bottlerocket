// If using eg. Visual Studio Code with Arduino plugin, ARDUINO may not be set:
#ifndef ARDUINO
#define ARDUINO 189
#endif

#include <Wire.h>
#include <Adafruit_Sensor.h>        // *modified* Adafruit Unified Sensor library
#include <Adafruit_BMP085_U.h>      // BMP085 Barometer
#include <Adafruit_ADXL345_U.h>     // ADXL345 Accelerometer
#include <Adafruit_HMC5883_U.h>     // HMC5883 Magnetometer

#define SET_BIT(a, b)   (a |= (0x1 << b))
#define CLEAR_BIT(a, b) (a &= ~(0x1 << b))
#define GET_BIT(a, b)   (a & (0x1 << b))

#define INTSRC_INTERRUPT                        2
#define INTSRC_GYROMETER                        -1
#define INTSRC_BAROMETER                        4
#define INTSRC_THERMOMETER                      5
#define INTSRC_ACCELEROMETER                    6
#define INTSRC_MAGNETOMETER                     7
#define I2C_SDA                                 18
#define I2C_SCL                                 19

#define STATUS_BAROMETER_DETECTED               0
#define STATUS_ACCELEROMETER_DETECTED           1
#define STATUS_GYROMETER_DETECTED               2
#define STATUS_MAGNETOMETER_DETECTED            3

#define STATUS_UPDATE                           0
#define STATUS_UPDATE_AVERAGE                   1
#define STATUS_REPORT                           2

#define STATE_GROUND_IDLE                       0   // ### Ground, idle state
#define STATE_GROUND_IDLE_ON_PAD                1   // ### Ground, on-pad idle state
#define STATE_GROUND_ARMED                      2   // ### Ground, armed state
#define STATE_AIRBORNE_OUTBOUND                 3   // ### Airborne, outbound state
#define STATE_AIRBORNE_DEPLOYMENT               4   // ### Airborne, deployment state
#define STATE_AIRBORNE_INBOUND                  5   // ### Airborne, inbound state
#define STATE_GROUND_RECOVERY                   6   // ### Ground, recovery state
#define STATE_GROUND_DATADUMP                   7   // ### Ground, data-dump state

#define COMMAND_LINE_BUFFER_SIZE                16
#define COMMAND_LINE_APPEND_THRESHOLD           ((COMMAND_LINE_BUFFER_SIZE / 4) * 3)

#define SENSORS_BAROMETER_NUM_VALUES            16
#define SENSORS_BAROMETER_FACTOR_SCALING        100
#define SENSORS_BAROMETER_SEALEVELHPA           (SENSORS_PRESSURE_SEALEVELHPA * SENSORS_BAROMETER_FACTOR_SCALING)

#define SENSORS_ACCELEROMETER_NUM_VALUES        16
#define SENSORS_ACCELEROMETER_FACTOR_SCALING    10

#define SENSORS_MAGNETOMETER_NUM_VALUES         16
#define SENSORS_MAGNETOMETER_FACTOR_SCALING     10

typedef struct {
    float values[16];
    float prev_average;
    float average;
    uint8_t filled_elements;
    uint8_t next_element_to_fill;
    uint16_t spare;
} savg16;

typedef struct {
    uint32_t millis_previous_loop;
    uint16_t interval_refresh_accelerometer;
    uint16_t interval_refresh_magnetometer;
    uint16_t interval_refresh_barometer;
    uint16_t interval_refresh_average_accelerometer;
    uint16_t interval_refresh_average_magnetometer;
    uint16_t interval_refresh_average_barometer;
    uint16_t interval_report_accelerometer;
    uint16_t interval_report_magnetometer;
    uint16_t interval_report_barometer;
    uint16_t limit_delta_barometer;
    uint16_t limit_delta_accelerometer;
    uint16_t time_to_next_update_accelerometer;
    uint16_t time_to_next_update_magnetometer;
    uint16_t time_to_next_update_barometer;
    uint16_t time_to_next_update_average_accelerometer;
    uint16_t time_to_next_update_average_magnetometer;
    uint16_t time_to_next_update_average_barometer;
    uint16_t time_to_next_report_accelerometer;
    uint16_t time_to_next_report_magnetometer;
    uint16_t time_to_next_report_barometer;
    uint16_t panic_timeout;
} sconfig;

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(24680);
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
Adafruit_ADXL345_Unified acc = Adafruit_ADXL345_Unified(13579);

char cmdline[COMMAND_LINE_BUFFER_SIZE] = { 0 };
uint8_t cmdlength = 0;

sconfig configuration = {
    0,   // millis_previous_loop
    5,  // interval_refresh_accelerometer
    50,  // interval_refresh_magnetometer
    25,  // interval_refresh_barometer
    25,  // interval_refresh_average_accelerometer
    250,  // interval_refresh_average_magnetometer
    100,  // interval_refresh_average_barometer
    25,  // interval_report_accelerometer
    250,  // interval_report_magnetometer
    100,  // interval_report_barometer
    250,  // limit_delta_barometer
    250,  // limit_delta_accelerometer
    0,  // time_to_next_update_accelerometer
    0,  // time_to_next_update_magnetometer
    0,  // time_to_next_update_barometer
    0,  // time_to_next_update_average_accelerometer
    0,  // time_to_next_update_average_magnetometer
    0,  // time_to_next_update_average_barometer
    0, // time_to_next_report_accelerometer
    0, // time_to_next_report_magnetometer
    0, // time_to_next_report_barometer
    3500 // panic_timeout
};

savg16 valsBarometer         = {};
savg16 valsAccelerometer     = {};
savg16 valsMagnetometer      = {};

byte stateMachineState = STATE_GROUND_IDLE;

// Set a default sea level (= 0m ASL) pressure
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

float peakAltitudePressure = -1;

float launchPadHeight = 0;

// How many consequitive positive barometer deltas to read in a row to say we're
//   steadily falling
#define SENSORS_BAROMETER_STRIKES   3
uint8_t barometerdeltastrikes = 3;

uint8_t panicActivate = 0;
uint16_t panicTimer = -1;

uint8_t append32(savg16 *vals, float value)
{
    // 0 is an error value
    if(value == 0)
    {
        return vals->filled_elements;
    }

    vals->values[vals->next_element_to_fill] = value;
    vals->next_element_to_fill = (vals->next_element_to_fill + 1) & 0xF;
    if(vals->filled_elements < 16)
    {
        vals->filled_elements++;
    }

    return vals->filled_elements;
}

float vlen3D(float x, float y, float z)
{
    return sqrt(
        (x*x)
        +
        (y*y)
        +
        (z*z)
    );
}

float favg32(savg16 *vals)
{
    uint8_t i = 0;
    uint8_t j = 0;
    int64_t sum = 0;
    int32_t avg = 0;

    // Catch the first call.
    if(vals->filled_elements == 0)
    {
        return 0;
    }

    // Point to the first element:
    j = (vals->next_element_to_fill + 16 - vals->filled_elements) & 0xF;

    for(i = 0; i < vals->filled_elements; i++)
    {
        sum += vals->values[j];        
        j = (j + 1) & 0xF;
    }

    avg = sum / vals->filled_elements;

    vals->prev_average = vals->average;
    vals->average = avg;

    return avg;

}

float sensorBarometerReadValue()
{
    sensors_event_t event;

    bmp.getEvent(&event);

    if(event.pressure)
    {
        return event.pressure;
    }
    else
    {
        //Serial.println("Barometer event error");
        return 0.0;
    }
}

float sensorAccelerometerReadValue()
{
    sensors_event_t event;

    acc.getEvent(&event);

    if(event.acceleration.x || event.acceleration.y || event.acceleration.z)
    {
        return vlen3D(event.acceleration.x, event.acceleration.y, event.acceleration.z);
    }
    else
    {
        //Serial.println("Accelerometer event error");
        return 0.0;
    }
}

float sensorMagnetometerReadValue()
{
    sensors_event_t event; 
    mag.getEvent(&event);

    if(event.magnetic.x || event.magnetic.y || event.magnetic.z)
    {
        return vlen3D(event.magnetic.x, event.magnetic.y, event.magnetic.z);
    }
    else
    {
        //Serial.println("Magnetometer event error");
        return 0;
    }
    
}


#if 0
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
#endif

uint8_t parseCommandConfiguration(uint8_t j)
{
    switch(cmdline[j++])
    {
        case 'A': // Averaging
        {
            //Serial.println("Averaging");
            switch(cmdline[j++])
            {
                case 'A': // Accelerometer
                {
                    //Serial.println("Accelerometer");
                    break;
                }

                case 'B': // Barometer
                {
                    //Serial.println("Barometer");
                    break;
                }

                case 'M': // Magnetometer
                {
                    //Serial.println("Magnetometer");
                    break;
                }

                default:
                    break;
            }
            break;
        }
        
        case 'G': // Launch pad height
        {
            //Serial.println("Sea level pressure");
            break;
        }

        case 'H': // Launch pad height
        {
            //Serial.println("Height");
            break;
        }

        case 'L': // Limits
        {
            //Serial.println("Limits");

            switch(cmdline[j++])
            {
                case 'A': // Accelerometer
                {
                    //Serial.println("Accelerometer");

                    switch(cmdline[j++])
                    {
                        case 'I': // Inbound
                        {
                            //Serial.println("Inbound");
                            break;
                        }
                        
                        case 'O': // Outbound
                        {
                            //Serial.println("Outbound");
                            break;
                        }

                        default:
                            break;

                    }
                    break;
                }

                case 'B': // Barometer
                {
                    //Serial.println("Barometer");
                    break;
                }

                case 'M': // Magnetometer
                {
                    //Serial.println("Magnetometer");
                    break;
                }

                default:
                    break;
            }
            break;
        }
        
        case 'T': // Timeouts
        {
            //Serial.println("Timeouts");
            switch(cmdline[j++])
            {
                case 'I': // Inbound
                {
                    //Serial.println("Inbound");
                    break;
                }

                case 'O': // Outbound
                {
                    //Serial.println("Outbound");
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
                //Serial.print("Configuration command ");
                if(
                    (stateMachineState == STATE_GROUND_IDLE)
                    ||
                    (stateMachineState == STATE_GROUND_IDLE_ON_PAD)
                )
                {
                    //Serial.println("input");
                    j = parseCommandConfiguration(j);
                    if(j < i)
                    {
                        //Serial.println("There were leftover characters!");
                    }
                    return j;
                }
                else
                {
                    //Serial.println("in wrong mode!");
                    // Ignore the command
                    return i;
                }
                break;
            }

            case 'D': // Debug
            {
                Serial.println("----- DEBUG -----");
                
                Serial.print("Current state is S");
                Serial.println(stateMachineState);
                Serial.print("System time is ");
                Serial.println(millis());
                Serial.print("Command buffer length: ");
                Serial.println(cmdlength);
                
                //sensorBarometerReadTemperature();
                
                doBarometerActions(0, ((1 << STATUS_REPORT)|(1 << STATUS_UPDATE)|(1 << STATUS_UPDATE_AVERAGE)));
                
                doAccelerometerActions(0, ((1 << STATUS_REPORT)|(1 << STATUS_UPDATE)|(1 << STATUS_UPDATE_AVERAGE)));
                
                doMagnetometerActions(0, ((1 << STATUS_REPORT)|(1 << STATUS_UPDATE)|(1 << STATUS_UPDATE_AVERAGE)));

                Serial.print("Current Accelerometer value: ");
                Serial.println(sensorAccelerometerReadValue());

                Serial.print("Current Accelerometer vector: ");
                
                sensors_event_t event; 
                acc.getEvent(&event);

                if(event.acceleration.x || event.acceleration.y || event.acceleration.z)
                {
                    Serial.print("[");
                    Serial.print(event.acceleration.x);
                    Serial.print(", ");
                    Serial.print(event.acceleration.y);
                    Serial.print(",  ");
                    Serial.print(event.acceleration.z);
                    Serial.print("] |v| = ");
                    Serial.println(vlen3D(event.acceleration.x, event.acceleration.y, event.acceleration.z));
                }
                else
                {
                    Serial.println("Error");
                }

                Serial.print("Current Barometer value: ");
                Serial.println(sensorBarometerReadValue());

                Serial.print("Current Magnetometer value: ");
                Serial.println(sensorMagnetometerReadValue());
                
                Serial.print("Current Magnetometer vector: ");
                mag.getEvent(&event);

                if(event.magnetic.x || event.magnetic.y || event.magnetic.z)
                {
                    Serial.print("[");
                    Serial.print(event.magnetic.x);
                    Serial.print(", ");
                    Serial.print(event.magnetic.y);
                    Serial.print(",  ");
                    Serial.print(event.magnetic.z);
                    Serial.print("] |v| = ");
                    Serial.println(vlen3D(event.magnetic.x, event.magnetic.y, event.magnetic.z));
                }
                else
                {
                    Serial.println("Error");
                }

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

                //Serial.print("State change ");
                j = (uint8_t)cmdline[j++] - 0x30;
                if(j >= 8)
                {
                    // Error!
                    Serial.println("error!");
                    return i;
                    break;
                }
                //Serial.print("from S");
                //Serial.print(stateMachineState);
                //Serial.print(" to S");
                //Serial.println(j);
                stateMachineState = j;
                return i;
                break;
            }
            
            default:
            {
                //Serial.println("E: Command not recognized.");
                Serial.println("E");
                return i;
                break;
            }
        } // switch(cmdline[0])
    } // if(i > 1)

    return 0;
} // parseCommand()

void displaySensorDetails(uint8_t sensorStatus)
{
#define SENSOR_REPORT 0
#if SENSOR_REPORT
    sensor_t sensor;
#endif

    if(!sensorStatus)
    {
        Serial.println("No sensors detected!");
        return;
    }
    
    if(!(
        GET_BIT(sensorStatus, STATUS_BAROMETER_DETECTED) && 
        GET_BIT(sensorStatus, STATUS_MAGNETOMETER_DETECTED) && 
        GET_BIT(sensorStatus, STATUS_ACCELEROMETER_DETECTED)
    ))
    {
        Serial.println("E: At least one sensor not detected.");
    }

#if SENSOR_REPORT
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
#endif
}

void doAccelerometerActions(uint16_t delta_millis, uint8_t force)
{
    uint8_t update = 0;
    int32_t diff = 0;

    if(delta_millis >= configuration.time_to_next_update_average_accelerometer)
    {
        update = 1;
        configuration.time_to_next_update_average_accelerometer = configuration.interval_refresh_average_accelerometer;
    }
    else {
        update = 0;
        configuration.time_to_next_update_average_accelerometer -= delta_millis;
    }

    if(update | GET_BIT(force, STATUS_UPDATE_AVERAGE))
    {
        //Serial.println();
        //Serial.print(millis());
        //Serial.println(": Aavg");
        favg32(&valsAccelerometer);
    }

    if(delta_millis >= configuration.time_to_next_report_accelerometer)
    {
        update = 1;
        configuration.time_to_next_report_accelerometer = configuration.interval_report_accelerometer;
    }
    else {
        update = 0;
        configuration.time_to_next_report_accelerometer -= delta_millis;
    }

    if(update | GET_BIT(force, STATUS_REPORT))
    {
        if(valsAccelerometer.filled_elements)
        {
            //Serial.println();
            Serial.print(millis());
            Serial.print("\tR\tA\t");

            //Serial.print("Previous Average: ");
            Serial.print(valsAccelerometer.prev_average);
            Serial.print("\t");
            //Serial.println(" m/s^2");

            //Serial.print("Average: ");
            Serial.print(valsAccelerometer.average);
            Serial.print("\t");
            //Serial.println(" m/s^2");

            if(valsAccelerometer.prev_average > valsAccelerometer.average)
            {
                diff = -1 * (valsAccelerometer.prev_average - valsAccelerometer.average);
            }
            else
            {
                diff = valsAccelerometer.average - valsAccelerometer.prev_average;
            }

            //Serial.print("Delta: ");
            Serial.print(diff);
            Serial.println();
            //Serial.println(" m/s^2");
    }
    }

    if(delta_millis >= configuration.time_to_next_update_accelerometer)
    {
        update = 1;
        configuration.time_to_next_update_accelerometer = configuration.interval_refresh_accelerometer;
    }
    else {
        update = 0;
        configuration.time_to_next_update_accelerometer -= delta_millis;
    }

    if(update | GET_BIT(force, STATUS_UPDATE))
    {
        //Serial.println();
        //Serial.print(millis());
        //Serial.println(": Aapd");
        append32(&valsAccelerometer, sensorAccelerometerReadValue());
    }
}

void doMagnetometerActions(uint16_t delta_millis, uint8_t force)
{
    uint8_t update = 0;
    int32_t diff = 0;

    if(delta_millis >= configuration.time_to_next_update_average_magnetometer)
    {
        update = 1;
        configuration.time_to_next_update_average_magnetometer = configuration.interval_refresh_average_magnetometer;
    }
    else {
        update = 0;
        configuration.time_to_next_update_average_magnetometer -= delta_millis;
    }

    if(update | GET_BIT(force, STATUS_UPDATE_AVERAGE))
    {
        //Serial.println();
        //Serial.print(millis());
        //Serial.println(": Mavg");
        favg32(&valsMagnetometer);
    }

    if(delta_millis >= configuration.time_to_next_report_magnetometer)
    {
        update = 1;
        configuration.time_to_next_report_magnetometer = configuration.interval_report_magnetometer;
    }
    else {
        update = 0;
        configuration.time_to_next_report_magnetometer -= delta_millis;
    }

    if(update | GET_BIT(force, STATUS_REPORT))
    {
        if(valsMagnetometer.filled_elements)
        {
            //Serial.println();
            Serial.print(millis());
            Serial.print("\tR\tM\t");
            //Serial.println(": Mrpt");

            //Serial.print("Previous Average: ");
            Serial.print(valsMagnetometer.prev_average);
            Serial.print("\t");
            //Serial.println(" uT");

            //Serial.print("Average: ");
            Serial.print(valsMagnetometer.average);
            Serial.print("\t");
            //Serial.println(" uT");

            if(valsMagnetometer.prev_average > valsMagnetometer.average)
            {
                diff = -1 * (valsMagnetometer.prev_average - valsMagnetometer.average);
            }
            else
            {
                diff = valsMagnetometer.average - valsMagnetometer.prev_average;
            }

            //Serial.print("Delta: ");
            Serial.print(diff);
            Serial.println();
            //Serial.println(" uT");
    }
    }

    if(delta_millis >= configuration.time_to_next_update_magnetometer)
    {
        update = 1;
        configuration.time_to_next_update_magnetometer = configuration.interval_refresh_magnetometer;
    }
    else {
        update = 0;
        configuration.time_to_next_update_magnetometer -= delta_millis;
    }

    if(update | GET_BIT(force, STATUS_UPDATE))
    {
        //Serial.println();
        //Serial.print(millis());
        //Serial.println(": Mapd");
        append32(&valsMagnetometer, sensorMagnetometerReadValue());
    }
}

void doBarometerActions(uint16_t delta_millis, uint8_t force)
{
    uint8_t update = 0;
    int32_t diff = 0;

    if(delta_millis >= configuration.time_to_next_update_average_barometer)
    {
        update = 1;
        configuration.time_to_next_update_average_barometer = configuration.interval_refresh_average_barometer;
    }
    else {
        update = 0;
        configuration.time_to_next_update_average_barometer -= delta_millis;
    }

    if(update | GET_BIT(force, STATUS_UPDATE_AVERAGE))
    {
        //Serial.println();
        //Serial.print(millis());
        //Serial.println(": Bavg");
        favg32(&valsBarometer);
    }

    if(delta_millis >= configuration.time_to_next_report_barometer)
    {
        update = 1;
        configuration.time_to_next_report_barometer = configuration.interval_report_barometer;
    }
    else {
        update = 0;
        configuration.time_to_next_report_barometer -= delta_millis;
    }

    if(update | GET_BIT(force, STATUS_REPORT))
    {
        if(valsBarometer.filled_elements)
        {
            //Serial.println();
            Serial.print(millis());
            Serial.print("\tR\tB\t");
            //Serial.println(": Brpt");

            //Serial.print("Previous Average: ");
            Serial.print(valsBarometer.prev_average);
            Serial.print("\t");
            //Serial.println(" hPa");

            //Serial.print("Average: ");
            Serial.print(valsBarometer.average);
            Serial.print("\t");
            //Serial.println(" hPa");

            if(valsBarometer.prev_average > valsBarometer.average)
            {
                diff = -1 * (valsBarometer.prev_average - valsBarometer.average);
            }
            else
            {
                diff = valsBarometer.average - valsBarometer.prev_average;
            }

            //Serial.print("Delta: ");
            Serial.print(diff);
            Serial.println();
            //Serial.println(" hPa");
        }
    }

    if(delta_millis >= configuration.time_to_next_update_barometer)
    {
        update = 1;
        configuration.time_to_next_update_barometer = configuration.interval_refresh_barometer;
    }
    else {
        update = 0;
        configuration.time_to_next_update_barometer -= delta_millis;
    }

    if(update | GET_BIT(force, STATUS_UPDATE))
    {
        //Serial.println();
        //Serial.print(millis());
        //Serial.println(": Bapd");
        append32(&valsBarometer, sensorBarometerReadValue());
    }
}

void panicTimerCountdown()
{
    if(!panicActivate)
    {
        if(!panicTimer--)
        {
            panicActivate = 1;
            Serial.println("Panic timeout!");
        }
    }
}

void panicTimerReset()
{
    panicTimer = configuration.panic_timeout;
    panicActivate = 0;
}

void releaseTheKrakenshoot()
{
    // Release the parachute by applying a different angle to the servo
    // or applying power to a motor
    // or something...
}

void engageAudioVisualBeacon()
{
    // Toggle power to audiovisual beacon
}

void stateMachine()
{
    int32_t diff = 0;
    switch(stateMachineState)
    {
        case STATE_GROUND_IDLE_ON_PAD:
        {
            break;
        }
        
        case STATE_GROUND_ARMED:
        {
            // Check the two possible launch-indicative magnutides

            // Accelerometer
            if(valsAccelerometer.prev_average > valsAccelerometer.average)
            {
                diff = valsAccelerometer.prev_average - valsAccelerometer.average;
            }
            else
            {
                diff = valsAccelerometer.average - valsAccelerometer.prev_average;
            }
            
            if(diff > configuration.limit_delta_accelerometer)
            {
                Serial.println("ACCELEROMETER MAGNITUDE ABOVE THRESHOLD");
                stateMachineState = STATE_AIRBORNE_OUTBOUND;
                panicTimerReset();
            }

            // Barometer
            if(valsBarometer.prev_average > valsBarometer.average)
            {
                diff = valsBarometer.prev_average - valsBarometer.average;
            }
            else
            {
                diff = valsBarometer.average - valsBarometer.prev_average;
            }
            
            if(diff > configuration.limit_delta_barometer)
            {
                Serial.println("BAROMETER MAGNITUDE ABOVE THRESHOLD");
                stateMachineState = STATE_AIRBORNE_OUTBOUND;
                panicTimerReset();
            }

            break;
        }
        
        case STATE_AIRBORNE_OUTBOUND:
        {

            // Check for lowest pressure yet
            if(valsBarometer.average < peakAltitudePressure)
            {
                peakAltitudePressure = valsBarometer.average;
            }

            // Check for three consequitive positive barometer delta, indicating a steadily decreasing height
            // A single positive value could perhaps simply be from pressure oscillations inside the payload capsule due to turbulence during launch, so we could get a premature deployment of parachute?
            if(valsBarometer.prev_average > valsBarometer.average)
            {
                diff = valsBarometer.prev_average - valsBarometer.average;
                Serial.println("BAROMETER MAGNITUDE ABOVE THRESHOLD");
                barometerdeltastrikes--;
            }
            else
            {
                barometerdeltastrikes = SENSORS_BAROMETER_STRIKES;
            }
            
            if(barometerdeltastrikes == 0)
            {
                Serial.println("THREE STRIKES; DEPLOYING");
                stateMachineState = STATE_AIRBORNE_DEPLOYMENT;
            }

            if(panicActivate)
            {
                Serial.println("DEPLOYING DUE TO PANIC TIMEOUT");
                stateMachineState = STATE_AIRBORNE_DEPLOYMENT;
            }

            break;
        }
        
        case STATE_AIRBORNE_DEPLOYMENT:
        {
            panicTimerReset();
            stateMachineState = STATE_AIRBORNE_INBOUND;
            break;
        }
        
        case STATE_AIRBORNE_INBOUND:
        {
            if(panicActivate)
            {
                Serial.println("Activating locator beacon due to panic timeout.");
                stateMachineState = STATE_GROUND_RECOVERY;
            }
            break;
        }
        
        case STATE_GROUND_RECOVERY:
        {
            // Wait for RBF to be re-applied
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
    uint8_t sensorStatus = 0;
    Serial.begin(115200);
    Serial.println("setup() begins");

    /*
    Serial.print("UART RX Buffer size: ");
    Serial.println(SERIAL_RX_BUFFER_SIZE);

    Serial.print("Command line buffer size: ");
    Serial.println(COMMAND_LINE_BUFFER_SIZE);

    Serial.print("Command line fill threshold: ");
    Serial.println(COMMAND_LINE_APPEND_THRESHOLD);
    */
    
    //Serial.println("Clearing command buffer...");
    for(cmdlength = COMMAND_LINE_BUFFER_SIZE; cmdlength > 0; cmdlength--)
    {
        cmdline[cmdlength-1] = '\0';
    }
    
    /*
    Serial.print("Command line length: ");
    Serial.println(cmdlength);
    */
    
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
        Serial.println("E");
        CLEAR_BIT(sensorStatus, STATUS_BAROMETER_DETECTED);
    }

    Serial.println("Magnetometer...");
    SET_BIT(sensorStatus, STATUS_MAGNETOMETER_DETECTED);
    if(!mag.begin())
    {
        Serial.println("E");
        CLEAR_BIT(sensorStatus, STATUS_MAGNETOMETER_DETECTED);
    }

    Serial.println("Accelerometer...");
    SET_BIT(sensorStatus, STATUS_ACCELEROMETER_DETECTED);
    if(!acc.begin())
    {
        Serial.println("E");
        CLEAR_BIT(sensorStatus, STATUS_ACCELEROMETER_DETECTED);
    }

    // At least one sensor was initialized...
    displaySensorDetails(sensorStatus);
    
    Serial.print("setup() ends at ");
    Serial.println(millis());
}

uint16_t getDeltaMillis()
{
    uint16_t delta_millis = 0;
    uint32_t this_millis = millis();

    // Check for wrap-around of millis counter
    if(this_millis < configuration.millis_previous_loop)
    {
        // Add the remainder until wrap-around for previous loop
        delta_millis += (uint32_t)-1 - configuration.millis_previous_loop;

        // Add this loop's millis
        delta_millis += this_millis;
    }
    else
    {
        delta_millis = this_millis - configuration.millis_previous_loop;
    }
    configuration.millis_previous_loop = this_millis;

    return delta_millis;
}

void tick(uint16_t delta_millis)
{
    // Nothing to do for zero delta
    if(!delta_millis)
    {
        return;
    }

    //Serial.print(".");

    panicTimerCountdown();

    doAccelerometerActions(delta_millis, 0);

    doMagnetometerActions(delta_millis, 0);

    doBarometerActions(delta_millis, 0);

}

void loop()
{

    uint8_t i = 0;
    uint16_t delta_millis = 0;
    delta_millis = getDeltaMillis();
    
    tick(delta_millis);

    grabSerial();

    i = parseCommand();

    if(i > 0)
    {
        stripCommand(i);
    }

    stateMachine();

    delay(10);
}
