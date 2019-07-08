// If using eg. Visual Studio Code with Arduino plugin, ARDUINO may not be set:
#ifndef ARDUINO
#define ARDUINO 189
#endif

#include <Wire.h>
#include <Adafruit_Sensor.h>        // *modified* Adafruit Unified Sensor library
#include <Adafruit_BMP085_U.h>      // BMP085 Barometer
#include <Adafruit_ADXL345_U.h>     // ADXL345 Accelerometer
#include <Adafruit_HMC5883_U.h>     // HMC5883 Magnetometer
#include <Servo.h>

#define SET_BIT(a, b)   (a |= (0x1 << b))
#define CLEAR_BIT(a, b) (a &= ~(0x1 << b))
#define GET_BIT(a, b)   (a & (0x1 << b))

#define INTSRC_INTERRUPT                        2
#define INTSRC_GYROMETER                        -1
#define INTSRC_BAROMETER                        4
#define INTSRC_THERMOMETER                      5
#define INTSRC_ACCELEROMETER                    6
#define INTSRC_MAGNETOMETER                     7
#define PARACHUTE_SERVO_PIN                     9
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
#define STATE_MAX_STATE                         STATE_GROUND_RECOVERY

#define COMMAND_LINE_BUFFER_SIZE                16
#define COMMAND_LINE_APPEND_THRESHOLD           ((COMMAND_LINE_BUFFER_SIZE / 4) * 3)

#define SENSORS_NUM_VALUES                      8
#define SENSORS_BAROMETER_SEALEVELHPA           SENSORS_PRESSURE_SEALEVELHPA

typedef struct {
    float values[SENSORS_NUM_VALUES];
    float prev_average;
    float average;
    uint8_t filled_elements;
    uint8_t next_element_to_fill;
    uint16_t spare;
} savg;

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
    uint16_t limit_delta_magnetometer;
    uint16_t time_to_next_update_accelerometer;
    uint16_t time_to_next_update_magnetometer;
    uint16_t time_to_next_update_barometer;
    uint16_t time_to_next_update_average_accelerometer;
    uint16_t time_to_next_update_average_magnetometer;
    uint16_t time_to_next_update_average_barometer;
    uint16_t time_to_next_report_accelerometer;
    uint16_t time_to_next_report_magnetometer;
    uint16_t time_to_next_report_barometer;
    uint16_t panic_timeout_outbound;
    uint16_t panic_timeout_inbound;
    uint16_t buzzer_frequency;
    uint8_t servo_safe;
    uint8_t servo_release;
} sconfig;

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(24680);
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
Adafruit_ADXL345_Unified acc = Adafruit_ADXL345_Unified(13579);

Servo parachuteReleaseServo;

char cmdline[COMMAND_LINE_BUFFER_SIZE] = { 0 };
uint8_t cmdlength = 0;

sconfig configuration = {
    0,      // millis_previous_loop
    1,     // interval_refresh_accelerometer
    50,     // interval_refresh_magnetometer
    25,     // interval_refresh_barometer
    15,     // interval_refresh_average_accelerometer
    500,    // interval_refresh_average_magnetometer
    100,    // interval_refresh_average_barometer
    50,     // interval_report_accelerometer
    500,    // interval_report_magnetometer
    100,    // interval_report_barometer
    30,     // limit_delta_barometer
    250,    // limit_delta_accelerometer
    175,    // limit_delta_magnetometer
    0,      // time_to_next_update_accelerometer
    0,      // time_to_next_update_magnetometer
    0,      // time_to_next_update_barometer
    0,      // time_to_next_update_average_accelerometer
    0,      // time_to_next_update_average_magnetometer
    0,      // time_to_next_update_average_barometer
    0,      // time_to_next_report_accelerometer
    0,      // time_to_next_report_magnetometer
    0,      // time_to_next_report_barometer
    3500,   // panic_timeout_outbound
    3000,   // panic_timeout_inbound
    440,    // buzzer_frequency
    45,     // servo_safe
    0       // servo_release
};

savg valsBarometer         = {};
savg valsAccelerometer     = {};
savg valsMagnetometer      = {};

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
int32_t panicTimer = -1;

uint8_t append32(savg *vals, float value)
{
    vals->values[vals->next_element_to_fill] = value;
    
    vals->next_element_to_fill++;
    if(vals->next_element_to_fill >= SENSORS_NUM_VALUES)
    {
        vals->next_element_to_fill -= SENSORS_NUM_VALUES;
    }
    
    if(vals->filled_elements < SENSORS_NUM_VALUES)
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

float favg32(savg *vals)
{
    uint8_t i = 0;
    float sum = 0;
    float avg = 0;

    // Catch the first call.
    if(vals->filled_elements == 0)
    {
        return 0;
    }

    for(i = 0; i < vals->filled_elements; i++)
    {
        sum += vals->values[i];
    }

    avg = sum / vals->filled_elements;

    vals->prev_average = vals->average;
    vals->average = avg;

    return avg;

}

float sensorBarometerReadValue(uint8_t report)
{
    sensors_event_t event;

    bmp.getEvent(&event);

    if(event.pressure)
    {
        if(report)
        {
            Serial.print("B: ");
            Serial.println(event.pressure);
        }
        return event.pressure;
    }
    else
    {
        if(report)
        {
            Serial.print("ERROR");
        }
        return 0.0;
    }
}

float sensorAccelerometerReadValue(uint8_t report)
{
    sensors_event_t event;

    acc.getEvent(&event);

    if(event.acceleration.x || event.acceleration.y || event.acceleration.z)
    {
        if(report)
        {
            Serial.print("A: ");
            Serial.print("[");
            Serial.print(event.acceleration.x);
            Serial.print(", ");
            Serial.print(event.acceleration.y);
            Serial.print(",  ");
            Serial.print(event.acceleration.z);
            Serial.println("]");
        }
        return vlen3D(event.acceleration.x, event.acceleration.y, event.acceleration.z);
    }
    else
    {
        if(report)
        {
            Serial.print("ERROR");
        }
        return 0.0;
    }
}

float sensorMagnetometerReadValue(uint8_t report)
{
    sensors_event_t event; 
    mag.getEvent(&event);

    if(event.magnetic.x || event.magnetic.y || event.magnetic.z)
    {
        if(report)
        {
            Serial.print("M: ");
            Serial.print("[");
            Serial.print(event.magnetic.x);
            Serial.print(", ");
            Serial.print(event.magnetic.y);
            Serial.print(",  ");
            Serial.print(event.magnetic.z);
            Serial.println("]");
        }
        return vlen3D(event.magnetic.x, event.magnetic.y, event.magnetic.z);
    }
    else
    {
        if(report)
        {
            Serial.print("ERROR");
        }
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
                
                Serial.print("Current Accelerometer value: ");
                Serial.println(sensorAccelerometerReadValue(1));

                Serial.print("Current Barometer value: ");
                Serial.println(sensorBarometerReadValue(1));

                Serial.print("Current Magnetometer value: ");
                Serial.println(sensorMagnetometerReadValue(1));

                doBarometerActions(0, ((1 << STATUS_REPORT)|(1 << STATUS_UPDATE)|(1 << STATUS_UPDATE_AVERAGE)), 1);
                
                doAccelerometerActions(0, ((1 << STATUS_REPORT)|(1 << STATUS_UPDATE)|(1 << STATUS_UPDATE_AVERAGE)), 1);
                
                doMagnetometerActions(0, ((1 << STATUS_REPORT)|(1 << STATUS_UPDATE)|(1 << STATUS_UPDATE_AVERAGE)), 1);

                Serial.print("Current buzzer frequency is ");
                Serial.print(configuration.buzzer_frequency);
                Serial.println(" Hz");

                Serial.print("Peak altitude pressure: ");
                Serial.println(peakAltitudePressure);


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
                if(j > STATE_MAX_STATE)
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
                setState(j);
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
}

void doAccelerometerActions(uint16_t delta_millis, uint8_t force, uint8_t report)
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

    if((report && update) || GET_BIT(force, STATUS_REPORT))
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
        append32(&valsAccelerometer, sensorAccelerometerReadValue(0));
    }
}

void doMagnetometerActions(uint16_t delta_millis, uint8_t force, uint8_t report)
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

    if((report && update) || GET_BIT(force, STATUS_REPORT))
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
        append32(&valsMagnetometer, sensorMagnetometerReadValue(0));
    }
}

void doBarometerActions(uint16_t delta_millis, uint8_t force, uint8_t report)
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

    if((report && update) || GET_BIT(force, STATUS_REPORT))
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
        append32(&valsBarometer, sensorBarometerReadValue(0));
    }
}

void panicTimerCountdown(uint16_t delta)
{
    if(!panicActivate)
    {
        panicTimer -= delta;
        if(panicTimer <= 0)
        {
            panicActivate = 1;
            Serial.println("Panic timeout!");
        }
    }
}

void panicTimerReset()
{
    switch(stateMachineState)
    {
        case STATE_AIRBORNE_INBOUND:
        {
            panicTimer = configuration.panic_timeout_inbound;
            break;
        }
        case STATE_AIRBORNE_OUTBOUND:
        {
            panicTimer = configuration.panic_timeout_outbound;
            break;
        }
        default:
        {
            panicTimer = 0;
        }
    }
    panicActivate = 0;
}

void releaseTheKrakenshoot()
{
    // Release the parachute by applying a different angle to the servo
    // or applying power to a motor
    // or something...
    parachuteReleaseServo.write(configuration.servo_release);
    delay(250);
}

void resetParachute()
{
    parachuteReleaseServo.write(configuration.servo_safe);
    delay(250);
}


void engageAudioVisualBeacon()
{
    // Toggle power to audiovisual beacon
}


void setState(uint8_t newState)
{
    uint8_t set = STATE_GROUND_IDLE;
    
    if(newState <= STATE_MAX_STATE)
    {
        set = newState;
    }

    Serial.print(millis());
    Serial.print(": S");
    Serial.print(stateMachineState);
    Serial.print(" -> S");
    Serial.println(set);

    stateMachineState = set;
    
    switch(stateMachineState)
    {
        default:
        case STATE_GROUND_IDLE:
        {
            panicTimerReset();
            break;
        }
        case STATE_GROUND_IDLE_ON_PAD:
        {
            peakAltitudePressure = valsBarometer.average;
            panicTimerReset();
            resetParachute();
            break;
        }
        case STATE_GROUND_ARMED:
        {
            panicTimerReset();
            break;
        }
        case STATE_AIRBORNE_OUTBOUND:
        {
            barometerdeltastrikes = SENSORS_BAROMETER_STRIKES;
            panicTimerReset();
            break;
        }
        case STATE_AIRBORNE_DEPLOYMENT:
        {
            break;
        }
        case STATE_AIRBORNE_INBOUND:
        {
            resetParachute();
            panicTimerReset();
            break;
        }
        case STATE_GROUND_RECOVERY:
        {
            panicTimerReset();
            break;
        }
        case STATE_GROUND_DATADUMP:
        {
            break;
        }

    }
}

void stateMachine(uint16_t delta_millis)
{
    int32_t diff = 0;
    switch(stateMachineState)
    {
        case STATE_GROUND_IDLE_ON_PAD:
        {
            // Make sure we have something in the arrays for small initial magnitude deltas
            if(valsAccelerometer.filled_elements < SENSORS_NUM_VALUES)
            {
                for(diff = 0; diff < SENSORS_NUM_VALUES; diff++)
                {
                    doBarometerActions(0, ((1 << STATUS_UPDATE)|(1 << STATUS_UPDATE_AVERAGE)), 0);
                    
                    doAccelerometerActions(0, ((1 << STATUS_UPDATE)|(1 << STATUS_UPDATE_AVERAGE)), 0);
                    
                    doMagnetometerActions(0, ((1 << STATUS_UPDATE)|(1 << STATUS_UPDATE_AVERAGE)), 0);

                }
            }

            doAccelerometerActions(delta_millis, 0, 0);

            // Keep watch on arming trigger
            doMagnetometerActions(delta_millis, 0, 1);

            // Keep watch on barometer, probably not the fastest changes, but we don't want a high magnitude just as we arm...
            doBarometerActions(delta_millis, 0, 0);


            // Magnetometer
            if(valsMagnetometer.prev_average > valsMagnetometer.average)
            {
                diff = valsMagnetometer.prev_average - valsMagnetometer.average;
            }
            else
            {
                diff = valsMagnetometer.average - valsMagnetometer.prev_average;
            }
            
            if(diff > configuration.limit_delta_magnetometer)
            {
                Serial.print(millis());
                Serial.println(": MAGNETOMETER MAGNITUDE ABOVE THRESHOLD");
                setState(STATE_GROUND_ARMED);
            }
            break;
        }
        
        case STATE_GROUND_ARMED:
        {
            // Check the two possible launch-indicative magnutides
            doAccelerometerActions(delta_millis, 0, 1);
            doBarometerActions(delta_millis, 0, 1);
            doMagnetometerActions(delta_millis, 0, 0);

            // Accelerometer
            diff = abs(valsAccelerometer.prev_average - valsAccelerometer.average);
            
            if(diff > configuration.limit_delta_accelerometer)
            {
                Serial.print(millis());
                Serial.println(": ACCELEROMETER MAGNITUDE ABOVE THRESHOLD");
                setState(STATE_AIRBORNE_OUTBOUND);
            }

            // Barometer
            diff = abs((valsBarometer.average - valsBarometer.prev_average)*100.0);
            
            if(diff > configuration.limit_delta_barometer)
            {
                Serial.print(millis());
                Serial.println(": BAROMETER MAGNITUDE ABOVE THRESHOLD");
                setState(STATE_AIRBORNE_OUTBOUND);
            }

            break;
        }
        
        case STATE_AIRBORNE_OUTBOUND:
        {
            panicTimerCountdown(delta_millis);

            doAccelerometerActions(delta_millis, 0, 1);
            doBarometerActions(delta_millis, 0, 1);
            doMagnetometerActions(delta_millis, 0, 0);

            // Check for lowest pressure yet
            if(valsBarometer.average < peakAltitudePressure)
            {
                peakAltitudePressure = valsBarometer.average;
            }

            diff = 0;
            
            // count down positive barometer deltas, indicating a decreasing height
            // A single positive value could perhaps simply be from pressure oscillations inside the payload capsule due to turbulence during launch, so we could get a premature deployment of parachute?
            if(valsBarometer.prev_average < valsBarometer.average)
            {
                // Decreasing
                diff = ((valsBarometer.average - valsBarometer.prev_average)*100.0);

                // Decreasing by enough?
                if(diff > configuration.limit_delta_barometer)
                {
                    Serial.print(millis());
                    Serial.print(": BAROMETER MAGNITUDE ABOVE THRESHOLD (");
                    Serial.print(diff);
                    Serial.print(">");
                    Serial.print(configuration.limit_delta_barometer);
                    Serial.println(")");

                    barometerdeltastrikes--;
                }
            }
            
            if(barometerdeltastrikes == 0)
            {
                Serial.print(millis());
                Serial.println(": THREE STRIKES; DEPLOYING");
                setState(STATE_AIRBORNE_DEPLOYMENT);
            }

            if(panicActivate)
            {
                Serial.print(millis());
                Serial.println(": DEPLOYING DUE TO PANIC TIMEOUT");
                setState(STATE_AIRBORNE_DEPLOYMENT);
            }

            break;
        }
        
        case STATE_AIRBORNE_DEPLOYMENT:
        {
            releaseTheKrakenshoot();
            setState(STATE_AIRBORNE_INBOUND);
            break;
        }
        
        case STATE_AIRBORNE_INBOUND:
        {
            panicTimerCountdown(delta_millis);

            doAccelerometerActions(delta_millis, 0, 1);
            doBarometerActions(delta_millis, 0, 1);
            doMagnetometerActions(delta_millis, 0, 0);

            if(panicActivate)
            {
                Serial.print(millis());
                Serial.println(": Activating locator beacon due to panic timeout.");
                setState(STATE_GROUND_RECOVERY);
            }
            break;
        }
        
        case STATE_GROUND_RECOVERY:
        {
            // Wait for RBF to be re-applied
            doAccelerometerActions(delta_millis, 0, 0);
            doBarometerActions(delta_millis, 0, 0);
            doMagnetometerActions(delta_millis, 0, 1);
            
            // Magnetometer
            diff = abs(valsMagnetometer.average - valsMagnetometer.prev_average);
            
            if(diff > configuration.limit_delta_magnetometer)
            {
                Serial.println("MAGNETOMETER MAGNITUDE ABOVE THRESHOLD");
                setState(STATE_GROUND_IDLE);
                //setState(STATE_GROUND_DATADUMP);
            }
            
            if(configuration.buzzer_frequency > 8000)
            {
                configuration.buzzer_frequency = 440;
            }
            
            tone(8, configuration.buzzer_frequency, 50);
            
            configuration.buzzer_frequency += 10;
            
            break;
        }
        
        case STATE_GROUND_DATADUMP:
        {
            // Not really doing anything
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
    int16_t servoPosition = configuration.servo_safe;
    Serial.begin(115200);
    Serial.println("setup() begins");

    delay(1000);

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

    delay(1000);

    //Serial.println("Initializing parachute release servo...");
    
    parachuteReleaseServo.attach(PARACHUTE_SERVO_PIN);

    //Serial.println("Safe position...");

    parachuteReleaseServo.write(configuration.servo_safe);

    delay(3000);

    //Serial.println("Traversing...");
    //
    //for (servoPosition = 180; servoPosition >= -180; servoPosition -= 3)
    //{
    //    Serial.println(servoPosition);
    //    parachuteReleaseServo.write(servoPosition);
    //    delay(500);
    //}
    //
    //delay(3000);
    //
    //Serial.println("Return to safe...");
    //
    //parachuteReleaseServo.write(configuration.servo_safe);

    Serial.println("Initializing sensors...");
    
    //Serial.println("Barometer...");
    SET_BIT(sensorStatus, STATUS_BAROMETER_DETECTED);
    if (!bmp.begin()) {
        Serial.println("BE");
        CLEAR_BIT(sensorStatus, STATUS_BAROMETER_DETECTED);
    }

    //Serial.println("Magnetometer...");
    SET_BIT(sensorStatus, STATUS_MAGNETOMETER_DETECTED);
    if(!mag.begin())
    {
        Serial.println("ME");
        CLEAR_BIT(sensorStatus, STATUS_MAGNETOMETER_DETECTED);
    }

    //Serial.println("Accelerometer...");
    SET_BIT(sensorStatus, STATUS_ACCELEROMETER_DETECTED);
    if(!acc.begin())
    {
        Serial.println("AE");
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
        delta_millis = (uint32_t)-1 - configuration.millis_previous_loop;

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



void loop()
{

    uint8_t i = 0;
    uint16_t delta_millis = 0;
    delta_millis = getDeltaMillis();
    
    grabSerial();

    i = parseCommand();

    if(i > 0)
    {
        stripCommand(i);
    }

    stateMachine(delta_millis);

    delay(10);
}
