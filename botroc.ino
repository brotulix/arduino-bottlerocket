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

#define COMMAND_LINE_BUFFER_SIZE    32

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

byte sensorStatus = 0x0;

char cmdline[COMMAND_LINE_BUFFER_SIZE] = { 0 };
uint8_t cmdlength = 0;

volatile byte stateMachineState = STATE_GROUND_IDLE;

// Set a default sea level (= 0m ASL) pressure
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;



uint8_t parseCommandConfiguration(uint8_t j)
{
    switch(cmdline[j++])
    {
        case 'A': // Averaging
        {
            switch(cmdline[j++])
            {
                case 'A': // Accelerometer
                {
                    break;
                }

                case 'B': // Barometer
                {
                    break;
                }

                case 'M': // Magnetometer
                {
                    break;
                }

                default:
                    break;
            }
            break;
        }
        
        case 'H': // Launch pad height
        {
            break;
        }

        case 'L': // Limits
        {
            switch(cmdline[j++])
            {
                case 'A': // Accelerometer
                {
                    switch(cmdline[j++])
                    {
                        case 'I': // Inbound
                        {
                            break;
                        }
                        
                        case 'O': // Outbound
                        {
                            break;
                        }

                        default:
                            break;

                    }
                    break;
                }

                case 'B': // Barometer
                {
                    break;
                }

                case 'M': // Magnetometer
                {
                    break;
                }

                default:
                    break;
            }
            break;
        }
        
        case 'T': // Timeouts
        {
            switch(cmdline[j++])
            {
                case 'I': // Inbound
                {
                    break;
                }

                case 'O': // Outbound
                {
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

    if(i >= COMMAND_LINE_BUFFER_SIZE)
    {
        // This is an error, but we "solve" it by clearing the whole buffer.
        i = COMMAND_LINE_BUFFER_SIZE - 1;
    }

    // Shift the rest of the command buffer to the left
    for(j = 0; j < (COMMAND_LINE_BUFFER_SIZE - (1 + i)); j++)
    {
        cmdline[j] = cmdline[j + 1];
    }
    
    // Clear the rest of the command buffer
    for(; j < COMMAND_LINE_BUFFER_SIZE; j++)
    {
        cmdline[j] = '\0';
    }
    
    // Decrement length counter
    if(cmdlength < 1 + i)
    {
        cmdlength = 0;
    }
    else
    {
        cmdlength -= 1 + i;
    }
}



void parseCommand(void)
{
    uint8_t i = 0;
    uint8_t j = 0;
    
    for(i = 0; i < cmdlength; i++)
    {
        // Check for a complete command
        if(cmdline[i] == '\n')
        {
            if(i > 1)
            {
                // It can theoretically be a command at only 2 bytes ("P\n")
                switch(cmdline[j++])
                {
                    case 'C': // Configuration
                    {
                        if(
                            (stateMachineState == STATE_GROUND_IDLE)
                            ||
                            (stateMachineState == STATE_GROUND_IDLE_ON_PAD)
                        )
                        {
                            j = parseCommandConfiguration(j);
                        }
                        else
                        {
                        }
                        break;
                    }

                    case 'P': // Ping
                    {
                        Serial.println("P");
                        break;
                    }
                    
                    case 'S': // State transition
                    {
                        i = cmdline[j++] - 48;
                        if(i >= 8)
                        {
                            // Error!
                            Serial.println("E");
                            break;
                        }
                        stateMachineState = i;
                        break;
                    }
                    
                    default:
                    {
                        Serial.println("_");
                        break;
                    }
                } // switch(cmdline[0])
            } // if(i > 1)

            Serial.print("pC: LF at: ");
            Serial.println(i);
            Serial.print("pC: utilized command length: ");
            Serial.println(j);

            stripCommand(i);

            Serial.print("pC: cmdline now has ");
            Serial.print(cmdlength);
            Serial.println(" byte remaining");

            
        } // if(cmdline[i] == '\n')
    } // for()
} // parseCommand()



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



void stateMachine()
{
    switch(stateMachineState)
    {
        case STATE_GROUND_IDLE_ON_PAD:
        {
            Serial.print("S");
            Serial.println(STATE_GROUND_IDLE_ON_PAD);
            break;
        }
        
        case STATE_GROUND_ARMED:
        {
            Serial.print("S");
            Serial.println(STATE_GROUND_ARMED);
            break;
        }
        
        case STATE_AIRBORNE_OUTBOUND:
        {
            Serial.print("S");
            Serial.println(STATE_AIRBORNE_OUTBOUND);
            break;
        }
        
        case STATE_AIRBORNE_DEPLOYMENT:
        {
            Serial.print("S");
            Serial.println(STATE_AIRBORNE_DEPLOYMENT);

            stateMachineState = STATE_AIRBORNE_INBOUND;

            break;
        }
        
        case STATE_AIRBORNE_INBOUND:
        {
            Serial.print("S");
            Serial.println(STATE_AIRBORNE_INBOUND);
            break;
        }
        
        case STATE_GROUND_RECOVERY:
        {
            Serial.print("S");
            Serial.println(STATE_GROUND_RECOVERY);
            break;
        }
        
        case STATE_GROUND_DATADUMP:
        {
            Serial.print("S");
            Serial.println(STATE_GROUND_DATADUMP);
            break;
        }

        default:
        case STATE_GROUND_IDLE:
        {
            Serial.print("S");
            Serial.println(STATE_GROUND_IDLE);
            break;
        }
    }
}



void grabSerial()
{
    uint8_t cmdlen = 0;
    uint8_t i = 0;
    
    // Check for commands
    cmdlen = Serial.available();
    if(cmdlen > 0)
    {
        Serial.print(cmdlen);
        Serial.println(" bytes command line");
        
        if(cmdlength + cmdlen >= COMMAND_LINE_BUFFER_SIZE)
        {
            Serial.println("UART buffer read would overshoot command buffer length.");
            cmdlen = COMMAND_LINE_BUFFER_SIZE - cmdlength;
        }

        i = Serial.readBytes((char*)(&cmdline + cmdlength), cmdlen);
        
        if(i < cmdlen)
        {
            Serial.println("Only read ");
            Serial.print(i);
            Serial.println(" bytes!");
        }

        cmdlength += i;

    }

}


void setup()
{
    Serial.begin(115200);
    Serial.println("setup() begins");

    Serial.print("UART RX Buffer size: ");
    Serial.println(SERIAL_RX_BUFFER_SIZE);

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
    
    grabSerial();

    parseCommand();

    stateMachine();

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
    
    delay(1000);
}
