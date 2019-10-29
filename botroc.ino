// If using eg. Visual Studio Code with Arduino plugin, ARDUINO may not be set:
#ifndef ARDUINO
#define ARDUINO 189
#endif

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <MPU9250_asukiaaa.h>
#include <Servo.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "functions.h"
#include "messages.h"

#define SET_BIT(a, b)   (a |= (0x1 << b))
#define CLEAR_BIT(a, b) (a &= ~(0x1 << b))
#define GET_BIT(a, b)   (a & (0x1 << b))



/*
#define INTSRC_INTERRUPT                        2
#define INTSRC_GYROMETER                        -1
#define INTSRC_BAROMETER                        4
#define INTSRC_THERMOMETER                      5
#define INTSRC_ACCELEROMETER                    6
#define INTSRC_MAGNETOMETER                     7
*/
#define PIN_RADIO_A                             5
#define PIN_RADIO_B                             6
#define PIN_SERVO_ENABLE                        7
#define PIN_BUZZER                              8
#define PIN_PARACHUTE_SERVO                     9
#define PIN_BLINKER                             10
#define I2C_SDA                                 18
#define I2C_SCL                                 19
#define PIN_BATTERY_VOLTAGE                     PIN_A2

#define STATUS_BAROMETER_DETECTED               0
#define STATUS_ACCELEROMETER_DETECTED           1
#define STATUS_GYROMETER_DETECTED               2
#define STATUS_MAGNETOMETER_DETECTED            3

#define STATUS_UPDATE                           0
#define STATUS_UPDATE_AVERAGE                   1
#define STATUS_REPORT                           2

#define AVERAGING_FACTOR                        0.05    // = 1/20
#define AVERAGING_COUNT                         20      // = 1/0.05

#define SENSORS_NUM_VALUES                      8

typedef struct
{
    uint16_t interval_read_accelerometer;
    uint16_t interval_read_barometer;
    uint16_t interval_read_magnetometer;
    uint16_t interval_read_battery;
    uint16_t limit_delta_accelerometer;
    uint16_t limit_delta_barometer;
    uint16_t limit_delta_magnetometer;
    uint16_t limit_battery_voltage;
    uint16_t panic_timeout_outbound;
    uint16_t panic_timeout_inbound;
    uint16_t buzzer_frequency;
    uint8_t servo_safe;
    uint8_t servo_release;
    uint16_t servo_hold;
} t_configurables;

typedef struct
{
    uint32_t millis_previous_loop;
    uint16_t time_to_next_update_accelerometer;
    uint16_t time_to_next_update_barometer;
    uint16_t time_to_next_update_magnetometer;
    uint16_t time_to_next_read_battery_voltage;
    uint16_t time_to_servo_stop_hold;
} t_dynamic_config;


typedef struct {
    t_dynamic_config dynamic;
    t_configurables configurable;
} t_config;

Adafruit_BMP280 bmp;
MPU9250_asukiaaa mpu;

Servo parachuteReleaseServo;

uint8_t cmdline[COMMAND_LINE_BUFFER_SIZE] = { 0 };

t_config configuration = { 0 };

t_msg_barometer     valsBarometer       = { 0 };
t_msg_accelerometer valsAccelerometer   = { 0 };
t_msg_magnetometer  valsMagnetometer    = { 0 };
t_msg_debug         valsBattery         = { 0 };

uint8_t stateMachineState = STATE_GROUND_IDLE;

//float peakAltitudePressure = -1.0;
//float groundStationPressure = -1.0;
float unarmedMagnetometerMagnitude = -1.0;
uint8_t unarmedMagnetometerReadings = AVERAGING_COUNT; // decrease to 0 before evaluating for arming
//float groundStationAccelerationMagnitude = -1.0;
//float peakLaunchAccelerationMagnitude = -1.0;

// How many consequitive positive barometer deltas to read in a row to say we're
//   steadily falling
#define SENSORS_BAROMETER_STRIKES   3
uint8_t barometerdeltastrikes = 3;

uint8_t panicActivate = 0;
int32_t panicTimer = -1;

RF24 radio(PIN_RADIO_A, PIN_RADIO_B);
const uint64_t pipes[2] = {
    0xABCDABCD71LL,         // Ground-to-air pipe (Read)
    0x544d52687CLL          // Air-to-ground pipe (Write)
};

void setDefaultConfig()
{
    // Configurable part of configuration
    configuration.configurable.interval_read_accelerometer          =     2;
    configuration.configurable.interval_read_barometer              =    50;
    configuration.configurable.interval_read_magnetometer           =    50;
    configuration.configurable.interval_read_battery                = 30000; // Battery only needs checking every 30 seconds.
    configuration.configurable.limit_delta_accelerometer            =    20;
    configuration.configurable.limit_delta_barometer                =    10;
    configuration.configurable.limit_delta_magnetometer             =    60;
    configuration.configurable.limit_battery_voltage                =   730;
    configuration.configurable.panic_timeout_outbound               =   300;
    configuration.configurable.panic_timeout_inbound                =   150;
    configuration.configurable.buzzer_frequency                     =   400;
    configuration.configurable.servo_safe                           =    45;
    configuration.configurable.servo_release                        =     0;
    configuration.configurable.servo_hold                           =   250;
    
    // Dynamic part of configuration
    configuration.dynamic.millis_previous_loop                      =     0,
    configuration.dynamic.time_to_next_update_accelerometer         =     0;
    configuration.dynamic.time_to_next_update_barometer             =     0;
    configuration.dynamic.time_to_next_update_magnetometer          =     0;
    configuration.dynamic.time_to_next_read_battery_voltage         =     0;
    configuration.dynamic.time_to_servo_stop_hold                   =     0;
}

void clearCommandBuffer()
{
    uint8_t cmdlength = 0;
    //Serial.println("Clearing command buffer...");
    for(cmdlength = COMMAND_LINE_BUFFER_SIZE; cmdlength > 0; cmdlength--)
    {
        cmdline[cmdlength-1] = '\0';
    }
}


bool sensorBarometerReadValue()
{
    valsBarometer.abs_pressure = bmp.readPressure();
    valsBarometer.temperature = bmp.readTemperature();
    return true;
}

bool sensorAccelerometerReadValue()
{
    if (mpu.accelUpdate() == 0)
    {
        valsAccelerometer.vector.x = mpu.accelX();
        valsAccelerometer.vector.y = mpu.accelY();
        valsAccelerometer.vector.z = mpu.accelZ();
        valsAccelerometer.vector.magnitude = mpu.accelSqrt();
        return true;
    }
    else
    {
        return false;
    }
}

bool sensorMagnetometerReadValue()
{
    if (mpu.magUpdate() == 0)
    {
        valsMagnetometer.vector.x = mpu.magX();
        valsMagnetometer.vector.y = mpu.magY();
        valsMagnetometer.vector.z = mpu.magZ();
        valsMagnetometer.vector.magnitude = vlen3D(valsMagnetometer.vector.x, valsMagnetometer.vector.y, valsMagnetometer.vector.z);
        valsMagnetometer.heading = (uint16_t)((mpu.magHorizDirection() * 100) + 0.5); // +0.5 for rounding of float to uint conversion instead of cropping
        return true;
    }
    else
    {
        return false;
    }
}

void sendError(uint32_t code)
{
    t_msg_debug* msg = (t_msg_debug*)&cmdline[0];
    clearMessage();

    // Send an error telegram with error code

    msg->header.id_full = CMD_ERROR;
    msg->header.millis = millis();
    msg->header.state = stateMachineState;
    msg->payload.code = code;
    msg->payload.value = 0;
    //RF.sendsendMsg(&cmdline[0], sizeof(t_msg_debug));


}

void reportConfig(uint8_t *message, uint8_t len)
{
    t_msg_debug *msg = (t_msg_debug*)message;
    t_msg_debug *buffer = (t_msg_debug*)cmdline[0];
    uint8_t outlen = sizeof(t_msg_config);
    
    clearMessage();
    
    buffer->header.millis = millis();
    buffer->header.id_full = (CMD_CONFIG << 8) & stateMachineState;
    
    switch(msg->payload.value)
    {
        case CONFIG_INTERVAL_READ_ACCELEROMETER:
        {
            buffer->payload.code = CONFIG_INTERVAL_READ_ACCELEROMETER;
            buffer->payload.value = configuration.configurable.interval_read_accelerometer;
            break;
        }
        
        case CONFIG_INTERVAL_READ_BAROMETER:
        {
            buffer->payload.code = CONFIG_INTERVAL_READ_BAROMETER;
            buffer->payload.value = configuration.configurable.interval_read_barometer;
            break;
        }
        
        case CONFIG_INTERVAL_READ_MAGNETOMETER:
        {
            buffer->payload.code = CONFIG_INTERVAL_READ_MAGNETOMETER;
            buffer->payload.value = configuration.configurable.interval_read_magnetometer;
            break;
        }
        
        case CONFIG_INTERVAL_READ_BATTERY:
        {
            buffer->payload.code = CONFIG_INTERVAL_READ_BATTERY;
            buffer->payload.value = configuration.configurable.interval_read_battery;
            break;
        }
        
        case CONFIG_LIMIT_DELTA_ACCELEROMETER:
        {
            buffer->payload.code = CONFIG_LIMIT_DELTA_ACCELEROMETER;
            buffer->payload.value = configuration.configurable.limit_delta_accelerometer;
            break;
        }
        
        case CONFIG_LIMIT_DELTA_BAROMETER:
        {
            buffer->payload.code = CONFIG_LIMIT_DELTA_BAROMETER;
            buffer->payload.value = configuration.configurable.limit_delta_barometer;
            break;
        }
        
        case CONFIG_LIMIT_DELTA_MAGNETOMETER:
        {
            buffer->payload.code = CONFIG_LIMIT_DELTA_MAGNETOMETER;
            buffer->payload.value = configuration.configurable.limit_delta_magnetometer;
            break;
        }
        
        case CONFIG_LIMIT_BATTERY_VOLTAGE:
        {
            buffer->payload.code = CONFIG_LIMIT_BATTERY_VOLTAGE;
            buffer->payload.value = configuration.configurable.limit_battery_voltage;
            break;
        }
        
        case CONFIG_TIMEOUT_PANIC_OUTBOUND:
        {
            buffer->payload.code = CONFIG_TIMEOUT_PANIC_OUTBOUND;
            buffer->payload.value = configuration.configurable.panic_timeout_outbound;
            break;
        }
        
        case CONFIG_TIMEOUT_PANIC_INBOUND:
        {
            buffer->payload.code = CONFIG_TIMEOUT_PANIC_INBOUND;
            buffer->payload.value = configuration.configurable.panic_timeout_inbound;
            break;
        }
        
        case CONFIG_BUZZER_FREQUENCY:
        {
            buffer->payload.code = CONFIG_BUZZER_FREQUENCY;
            buffer->payload.value = configuration.configurable.buzzer_frequency;
            break;
        }
        
        case CONFIG_SERVO_SAFE:
        {
            buffer->payload.code = CONFIG_SERVO_SAFE;
            buffer->payload.value = configuration.configurable.servo_safe;
            break;
        }
        
        case CONFIG_SERVO_RELEASE:
        {
            buffer->payload.code = CONFIG_SERVO_RELEASE;
            buffer->payload.value = configuration.configurable.servo_release;
            break;
        }
        
        case CONFIG_SERVO_HOLD:
        {
            buffer->payload.code = CONFIG_SERVO_HOLD;
            buffer->payload.value = configuration.configurable.servo_hold;
            break;
        }

        default:
        {
            Serial.print("reportConfig(): Invalid config code (");
            Serial.print(msg->payload.value);
            Serial.println(")");
            sendError(ERROR_INVALID_CONFIG);
            break;
        }
    }

    sendMsg((uint8_t *)buffer, outlen);
}

void reportSensor(uint8_t sensor)
{
    uint8_t *buffer = NULL;
    uint8_t len = 0;
    
    switch (sensor)
    {
        // Accelerometer
        case SENSOR_TYPE_ACCELEROMETER:
        {
            buffer = (uint8_t*)&valsAccelerometer;
            len = sizeof(valsAccelerometer);
            break;
        }

        // Barometer (incl. temperature!)
        case SENSOR_TYPE_PRESSURE:
        {
            buffer = (uint8_t*)&valsBarometer;
            len = sizeof(valsBarometer);
            break;
        }

        // Magnetometer
        case SENSOR_TYPE_MAGNETIC_FIELD:
        {
            buffer = (uint8_t*)&valsMagnetometer;
            len = sizeof(valsMagnetometer);
            break;
        }

        // Voltage
        case SENSOR_TYPE_VOLTAGE:
        {
            buffer = (uint8_t*)&valsBattery;
            len = sizeof(valsBattery);
            break;
        }

        default:
        {
            sendError(ERROR_INVALID_COMMAND);
            return;
            break;
        }
    }

    sendMsg(buffer, len);
}

void sendMsg(uint8_t *buffer, uint8_t len)
{
    radio.stopListening();
    if (!radio.write(buffer, len))
    {
        sendError(ERROR_SEND_FAILED);
    }

    // Rf.write(buffer, len);
}

void reportState()
{
    t_msg_debug* msg = (t_msg_debug*)&cmdline[0];
    clearMessage();

    // Send CMD_STATE with current state

    msg->header.id_full = CMD_STATE;
    msg->header.millis = millis();
    msg->header.state = stateMachineState;
    msg->payload.code = STATE_REPORT;
    msg->payload.value = stateMachineState;
    //rf.sendMsg(&cmdline[0], sizeof(t_msg_debug));
}

void parseCommandConfiguration(uint8_t* message, uint8_t len)
{
    if(len < sizeof(t_msg_debug))
    {
        sendError(ERROR_MESSAGE_LENGTH);
        return;
    }

    t_msg_debug *msg = (t_msg_debug*)message;
    
    switch(msg->payload.code)
    {
        case CONFIG_REPORT:
        {
            reportConfig(message, len);
            break;
        }

        case CONFIG_INTERVAL_READ_ACCELEROMETER:
        {
            configuration.configurable.interval_read_accelerometer = (uint16_t)msg->payload.value;
            break;
        }

        case CONFIG_INTERVAL_READ_BAROMETER:
        {
            configuration.configurable.interval_read_barometer = (uint16_t)msg->payload.value;
            break;
        }

        case CONFIG_INTERVAL_READ_MAGNETOMETER:
        {
            configuration.configurable.interval_read_magnetometer = (uint16_t)msg->payload.value;
            break;
        }

        case CONFIG_INTERVAL_READ_BATTERY:
        {
            configuration.configurable.interval_read_battery = (uint16_t)msg->payload.value;
            break;
        }

        case CONFIG_LIMIT_DELTA_ACCELEROMETER:
        {
            configuration.configurable.limit_delta_accelerometer = (uint16_t)msg->payload.value;
            break;
        }

        case CONFIG_LIMIT_DELTA_BAROMETER:
        {
            configuration.configurable.limit_delta_barometer = (uint16_t)msg->payload.value;
            break;
        }

        case CONFIG_LIMIT_DELTA_MAGNETOMETER:
        {
            configuration.configurable.limit_delta_magnetometer = (uint16_t)msg->payload.value;
            break;
        }

        case CONFIG_LIMIT_BATTERY_VOLTAGE:
        {
            configuration.configurable.limit_battery_voltage = (uint16_t)msg->payload.value;
            break;
        }

        case CONFIG_TIMEOUT_PANIC_OUTBOUND:
        {
            configuration.configurable.panic_timeout_outbound = (uint16_t)msg->payload.value;
            break;
        }

        case CONFIG_TIMEOUT_PANIC_INBOUND:
        {
            configuration.configurable.panic_timeout_inbound = (uint16_t)msg->payload.value;
            break;
        }
        
        case CONFIG_BUZZER_FREQUENCY:
        {
            configuration.configurable.buzzer_frequency = (uint16_t)msg->payload.value;
            break;
        }

        case CONFIG_SERVO_SAFE:
        {
            configuration.configurable.servo_safe = (uint8_t)msg->payload.value;
            break;
        }

        case CONFIG_SERVO_RELEASE:
        {
            configuration.configurable.servo_release = (uint8_t)msg->payload.value;
            break;
        }

        case CONFIG_SERVO_HOLD:
        {
            configuration.configurable.servo_hold = (uint16_t)msg->payload.value;
            break;
        }

        default:
        {
            sendError(ERROR_INVALID_CONFIG);
            break;
        }
    }
}

void parseCommandState(uint8_t* message, uint8_t len)
{
    if(len < sizeof(t_msg_debug))
    {
        sendError(ERROR_MESSAGE_LENGTH);
        return;
    }

    t_msg_debug *msg = (t_msg_debug*)message;

    switch(msg->payload.code)
    {
        case STATE_REPORT:
        {
            reportState();
            break;
        }

        case STATE_CHANGE:
        {
            setState((uint8_t)msg->payload.value);
            break;
        }

        default:
        {
            sendError(ERROR_INVALID_COMMAND);
            break;
        }
    }
    
}

void clearMessage()
{
    uint8_t i = 0;
    for(i = 0; i < COMMAND_LINE_BUFFER_SIZE; i++)
    {
        cmdline[i] = 0;
    }
}

void pong()
{
    t_msg_header* buffer = (t_msg_header*)&cmdline[0];
    uint8_t len = sizeof(t_msg_debug);
    buffer->millis = millis();
    buffer->id_full = (CMD_PINGPONG << 8) & stateMachineState;
    sendMsg((uint8_t*)buffer, len);
}

bool parseMessage(uint8_t* message, uint8_t len)
{
    bool status = true;
    t_msg_header *msg = (t_msg_header*)&cmdline[0];

    if(len > COMMAND_LINE_BUFFER_SIZE)
    {
        sendError(ERROR_MESSAGE_LENGTH);
    }

    // Check the first three bytes of message for message type
    uint32_t id = (msg->id_full & 0xFFFFFF00) >> 8;

    switch (id)
    {
        case CMD_PINGPONG:
        {
            pong();
            break;
        }
        
        case CMD_STATE:
        {
            // change states
            break;
        }

        case CMD_READ:
        {
            // read accelerometer (and report)
            break;
        }
        

        case CMD_CONFIG:
        {
            parseCommandConfiguration(message, len);
            break;
        }

        default:
        {
            status = false;
            break;
        }
    }

    clearMessage();
    return status;
    
}

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
        sendError(ERROR_DEVICE_NOT_FOUND);
        //Serial.println("E: At least one sensor not detected.");
    }
}

uint8_t doAccelerometerActions(uint16_t delta_millis, uint8_t force)
{
    uint8_t update = 0;
    bool updated = false;

    if(delta_millis >= configuration.dynamic.time_to_next_update_accelerometer)
    {
        SET_BIT(update, STATUS_UPDATE);
        configuration.dynamic.time_to_next_update_accelerometer = configuration.configurable.interval_read_accelerometer;
    }
    else {
        CLEAR_BIT(update, STATUS_UPDATE);
        configuration.dynamic.time_to_next_update_accelerometer -= delta_millis;
    }

    if(GET_BIT(update, STATUS_UPDATE) || GET_BIT(force, STATUS_UPDATE))
    {
        updated = sensorAccelerometerReadValue();
    }

    if(updated)
    {
        reportSensor(SENSOR_TYPE_ACCELEROMETER);
    }

    return (update | force);
}

uint8_t doMagnetometerActions(uint16_t delta_millis, uint8_t force)
{
    uint8_t update = 0;
    bool updated = false;

    if(delta_millis >= configuration.dynamic.time_to_next_update_magnetometer)
    {
        SET_BIT(update, STATUS_UPDATE);
        configuration.dynamic.time_to_next_update_magnetometer = configuration.configurable.interval_read_magnetometer;
    }
    else {
        CLEAR_BIT(update, STATUS_UPDATE);
        configuration.dynamic.time_to_next_update_magnetometer -= delta_millis;
    }

    if(GET_BIT(update, STATUS_UPDATE) || GET_BIT(force, STATUS_UPDATE))
    {
        updated = sensorMagnetometerReadValue();
    }

    if(updated)
    {
        reportSensor(SENSOR_TYPE_MAGNETIC_FIELD);
    }

    return (update | force);
}

uint8_t doBarometerActions(uint16_t delta_millis, uint8_t force)
{
    uint8_t update = 0;
    bool updated = false;

    if(delta_millis >= configuration.dynamic.time_to_next_update_barometer)
    {
        SET_BIT(update, STATUS_UPDATE);
        configuration.dynamic.time_to_next_update_barometer = configuration.configurable.interval_read_barometer;
    }
    else {
        CLEAR_BIT(update, STATUS_UPDATE);
        configuration.dynamic.time_to_next_update_barometer -= delta_millis;
    }

    if(GET_BIT(update, STATUS_UPDATE) || GET_BIT(force, STATUS_UPDATE))
    {
        sensorMagnetometerReadValue();
    }

    if(updated)
    {
        reportSensor(SENSOR_TYPE_PRESSURE);
    }
    
    return (update | force);
}

void doBatteryActions(uint16_t delta_millis, uint8_t force)
{
    uint8_t update = 0;
    uint16_t voltage = 0;
    bool updated = false;
    
    if(delta_millis >= configuration.dynamic.time_to_next_read_battery_voltage)
    {
        SET_BIT(update, STATUS_UPDATE);
        configuration.dynamic.time_to_next_read_battery_voltage = configuration.configurable.interval_read_battery;
    }
    else {
        CLEAR_BIT(update, STATUS_UPDATE);
        configuration.dynamic.time_to_next_read_battery_voltage -= delta_millis;
    }

    if(GET_BIT(update, STATUS_UPDATE) || GET_BIT(force, STATUS_UPDATE))
    {
        updated = true;

        readBatteryVoltage();
        
        if(valsBattery.payload.value < configuration.configurable.limit_battery_voltage)
        {
            sendError(ERROR_LOW_BATTERY_WARNING);
            tone(PIN_BUZZER, 440, configuration.configurable.interval_read_battery/2);
        }
    }

    if(updated)
    {
        reportSensor(SENSOR_TYPE_VOLTAGE);
    }
}

void readBatteryVoltage()
{
    valsBattery.payload.value = (uint16_t)analogRead(PIN_BATTERY_VOLTAGE);
    valsBattery.header.millis = millis();
}

void panicTimerCountdown(uint16_t delta)
{
    if(!panicActivate)
    {
        panicTimer -= delta;
        if(panicTimer <= 0)
        {
            panicActivate = 1;
            sendError(ERROR_PANIC_TIMEOUT);
        }
    }
}

void panicTimerReset()
{
    switch(stateMachineState)
    {
        case STATE_AIRBORNE_INBOUND:
        {
            panicTimer = configuration.configurable.panic_timeout_inbound;
            break;
        }

        case STATE_AIRBORNE_OUTBOUND:
        {
            panicTimer = configuration.configurable.panic_timeout_outbound;
            break;
        }

        default:
        {
            panicTimer = 0;
        }
    }
    panicActivate = 0;
}

uint8_t releaseTheKrakenshoot(uint16_t delta_millis)
{
    if(delta_millis < configuration.dynamic.time_to_servo_stop_hold)
    {
        digitalWrite(PIN_SERVO_ENABLE, HIGH);
        parachuteReleaseServo.write(configuration.configurable.servo_release);
        configuration.dynamic.time_to_servo_stop_hold -= delta_millis;
        return 0;
    }
    else
    {
        digitalWrite(PIN_SERVO_ENABLE, LOW);
        return 1;
    }
}


// Reset the parachute by enabling servo drive FET for a little while and setting the "safe" position. When time is up, disable servo drive FET to conserve that precious juice.
uint8_t resetParachute(uint16_t delta_millis)
{
    if(delta_millis < configuration.dynamic.time_to_servo_stop_hold)
    {
        digitalWrite(PIN_SERVO_ENABLE, HIGH);
        parachuteReleaseServo.write(configuration.configurable.servo_safe);
        configuration.dynamic.time_to_servo_stop_hold -= delta_millis;
        return 0;
    }
    else
    {
        configuration.dynamic.time_to_servo_stop_hold = configuration.configurable.servo_hold;
        digitalWrite(PIN_SERVO_ENABLE, LOW);
        return 1;
    }
}


void enableAudioVisualBeacon()
{
    // Toggle power to audiovisual beacon
    digitalWrite(PIN_BLINKER, HIGH);
    tone(PIN_BUZZER, configuration.configurable.buzzer_frequency, 250);
}

void disableAudioVisualBeacon()
{
    // Toggle power to audiovisual beacon
    digitalWrite(PIN_BLINKER, LOW);
    //delay(25);
}


void setState(uint8_t newState)
{
    uint16_t diff = 0;
    t_msg_debug* msg = (t_msg_debug*)&cmdline[0];

    if(newState >= STATE_NUM_STATES)
    {
        sendError(ERROR_INVALID_STATE);
        return;
    }

    clearMessage();

    // Send CMD_STATE with current state
    msg->header.id_full = CMD_STATE;
    msg->header.millis = millis();
    msg->header.state = stateMachineState;
    msg->payload.code = STATE_REPORT;
    msg->payload.value = newState;
    sendMsg((uint8_t *)msg, sizeof(t_msg_debug));
   
    stateMachineState = newState;
    
    switch(stateMachineState)
    {
        default:
        case STATE_GROUND_IDLE:
        {
            disableAudioVisualBeacon();
            panicTimerReset();
            
            break;
        }

        case STATE_GROUND_IDLE_ON_PAD:
        {
            valsBarometer.ground_pressure = 0.0;
            valsBarometer.apex_pressure = 0.0;

            disableAudioVisualBeacon();
            panicTimerReset();

            break;
        }

        case STATE_GROUND_ARMED:
        {
            disableAudioVisualBeacon();
            panicTimerReset();
            
            break;
        }

        case STATE_AIRBORNE_OUTBOUND:
        {
            disableAudioVisualBeacon();
            panicTimerReset();
            
            barometerdeltastrikes = SENSORS_BAROMETER_STRIKES;
            valsBarometer.apex_pressure = valsBarometer.ground_pressure;
            
            break;
        }

        case STATE_AIRBORNE_DEPLOYMENT:
        {
            disableAudioVisualBeacon();
            
            break;
        }

        case STATE_AIRBORNE_INBOUND:
        {
            disableAudioVisualBeacon();
            panicTimerReset();

            break;
        }

        case STATE_GROUND_RECOVERY:
        {
            enableAudioVisualBeacon();
            panicTimerReset();
            
            break;
        }

        case STATE_GROUND_RESET:
        {
            break;
        }
    }
}

void stateMachine(uint16_t delta_millis)
{
    int32_t diff = 0;
    
    // Always read battery voltage
    doBatteryActions(delta_millis, 0);
    
    switch(stateMachineState)
    {
        case STATE_GROUND_IDLE_ON_PAD:
        {
            doMagnetometerActions(delta_millis, 0);
            doBarometerActions(delta_millis, 0);

            // Keep watch on magnetometer for arming trigger
            if(unarmedMagnetometerReadings > 0)
            {
                // Establish an average for magnetometer magnitude
                unarmedMagnetometerReadings--;
                unarmedMagnetometerMagnitude = (
                                                    ((1 - AVERAGING_FACTOR) * unarmedMagnetometerMagnitude) +
                                                    (AVERAGING_FACTOR * valsMagnetometer.vector.magnitude)
                                                );
                    
            }
            else
            {
                // Start comparing against the established average magnitude
                diff = (int32_t)abs(valsMagnetometer.vector.magnitude - unarmedMagnetometerMagnitude);
                if(diff > configuration.configurable.limit_delta_magnetometer)
                {
                    Serial.print(millis());
                    Serial.println(": MAGNETOMETER MAGNITUDE ABOVE THRESHOLD");
                    setState(STATE_GROUND_ARMED);
                }
            }
            
            // Crude averaging to establish ground pressure
            valsBarometer.ground_pressure = (
                                                ((1 - AVERAGING_FACTOR) * valsBarometer.ground_pressure) +
                                                (AVERAGING_FACTOR * valsBarometer.abs_pressure)
                                            );
            diff = 0;

            break;
        }
        
        case STATE_GROUND_ARMED:
        {
            // Check the two possible launch-indicative magnutides
            doAccelerometerActions(delta_millis, 0);
            doBarometerActions(delta_millis, 0);
            doMagnetometerActions(delta_millis, 0);

            // Accelerometer
            if(valsAccelerometer.vector.magnitude > configuration.configurable.limit_delta_accelerometer)
            {
                Serial.print(millis());
                Serial.println(": ACCELEROMETER MAGNITUDE ABOVE THRESHOLD");
                setState(STATE_AIRBORNE_OUTBOUND);
            }

            // Barometer
            // We'll see an increase in pressure during launch, before it will decrease due to greater altitude. Either way we're simply looking for an absolute difference sufficiently large, so we use abs() here.
            diff = (int32_t) abs(valsBarometer.ground_pressure - valsBarometer.abs_pressure);
            
            if(diff > configuration.configurable.limit_delta_barometer)
            {
                Serial.print(millis());
                Serial.println(": BAROMETER MAGNITUDE ABOVE THRESHOLD");
                setState(STATE_AIRBORNE_OUTBOUND);
            }

            diff = 0;

            break;
        }
        
        case STATE_AIRBORNE_OUTBOUND:
        {
            panicTimerCountdown(delta_millis);

            doAccelerometerActions(delta_millis, 0);
            doBarometerActions(delta_millis, 0);
            doMagnetometerActions(delta_millis, 0);

            // Check for lowest pressure yet
            if(valsBarometer.abs_pressure < valsBarometer.apex_pressure)
            {
                valsBarometer.apex_pressure = valsBarometer.abs_pressure;
            }

            // count down positive barometer deltas, indicating a decreasing height
            // A single positive value could perhaps simply be from pressure oscillations inside the payload capsule due to turbulence during launch, so we could get a premature deployment of parachute?
            if(valsBarometer.abs_pressure < valsBarometer.ground_pressure)
            {
                if((valsBarometer.abs_pressure - valsBarometer.apex_pressure) > configuration.configurable.limit_delta_barometer)
                {
                    Serial.print(millis());
                    Serial.println(": BAROMETER MAGNITUDE ABOVE THRESHOLD (");
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
            
            diff = 0;

            break;
        }
        
        case STATE_AIRBORNE_DEPLOYMENT:
        {
            doAccelerometerActions(delta_millis, 0);
            doBarometerActions(delta_millis, 0);
            doMagnetometerActions(delta_millis, 0);
            
            // Wait for parachute timers
            diff = releaseTheKrakenshoot(delta_millis);
            
            if(diff)
            {
                setState(STATE_AIRBORNE_INBOUND);
            }
            
            diff = 0;
            
            break;
        }
        
        case STATE_AIRBORNE_INBOUND:
        {
            // Counting down to recovery state
            panicTimerCountdown(delta_millis);

            doAccelerometerActions(delta_millis, 0);
            doBarometerActions(delta_millis, 0);
            doMagnetometerActions(delta_millis, 0);

            if(panicActivate)
            {
                Serial.print(millis());
                Serial.println(": Activating locator beacon due to panic timeout.");
                setState(STATE_GROUND_RECOVERY);
            }

            diff = 0;

            break;
        }
        
        case STATE_GROUND_RECOVERY:
        {
            enableAudioVisualBeacon();
            
            // Wait for RBF to be re-applied
            doAccelerometerActions(delta_millis, 0);
            doBarometerActions(delta_millis, 0);
            doMagnetometerActions(delta_millis, 0);
            
            // Magnetometer
            diff = abs(valsMagnetometer.vector.magnitude - unarmedMagnetometerMagnitude);
            
            if(diff < configuration.configurable.limit_delta_magnetometer)
            {
                Serial.println("MAGNETOMETER MAGNITUDE ABOVE THRESHOLD");
                setState(STATE_GROUND_RESET);
                //setState(STATE_GROUND_DATADUMP);
            }

            diff = 0;

            break;
        }

        case STATE_GROUND_RESET:
        {
            doAccelerometerActions(delta_millis, 0);
            doBarometerActions(delta_millis, 0);
            doMagnetometerActions(delta_millis, 0);

            // Wait for parachute reset timers
            diff = resetParachute(delta_millis);
            
            if(diff)
            {
                setState(STATE_GROUND_IDLE);
            }

            diff = 0;

            break;
        }
        
        default:
        case STATE_GROUND_IDLE:
        {
            break;
        }
    }
}

/** Rewrite from serial to RF receive buffer...
 */
void commandCheck()
{
    uint8_t i = 0;
    uint8_t *buffer = &cmdline[0];
    
    // Check for bytes in UART receive buffer
    //cmdlen = Serial.available();
    if(radio.available(PIPE_READ))
    {
        clearMessage();

        for (i = 0; i < COMMAND_LINE_BUFFER_SIZE; i++)
        {
            if(radio.available(PIPE_READ))
            {
                radio.read(buffer++, 1);
            }
        }

        buffer = &cmdline[0];
        
        parseMessage(buffer, i);
    }
}

void setup()
{
    uint8_t sensorStatus = 0;
    uint8_t sensorId = 0;
    Serial.begin(115200);
    while(!Serial)
    {
        // Wait for Serial
    }
    
    Serial.println("setup() begins");

    /*
    Serial.print("UART RX Buffer size: ");
    Serial.println(SERIAL_RX_BUFFER_SIZE);

    Serial.print("Command line buffer size: ");
    Serial.println(COMMAND_LINE_BUFFER_SIZE);

    Serial.print("Command line fill threshold: ");
    Serial.println(COMMAND_LINE_APPEND_THRESHOLD);
    */
    
    clearCommandBuffer();
    
    /*
    Serial.print("Command line length: ");
    Serial.println(cmdlength);
    */
    
    Serial.println("Setting up pins...");

    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_PARACHUTE_SERVO, OUTPUT);
    pinMode(PIN_SERVO_ENABLE, OUTPUT);
    pinMode(PIN_BLINKER, OUTPUT);

    delay(1000);

    // Setup and configure rf radio
    radio.begin();
    radio.setAutoAck(1);                    // Ensure autoACK is enabled
    radio.enableAckPayload();               // Allow optional ack payloads
    radio.setRetries(0, 5);                 // Smallest time between retries, max no. of retries
    //radio.setPayloadSize(32);
    radio.openWritingPipe(pipes[PIPE_WRITE]);        // Both radios listen on the same pipes by default, and switch when writing
    radio.openReadingPipe(1, pipes[PIPE_READ]);
    radio.startListening();                 // Start listening
    radio.printDetails();                   // Dump the configuration of the rf unit for debugging

    //Serial.println("Initializing parachute release servo...");
    
    parachuteReleaseServo.attach(PIN_PARACHUTE_SERVO);

    //Serial.println("Safe position...");

    resetParachute(0);
    //parachuteReleaseServo.write(configuration.servo_safe);

    delay(1000);

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
    if(!bmp.begin())
    {
        Serial.println("Default address initialization of BMP280 failed.");
        if(!bmp.begin(BMP280_ADDRESS_ALT))
        {
            // error
            CLEAR_BIT(sensorStatus, STATUS_BAROMETER_DETECTED);
        }
    }

    bmp.setSampling(
        Adafruit_BMP280::MODE_NORMAL,   /* Operating Mode. */
        Adafruit_BMP280::SAMPLING_X2,   /* Temp. oversampling */
        Adafruit_BMP280::SAMPLING_X16,  /* Pressure oversampling */
        Adafruit_BMP280::FILTER_OFF,    /* Filtering. */
        Adafruit_BMP280::STANDBY_MS_1   /* Standby time. */
    );


    //mag.setMagGain();

    //Serial.println("Accelerometer...");
    SET_BIT(sensorStatus, STATUS_ACCELEROMETER_DETECTED);
    SET_BIT(sensorStatus, STATUS_MAGNETOMETER_DETECTED);

    mpu.beginAccel();
    mpu.beginMag(MAG_MODE_CONTINUOUS_100HZ);

    if (mpu.readId(&sensorId) == 0)
    {
        // A-OK
        Serial.println("sensorId: " + String(sensorId));
    }
    else
    {
        Serial.println("Cannot read sensorId");
        // Error
        CLEAR_BIT(sensorStatus, STATUS_ACCELEROMETER_DETECTED);
        CLEAR_BIT(sensorStatus, STATUS_MAGNETOMETER_DETECTED);

        //while(true)
        //{
        //    // Halt
        //}
    }

    Serial.print("setup() ends at ");
    Serial.println(millis());
}

uint16_t getDeltaMillis()
{
    uint16_t delta_millis = 0;
    uint32_t this_millis = millis();

    // Check for wrap-around of millis counter
    if(this_millis < configuration.dynamic.millis_previous_loop)
    {
        // Add the remainder until wrap-around for previous loop
        delta_millis = (uint32_t)-1 - configuration.dynamic.millis_previous_loop;

        // Add this loop's millis
        delta_millis += this_millis;
    }
    else
    {
        delta_millis = this_millis - configuration.dynamic.millis_previous_loop;
    }

    configuration.dynamic.millis_previous_loop = this_millis;

    return delta_millis;
}

void loop()
{

    uint8_t i = 0;
    uint16_t delta_millis = 0;
    delta_millis = getDeltaMillis();
    
    commandCheck();

    stateMachine(delta_millis);

    delay(10);
}
