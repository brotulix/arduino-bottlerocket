#ifndef __MESSAGES_H

#define COMMAND_LINE_BUFFER_SIZE                32
#define COMMAND_LINE_APPEND_THRESHOLD           ((COMMAND_LINE_BUFFER_SIZE / 4) * 3)

#define PIPE_READ                               0
#define PIPE_WRITE                              1

typedef struct
{
    union {
        uint8_t id_long[4];
        struct {
            uint8_t id_short[3];
            uint8_t state;
        };
        uint32_t id_full;
    };
    uint32_t millis;
} t_msg_header;

typedef struct
{
    float x;
    float y;
    float z;
    float magnitude;
} t_vector;

typedef struct
{
    t_msg_header header;
    t_vector vector;
} t_msg_accelerometer;

typedef struct
{
    t_msg_header header;
    t_vector vector;
    uint16_t heading;   // Magnetic heading in 0.01 degree resolution
} t_msg_magnetometer;

typedef struct
{
    t_msg_header header;
    float abs_pressure;
    float temperature;
    float altitude;
    float apex_pressure;
    float apex_altitude;
    float ground_pressure;
} t_msg_barometer;

typedef struct
{
    uint32_t code;
    uint32_t value;
} t_msg_config;

typedef struct
{
    t_msg_header header;
    t_msg_config payload;
} t_msg_debug;

typedef enum {
    STATE_GROUND_IDLE           = (0x00),
    STATE_GROUND_IDLE_ON_PAD    = (0x01),
    STATE_GROUND_ARMED          = (0x02),
    STATE_AIRBORNE_OUTBOUND     = (0x03),
    STATE_AIRBORNE_DEPLOYMENT   = (0x04),
    STATE_AIRBORNE_INBOUND      = (0x05),
    STATE_GROUND_RECOVERY       = (0x06),
    STATE_GROUND_RESET          = (0x07),
    STATE_NUM_STATES            = (0x08),
    
    STATE_REPORT                = (0x52),   // 'R'
    STATE_CHANGE                = (0x43)    // 'C'
} t_e_states;

typedef enum {
    ERROR_OK,
    ERROR_GENERIC,
    ERROR_INVALID_COMMAND,
    ERROR_INVALID_CONFIG,
    ERROR_INVALID_STATE,
    ERROR_MESSAGE_LENGTH,
    ERROR_LOW_BATTERY_WARNING,
    ERROR_PANIC_TIMEOUT,
    ERROR_DEVICE_NOT_FOUND,
    ERROR_SEND_FAILED
} t_errs;

typedef enum {
    CONFIG_REPORT,

    // Configurable part of configuration
    CONFIG_INTERVAL_READ_ACCELEROMETER,
    CONFIG_INTERVAL_READ_BAROMETER,
    CONFIG_INTERVAL_READ_MAGNETOMETER,
    CONFIG_INTERVAL_READ_BATTERY,
    CONFIG_LIMIT_DELTA_ACCELEROMETER,
    CONFIG_LIMIT_DELTA_BAROMETER,
    CONFIG_LIMIT_DELTA_MAGNETOMETER,
    CONFIG_LIMIT_BATTERY_VOLTAGE,
    CONFIG_TIMEOUT_PANIC_OUTBOUND,
    CONFIG_TIMEOUT_PANIC_INBOUND,
    CONFIG_BUZZER_FREQUENCY,
    CONFIG_SERVO_SAFE,
    CONFIG_SERVO_RELEASE,
    CONFIG_SERVO_HOLD
} t_e_config_fields;

typedef enum
{
    CMD_CONFIG                  = (0x244324), // "$C$"
    CMD_DEBUG                   = (0x244424), // "$D$"
    CMD_ERROR                   = (0x244524), // "$E$"
    CMD_PINGPONG                = (0x245024), // "$P$"
    CMD_READ                    = (0x245224), // "$R$"
    CMD_STATE                   = (0x245324), // "$S$"
    CMD_RESET                   = (0x245A24), // "$Z$"
} t_e_commands;


#endif // __MESSAGES_H