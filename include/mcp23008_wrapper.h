#ifndef MCP23008_WRAPPER_H
#define MCP23008_WRAPPER_H

#include "mcp23008.h" // For mcp23008_t and esp_err_t
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MCP23008_DEFAULT_IODIR ( \
    (1 << MCP_PIN_JOYSTICK_ENTER)    | \
    (0 << MCP_PIN_BATT_SENSE_SWITCH)| \
    (1 << MCP_PIN_ACCESSORY_C)      | \
    (0 << MCP_PIN_CD4053B_S1)       | \
    (0 << MCP_PIN_CD4053B_S2)       | \
    (0 << MCP_PIN_CD4053B_S3)       | \
    (0 << MCP_PIN_ETH_LED_1)        | \
    (0 << MCP_PIN_ETH_LED_2)          \
)

#define MCP23008_DEFAULT_GPPU ( \
    (1 << MCP_PIN_JOYSTICK_ENTER)    | \
    (0 << MCP_PIN_BATT_SENSE_SWITCH) | \
    (1 << MCP_PIN_ACCESSORY_C)      | \
    (0 << MCP_PIN_CD4053B_S1)       | \
    (0 << MCP_PIN_CD4053B_S2)       | \
    (0 << MCP_PIN_CD4053B_S3)       | \
    (0 << MCP_PIN_ETH_LED_1)        | \
    (0 << MCP_PIN_ETH_LED_2)          \
)

// --- SECOND MCP23008 SUPPORT ---
#define MCP23008_2_DEFAULT_IODIR ( \
    (0 << MCP2_PIN_HAPTIC_MOTOR)    | \
    (1 << MCP2_PIN_GUN_TRIGGER)      | \
    (1 << MCP2_PIN_ACCESSORY_4)      | \
    (0 << MCP2_PIN_CD4053B_S3)       | \
    (0 << MCP2_PIN_CD4053B_S2)       | \
    (0 << MCP2_PIN_CD4053B_S1)       | \
    (0 << MCP2_PIN_ETH_LED_1)        | \
    (0 << MCP2_PIN_ETH_LED_2)          \
)
#define MCP23008_2_DEFAULT_GPPU ( \
    (0 << MCP2_PIN_HAPTIC_MOTOR)    | \
    (1 << MCP2_PIN_GUN_TRIGGER) | \
    (1 << MCP2_PIN_ACCESSORY_4)      | \
    (0 << MCP2_PIN_CD4053B_S1)       | \
    (0 << MCP2_PIN_CD4053B_S2)       | \
    (0 << MCP2_PIN_CD4053B_S3)       | \
    (0 << MCP2_PIN_ETH_LED_1)        | \
    (0 << MCP2_PIN_ETH_LED_2)          \
)

typedef enum {
    MCP_PIN_JOYSTICK_ENTER    = 0, // GPIO Pin 0
    MCP_PIN_BATT_SENSE_SWITCH  = 1, // GPIO Pin 1
    MCP_PIN_ACCESSORY_C  = 2, // GPIO Pin 2
    MCP_PIN_CD4053B_S1 = 3, // GPIO Pin 3
    MCP_PIN_CD4053B_S2 = 4, // GPIO Pin 4
    MCP_PIN_CD4053B_S3   = 5, // GPIO Pin 5
    MCP_PIN_ETH_LED_1   = 6, // GPIO Pin 6
    MCP_PIN_ETH_LED_2  = 7  // GPIO Pin 7
} MCP23008_NamedPin;

typedef enum {
    MCP2_PIN_HAPTIC_MOTOR    = 0, // GPIO Pin 0
    MCP2_PIN_GUN_TRIGGER  = 1, // GPIO Pin 1
    MCP2_PIN_ACCESSORY_4  = 2, // GPIO Pin 2
    MCP2_PIN_CD4053B_S3 = 3, // GPIO Pin 3
    MCP2_PIN_CD4053B_S2 = 4, // GPIO Pin 4
    MCP2_PIN_CD4053B_S1   = 5, // GPIO Pin 5
    MCP2_PIN_ETH_LED_1   = 6, // GPIO Pin 6
    MCP2_PIN_ETH_LED_2  = 7  // GPIO Pin 7
} MCP23008_2_NamedPin;

typedef enum {
    MCP_PIN_DIR_OUTPUT = 0,
    MCP_PIN_DIR_INPUT  = 1
} MCP23008_PinDirection;

esp_err_t mcp23008_wrapper_init_with_defaults(mcp23008_t *mcp, uint8_t iodir, uint8_t gppu);
esp_err_t mcp23008_wrapper_init(mcp23008_t *mcp);
esp_err_t mcp23008_wrapper_set_pin_direction(mcp23008_t *mcp, MCP23008_NamedPin pin, MCP23008_PinDirection direction);
esp_err_t mcp23008_wrapper_set_pin_pullup(mcp23008_t *mcp, MCP23008_NamedPin pin, bool enabled);
esp_err_t mcp23008_wrapper_write_pin(mcp23008_t *mcp, MCP23008_NamedPin pin, bool value);
esp_err_t mcp23008_wrapper_read_pin(mcp23008_t *mcp, MCP23008_NamedPin pin, bool *value);
esp_err_t mcp23008_wrapper_toggle_pin(mcp23008_t *mcp, MCP23008_NamedPin pin);
esp_err_t mcp23008_check_present(mcp23008_t *mcp);

#ifdef __cplusplus
}
#endif

#endif // MCP23008_WRAPPER_H
