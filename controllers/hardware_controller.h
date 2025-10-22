/**
 * @file hardware_controller.h
 * @brief Hardware Abstraction Layer Controller
 * @author Enhanced by AI Assistant
 * @version v2.0.0
 * @date 2025
 *
 * Hardware controller that manages all low-level hardware interfaces including
 * ADC, PWM, GPIO, timers, and other peripherals. This provides a clean
 * abstraction layer between hardware and application logic.
 */

#ifndef HARDWARE_CONTROLLER_H
#define HARDWARE_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /* =============================================================================
     * TYPE DEFINITIONS AND ENUMERATIONS
     * ============================================================================= */

    /**
     * @brief GPIO pin states
     */
    typedef enum
    {
        GPIO_STATE_LOW = 0, /**< GPIO pin low */
        GPIO_STATE_HIGH     /**< GPIO pin high */
    } GpioState_t;

    /**
     * @brief GPIO pin identifiers
     */
    typedef enum
    {
        GPIO_PIN_LED_STATUS = 0, /**< Status LED pin */
        GPIO_PIN_BUTTON_USER,    /**< User button pin */
        GPIO_PIN_EMERGENCY_STOP, /**< Emergency stop button */
        GPIO_PIN_MOTOR1_ENABLE,  /**< Motor 1 enable pin */
        GPIO_PIN_MOTOR2_ENABLE,  /**< Motor 2 enable pin */
        GPIO_PIN_FAULT_OUTPUT,   /**< Fault output pin */
        GPIO_PIN_COUNT           /**< Total number of GPIO pins */
    } GpioPinId_t;

    /**
     * @brief ADC channel identifiers
     */
    typedef enum
    {
        ADC_CHANNEL_MOTOR1_IA = 0, /**< Motor 1 phase A current */
        ADC_CHANNEL_MOTOR1_IB,     /**< Motor 1 phase B current */
        ADC_CHANNEL_MOTOR1_IC,     /**< Motor 1 phase C current */
        ADC_CHANNEL_MOTOR2_IA,     /**< Motor 2 phase A current */
        ADC_CHANNEL_MOTOR2_IB,     /**< Motor 2 phase B current */
        ADC_CHANNEL_MOTOR2_IC,     /**< Motor 2 phase C current */
        ADC_CHANNEL_DC_VOLTAGE,    /**< DC bus voltage */
        ADC_CHANNEL_TEMPERATURE,   /**< System temperature */
        ADC_CHANNEL_SPEED_POT,     /**< Speed potentiometer */
        ADC_CHANNEL_COUNT          /**< Total number of ADC channels */
    } AdcChannelId_t;

    /**
     * @brief PWM channel identifiers
     */
    typedef enum
    {
        PWM_CHANNEL_MOTOR1_A_HIGH = 0, /**< Motor 1 phase A high side */
        PWM_CHANNEL_MOTOR1_A_LOW,      /**< Motor 1 phase A low side */
        PWM_CHANNEL_MOTOR1_B_HIGH,     /**< Motor 1 phase B high side */
        PWM_CHANNEL_MOTOR1_B_LOW,      /**< Motor 1 phase B low side */
        PWM_CHANNEL_MOTOR1_C_HIGH,     /**< Motor 1 phase C high side */
        PWM_CHANNEL_MOTOR1_C_LOW,      /**< Motor 1 phase C low side */
        PWM_CHANNEL_MOTOR2_A_HIGH,     /**< Motor 2 phase A high side */
        PWM_CHANNEL_MOTOR2_A_LOW,      /**< Motor 2 phase A low side */
        PWM_CHANNEL_MOTOR2_B_HIGH,     /**< Motor 2 phase B high side */
        PWM_CHANNEL_MOTOR2_B_LOW,      /**< Motor 2 phase B low side */
        PWM_CHANNEL_MOTOR2_C_HIGH,     /**< Motor 2 phase C high side */
        PWM_CHANNEL_MOTOR2_C_LOW,      /**< Motor 2 phase C low side */
        PWM_CHANNEL_COUNT              /**< Total number of PWM channels */
    } PwmChannelId_t;

    /**
     * @brief Hall sensor identifiers
     */
    typedef enum
    {
        HALL_SENSOR_MOTOR1_A = 0, /**< Motor 1 Hall sensor A */
        HALL_SENSOR_MOTOR1_B,     /**< Motor 1 Hall sensor B */
        HALL_SENSOR_MOTOR1_C,     /**< Motor 1 Hall sensor C */
        HALL_SENSOR_MOTOR2_A,     /**< Motor 2 Hall sensor A */
        HALL_SENSOR_MOTOR2_B,     /**< Motor 2 Hall sensor B */
        HALL_SENSOR_MOTOR2_C,     /**< Motor 2 Hall sensor C */
        HALL_SENSOR_COUNT         /**< Total number of Hall sensors */
    } HallSensorId_t;

    /**
     * @brief Hardware configuration structure
     */
    typedef struct
    {
        struct
        {
            uint32_t system_clock_hz;  /**< System clock frequency */
            uint32_t adc_clock_hz;     /**< ADC clock frequency */
            uint32_t pwm_frequency_hz; /**< PWM switching frequency */
            uint16_t dead_time_ns;     /**< PWM dead time */
        } clocks;

        struct
        {
            bool continuous_mode; /**< ADC continuous conversion mode */
            bool dma_enabled;     /**< DMA enabled for ADC */
            uint8_t sample_time;  /**< ADC sample time setting */
            bool injected_mode;   /**< Use injected conversion group */
        } adc_config;

        struct
        {
            bool complementary_mode; /**< Enable complementary PWM outputs */
            bool center_aligned;     /**< Center-aligned PWM mode */
            uint8_t resolution_bits; /**< PWM resolution in bits */
            bool fault_protection;   /**< Enable fault protection */
        } pwm_config;

    } HardwareConfig_t;

    /**
     * @brief ADC conversion results structure
     */
    typedef struct
    {
        uint16_t raw_values[ADC_CHANNEL_COUNT];       /**< Raw ADC values */
        int16_t calibrated_values[ADC_CHANNEL_COUNT]; /**< Calibrated values */
        bool conversion_complete;                     /**< Conversion complete flag */
        uint32_t timestamp_us;                        /**< Conversion timestamp */
    } AdcResults_t;

    /**
     * @brief PWM duty cycle structure (per motor)
     */
    typedef struct
    {
        uint16_t phase_a_duty; /**< Phase A duty cycle (0-1000) */
        uint16_t phase_b_duty; /**< Phase B duty cycle (0-1000) */
        uint16_t phase_c_duty; /**< Phase C duty cycle (0-1000) */
        bool enable_output;    /**< Enable PWM output */
    } PwmDutyCycle_t;

/* =============================================================================
 * CONFIGURATION CONSTANTS
 * ============================================================================= */

/** Default system clock frequency (Hz) */
#define HW_DEFAULT_SYSTEM_CLOCK_HZ (144000000UL)

/** Default PWM frequency (Hz) */
#define HW_DEFAULT_PWM_FREQUENCY_HZ (20000)

/** PWM resolution (0-1000 for 0-100%) */
#define HW_PWM_RESOLUTION (1000)

/** ADC reference voltage (mV) */
#define HW_ADC_REFERENCE_MV (3300)

/** ADC resolution (bits) */
#define HW_ADC_RESOLUTION_BITS (12)

/** Maximum ADC value */
#define HW_ADC_MAX_VALUE (4095)

/** GPIO debounce time (ms) */
#define HW_GPIO_DEBOUNCE_MS (50)

    /* =============================================================================
     * PUBLIC FUNCTION DECLARATIONS
     * ============================================================================= */

    /**
     * @brief Initialize hardware controller
     * @details Initializes all hardware peripherals and configurations
     * @param config Hardware configuration structure
     * @return bool True if initialization successful
     */
    bool HardwareController_Initialize(const HardwareConfig_t *config);

    /**
     * @brief Deinitialize hardware controller
     * @details Safely shuts down all hardware peripherals
     */
    void HardwareController_Deinitialize(void);

    /* GPIO Functions */
    /**
     * @brief Set GPIO pin state
     * @param pin_id GPIO pin identifier
     * @param state Pin state to set
     * @return bool True if operation successful
     */
    bool HardwareController_SetGpioState(GpioPinId_t pin_id, GpioState_t state);

    /**
     * @brief Get GPIO pin state
     * @param pin_id GPIO pin identifier
     * @return GpioState_t Current pin state
     */
    GpioState_t HardwareController_GetGpioState(GpioPinId_t pin_id);

    /**
     * @brief Toggle GPIO pin state
     * @param pin_id GPIO pin identifier
     * @return bool True if operation successful
     */
    bool HardwareController_ToggleGpio(GpioPinId_t pin_id);

    /* ADC Functions */
    /**
     * @brief Start ADC conversion
     * @details Initiates ADC conversion for all configured channels
     * @return bool True if conversion started successfully
     */
    bool HardwareController_StartAdcConversion(void);

    /**
     * @brief Get ADC conversion results
     * @param results Pointer to structure to populate with ADC results
     * @return bool True if results are available and valid
     */
    bool HardwareController_GetAdcResults(AdcResults_t *results);

    /**
     * @brief Get single ADC channel value
     * @param channel_id ADC channel identifier
     * @return uint16_t Raw ADC value (0-4095)
     */
    uint16_t HardwareController_GetAdcValue(AdcChannelId_t channel_id);

    /**
     * @brief Convert ADC value to voltage
     * @param adc_value Raw ADC value
     * @return uint16_t Voltage in millivolts
     */
    uint16_t HardwareController_AdcToVoltage(uint16_t adc_value);

    /**
     * @brief Convert ADC value to current
     * @param adc_value Raw ADC value
     * @param shunt_resistance_mohms Shunt resistance in milliohms
     * @param amplifier_gain Amplifier gain
     * @return int16_t Current in milliamps (signed)
     */
    int16_t HardwareController_AdcToCurrent(uint16_t adc_value,
                                            uint16_t shunt_resistance_mohms,
                                            uint16_t amplifier_gain);

    /* PWM Functions */
    /**
     * @brief Set motor PWM duty cycles
     * @param motor_id Motor identifier (0 or 1)
     * @param duty_cycles PWM duty cycle structure
     * @return bool True if operation successful
     */
    bool HardwareController_SetMotorPwm(uint8_t motor_id, const PwmDutyCycle_t *duty_cycles);

    /**
     * @brief Enable/disable motor PWM outputs
     * @param motor_id Motor identifier (0 or 1)
     * @param enable Enable flag
     * @return bool True if operation successful
     */
    bool HardwareController_EnableMotorPwm(uint8_t motor_id, bool enable);

    /**
     * @brief Emergency disable all PWM outputs
     * @details Immediately disables all PWM outputs for safety
     */
    void HardwareController_EmergencyDisablePwm(void);

    /**
     * @brief Set PWM frequency
     * @param frequency_hz New PWM frequency in Hz
     * @return bool True if frequency set successfully
     */
    bool HardwareController_SetPwmFrequency(uint32_t frequency_hz);

    /* Hall Sensor Functions */
    /**
     * @brief Get Hall sensor state
     * @param motor_id Motor identifier (0 or 1)
     * @return uint8_t Hall sensor state (3-bit value)
     */
    uint8_t HardwareController_GetHallState(uint8_t motor_id);

    /**
     * @brief Get individual Hall sensor value
     * @param sensor_id Hall sensor identifier
     * @return bool Hall sensor state (true = high, false = low)
     */
    bool HardwareController_GetHallSensor(HallSensorId_t sensor_id);

    /* Timer Functions */
    /**
     * @brief Get system timestamp in microseconds
     * @return uint32_t Current timestamp in microseconds
     */
    uint32_t HardwareController_GetTimestampUs(void);

    /**
     * @brief Get system timestamp in milliseconds
     * @return uint32_t Current timestamp in milliseconds
     */
    uint32_t HardwareController_GetTimestampMs(void);

    /**
     * @brief Delay execution for specified microseconds
     * @param microseconds Delay time in microseconds
     */
    void HardwareController_DelayUs(uint32_t microseconds);

    /**
     * @brief Delay execution for specified milliseconds
     * @param milliseconds Delay time in milliseconds
     */
    void HardwareController_DelayMs(uint32_t milliseconds);

    /* Interrupt Functions */
    /**
     * @brief Register ADC conversion complete callback
     * @param callback Callback function to register
     * @return bool True if callback registered successfully
     */
    bool HardwareController_RegisterAdcCallback(void (*callback)(AdcResults_t *results));

    /**
     * @brief Register PWM period elapsed callback
     * @param callback Callback function to register
     * @return bool True if callback registered successfully
     */
    bool HardwareController_RegisterPwmCallback(void (*callback)(void));

    /**
     * @brief Register Hall sensor change callback
     * @param callback Callback function to register
     * @return bool True if callback registered successfully
     */
    bool HardwareController_RegisterHallCallback(void (*callback)(uint8_t motor_id, uint8_t hall_state));

    /* System Functions */
    /**
     * @brief Reset system microcontroller
     * @details Performs a software reset of the system
     */
    void HardwareController_SystemReset(void);

    /**
     * @brief Enter low power mode
     * @details Puts the system into low power sleep mode
     */
    void HardwareController_EnterLowPowerMode(void);

    /**
     * @brief Get hardware fault status
     * @return uint16_t Hardware fault status bits
     */
    uint16_t HardwareController_GetFaultStatus(void);

    /**
     * @brief Clear hardware faults
     * @return bool True if faults cleared successfully
     */
    bool HardwareController_ClearFaults(void);

/* =============================================================================
 * UTILITY MACROS
 * ============================================================================= */

/** Convert duty cycle percentage to internal format */
#define HW_PERCENT_TO_DUTY(percent) ((uint16_t)((percent) * HW_PWM_RESOLUTION / 100))

/** Convert internal duty cycle to percentage */
#define HW_DUTY_TO_PERCENT(duty) ((uint8_t)((duty) * 100 / HW_PWM_RESOLUTION))

/** Check if ADC value is valid */
#define HW_IS_VALID_ADC(value) ((value) <= HW_ADC_MAX_VALUE)

/** Check if PWM duty cycle is valid */
#define HW_IS_VALID_DUTY(duty) ((duty) <= HW_PWM_RESOLUTION)

/** Convert voltage to ADC counts */
#define HW_VOLTAGE_TO_ADC(mv) ((uint16_t)((mv) * HW_ADC_MAX_VALUE / HW_ADC_REFERENCE_MV))

#ifdef __cplusplus
}
#endif

#endif /* HARDWARE_CONTROLLER_H */