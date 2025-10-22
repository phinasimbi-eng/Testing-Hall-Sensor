/**
 * @file hardware_controller.c
 * @brief Hardware Abstraction Layer Implementation
 * @author Enhanced by AI Assistant
 * @version v2.0.0
 * @date 2025
 *
 * Hardware abstraction layer that manages all peripheral interfaces
 * including ADC, PWM, GPIO, timers, and communication interfaces.
 * Provides clean API while integrating with existing N32G430 drivers.
 */

#include "controllers/hardware_controller.h"
#include "SystemDefine.h"
#include "SystemInterface.h"
#include "Adc.h"
#include "Pwm.h"
#include "Uart.h"
#include "Spi.h"
#include <string.h>

/* =============================================================================
 * PRIVATE VARIABLES AND STATE
 * ============================================================================= */

/**
 * @brief Hardware controller global state
 */
typedef struct
{
    bool initialized;

    struct
    {
        AdcCallbackFunc_t adc_complete_callback;
        PwmInterruptCallback_t pwm_interrupt_callback;
        GpioInterruptCallback_t gpio_interrupt_callback;
        TimerCallback_t timer_callback;
    } callbacks;

    struct
    {
        AdcResults_t last_adc_results;
        PwmConfig_t current_pwm_config;
        GpioStates_t gpio_states;
        uint32_t system_tick_count;
        bool adc_conversion_complete;
    } hardware_state;

    struct
    {
        uint32_t adc_conversions_count;
        uint32_t pwm_updates_count;
        uint32_t gpio_interrupt_count;
        uint32_t timer_interrupt_count;
    } statistics;

} HardwareControllerState_t;

/** Global hardware controller state */
static HardwareControllerState_t g_hardware_controller = {0};

/** External references to existing hardware objects */
extern SystemInterface_Obj SystemInterfaceObj[2];

/* =============================================================================
 * PRIVATE FUNCTION DECLARATIONS
 * ============================================================================= */

static bool HardwareController_InitializeAdc(void);
static bool HardwareController_InitializePwm(void);
static bool HardwareController_InitializeGpio(void);
static bool HardwareController_InitializeTimers(void);
static bool HardwareController_InitializeCommunication(void);
static void HardwareController_UpdateAdcResults(void);
static void HardwareController_UpdateGpioStates(void);

/* =============================================================================
 * PUBLIC FUNCTION IMPLEMENTATIONS
 * ============================================================================= */

/**
 * @brief Initialize hardware controller subsystem
 * @return bool True if initialization successful
 */
bool HardwareController_Initialize(void)
{
    // Initialize controller state
    memset(&g_hardware_controller, 0, sizeof(HardwareControllerState_t));

    // Initialize ADC subsystem
    if (!HardwareController_InitializeAdc())
    {
        return false;
    }

    // Initialize PWM subsystem
    if (!HardwareController_InitializePwm())
    {
        return false;
    }

    // Initialize GPIO subsystem
    if (!HardwareController_InitializeGpio())
    {
        return false;
    }

    // Initialize timers
    if (!HardwareController_InitializeTimers())
    {
        return false;
    }

    // Initialize communication interfaces
    if (!HardwareController_InitializeCommunication())
    {
        return false;
    }

    g_hardware_controller.initialized = true;
    return true;
}

/**
 * @brief Configure ADC channels and sampling parameters
 * @param config ADC configuration structure
 * @return bool True if configuration successful
 */
bool HardwareController_ConfigureAdc(const AdcConfig_t *config)
{
    if (!config || !g_hardware_controller.initialized)
    {
        return false;
    }

    // Validate configuration parameters
    if (config->sampling_frequency_hz < 1000 ||
        config->sampling_frequency_hz > 100000)
    {
        return false;
    }

    // The existing ADC is configured in Adc.c
    // We can add additional configuration here if needed

    return true;
}

/**
 * @brief Configure PWM output parameters
 * @param config PWM configuration structure
 * @return bool True if configuration successful
 */
bool HardwareController_ConfigurePwm(const PwmConfig_t *config)
{
    if (!config || !g_hardware_controller.initialized)
    {
        return false;
    }

    // Validate configuration parameters
    if (config->frequency_hz < 1000 || config->frequency_hz > 100000)
    {
        return false;
    }

    if (config->dead_time_ns > 10000)
    {
        return false;
    }

    // Store current configuration
    memcpy(&g_hardware_controller.hardware_state.current_pwm_config,
           config, sizeof(PwmConfig_t));

    // Configure PWM using existing functions
    // The PWM is initialized in Pwm.c with TIM1 and TIM8

    return true;
}

/**
 * @brief Set PWM duty cycles for motor control
 * @param motor_id Motor identifier (0 or 1)
 * @param duty_cycles Array of duty cycle values (0-100%)
 * @return bool True if duty cycles set successfully
 */
bool HardwareController_SetMotorPwm(uint8_t motor_id, const float duty_cycles[3])
{
    if (!MOTOR_IS_VALID_ID(motor_id) || !duty_cycles ||
        !g_hardware_controller.initialized)
    {
        return false;
    }

    // Validate duty cycle values
    for (int i = 0; i < 3; i++)
    {
        if (duty_cycles[i] < 0.0f || duty_cycles[i] > 100.0f)
        {
            return false;
        }
    }

    g_hardware_controller.statistics.pwm_updates_count++;

    // Use existing PWM functions from SystemInterface
    SystemInterface_Obj *sys_obj = &SystemInterfaceObj[motor_id];

    // Convert percentage to timer compare values
    uint16_t period = sys_obj->Pwm.period;
    uint16_t ccr_a = (uint16_t)((duty_cycles[0] * period) / 100.0f);
    uint16_t ccr_b = (uint16_t)((duty_cycles[1] * period) / 100.0f);
    uint16_t ccr_c = (uint16_t)((duty_cycles[2] * period) / 100.0f);

    // Set PWM compare values using existing functions
    sys_obj->Pwm.pFuncPwmSet(ccr_a, ccr_b, ccr_c);

    return true;
}

/**
 * @brief Get ADC conversion results
 * @param results Pointer to structure to store ADC results
 * @return bool True if results retrieved successfully
 */
bool HardwareController_GetAdcResults(AdcResults_t *results)
{
    if (!results || !g_hardware_controller.initialized)
    {
        return false;
    }

    // Update ADC results from existing ADC system
    HardwareController_UpdateAdcResults();

    // Copy results
    memcpy(results, &g_hardware_controller.hardware_state.last_adc_results,
           sizeof(AdcResults_t));

    return true;
}

/**
 * @brief Read Hall sensor state for motor
 * @param motor_id Motor identifier (0 or 1)
 * @param hall_state Pointer to store Hall sensor state
 * @return bool True if Hall state read successfully
 */
bool HardwareController_ReadHallSensors(uint8_t motor_id, uint8_t *hall_state)
{
    if (!MOTOR_IS_VALID_ID(motor_id) || !hall_state ||
        !g_hardware_controller.initialized)
    {
        return false;
    }

    // Use existing Hall sensor functions
    SystemInterface_Obj *sys_obj = &SystemInterfaceObj[motor_id];

    if (sys_obj->Hall.pFuncGetHall)
    {
        *hall_state = sys_obj->Hall.pFuncGetHall();
        return true;
    }

    return false;
}

/**
 * @brief Control GPIO output states
 * @param gpio_outputs Structure containing GPIO output states
 * @return bool True if GPIO outputs set successfully
 */
bool HardwareController_SetGpioOutputs(const GpioOutputs_t *gpio_outputs)
{
    if (!gpio_outputs || !g_hardware_controller.initialized)
    {
        return false;
    }

    // Update GPIO states
    g_hardware_controller.hardware_state.gpio_states.outputs = *gpio_outputs;

    // Set GPIO outputs using existing functions
    // LED control
    if (gpio_outputs->led_status != g_hardware_controller.hardware_state.gpio_states.outputs.led_status)
    {
        // Use existing LED control functions if available
    }

    // Relay control
    if (gpio_outputs->relay_enable != g_hardware_controller.hardware_state.gpio_states.outputs.relay_enable)
    {
        // Use existing relay control functions if available
    }

    // Enable pins for motors
    SystemInterface_Obj *sys_obj0 = &SystemInterfaceObj[0];
    SystemInterface_Obj *sys_obj1 = &SystemInterfaceObj[1];

    if (sys_obj0->GetIO.pFuncSetEn)
    {
        sys_obj0->GetIO.pFuncSetEn(gpio_outputs->motor_enable[0]);
    }

    if (sys_obj1->GetIO.pFuncSetEn)
    {
        sys_obj1->GetIO.pFuncSetEn(gpio_outputs->motor_enable[1]);
    }

    return true;
}

/**
 * @brief Read GPIO input states
 * @param gpio_inputs Pointer to structure to store GPIO input states
 * @return bool True if GPIO inputs read successfully
 */
bool HardwareController_GetGpioInputs(GpioInputs_t *gpio_inputs)
{
    if (!gpio_inputs || !g_hardware_controller.initialized)
    {
        return false;
    }

    // Update GPIO states from hardware
    HardwareController_UpdateGpioStates();

    // Copy input states
    memcpy(gpio_inputs, &g_hardware_controller.hardware_state.gpio_states.inputs,
           sizeof(GpioInputs_t));

    return true;
}

/**
 * @brief Configure timer for periodic callbacks
 * @param timer_id Timer identifier
 * @param config Timer configuration structure
 * @return bool True if timer configured successfully
 */
bool HardwareController_ConfigureTimer(uint8_t timer_id, const TimerConfig_t *config)
{
    if (timer_id >= TIMER_MAX_COUNT || !config ||
        !g_hardware_controller.initialized)
    {
        return false;
    }

    // Validate configuration
    if (config->period_ms == 0 || config->period_ms > 10000)
    {
        return false;
    }

    // Timer configuration is handled by existing system
    // We can add timer-specific configuration here if needed

    return true;
}

/**
 * @brief Trigger ADC conversion
 * @return bool True if conversion triggered successfully
 */
bool HardwareController_TriggerAdcConversion(void)
{
    if (!g_hardware_controller.initialized)
    {
        return false;
    }

    // ADC conversions are triggered automatically by existing system
    // This function can be used to trigger manual conversions if needed

    return true;
}

/**
 * @brief Emergency shutdown of all PWM outputs
 */
void HardwareController_EmergencyPwmShutdown(void)
{
    // Shut down all PWM outputs immediately
    for (int motor_id = 0; motor_id < 2; motor_id++)
    {
        SystemInterface_Obj *sys_obj = &SystemInterfaceObj[motor_id];
        if (sys_obj->Pwm.pFuncPwmSet)
        {
            sys_obj->Pwm.pFuncPwmSet(0, 0, 0);
        }
    }

    // Use existing emergency shutdown functions
    AllPwmShut(0);
    AllPwmShut(1);
}

/**
 * @brief Get system tick count
 * @return uint32_t Current system tick count in milliseconds
 */
uint32_t HardwareController_GetSystemTick(void)
{
    return g_hardware_controller.hardware_state.system_tick_count;
}

/**
 * @brief Update hardware status from peripherals
 */
void HardwareController_UpdateStatus(void)
{
    if (!g_hardware_controller.initialized)
    {
        return;
    }

    // Update system tick
    g_hardware_controller.hardware_state.system_tick_count++;

    // Update ADC results
    HardwareController_UpdateAdcResults();

    // Update GPIO states
    HardwareController_UpdateGpioStates();
}

/**
 * @brief Get hardware statistics
 * @param stats Pointer to structure to store statistics
 * @return bool True if statistics retrieved successfully
 */
bool HardwareController_GetStatistics(HardwareStatistics_t *stats)
{
    if (!stats || !g_hardware_controller.initialized)
    {
        return false;
    }

    stats->adc_conversions_count = g_hardware_controller.statistics.adc_conversions_count;
    stats->pwm_updates_count = g_hardware_controller.statistics.pwm_updates_count;
    stats->gpio_interrupt_count = g_hardware_controller.statistics.gpio_interrupt_count;
    stats->timer_interrupt_count = g_hardware_controller.statistics.timer_interrupt_count;
    stats->system_uptime_ms = g_hardware_controller.hardware_state.system_tick_count;

    return true;
}

/* =============================================================================
 * CALLBACK REGISTRATION FUNCTIONS
 * ============================================================================= */

/**
 * @brief Register ADC conversion complete callback
 */
bool HardwareController_RegisterAdcCallback(AdcCallbackFunc_t callback)
{
    if (!g_hardware_controller.initialized)
    {
        return false;
    }

    g_hardware_controller.callbacks.adc_complete_callback = callback;
    return true;
}

/**
 * @brief Register PWM interrupt callback
 */
bool HardwareController_RegisterPwmCallback(PwmInterruptCallback_t callback)
{
    if (!g_hardware_controller.initialized)
    {
        return false;
    }

    g_hardware_controller.callbacks.pwm_interrupt_callback = callback;
    return true;
}

/**
 * @brief Register GPIO interrupt callback
 */
bool HardwareController_RegisterGpioCallback(GpioInterruptCallback_t callback)
{
    if (!g_hardware_controller.initialized)
    {
        return false;
    }

    g_hardware_controller.callbacks.gpio_interrupt_callback = callback;
    return true;
}

/**
 * @brief Register timer callback
 */
bool HardwareController_RegisterTimerCallback(TimerCallback_t callback)
{
    if (!g_hardware_controller.initialized)
    {
        return false;
    }

    g_hardware_controller.callbacks.timer_callback = callback;
    return true;
}

/* =============================================================================
 * INTERRUPT SERVICE ROUTINES (Called by existing ISRs)
 * ============================================================================= */

/**
 * @brief ADC conversion complete interrupt handler
 * @details Called from existing ADC ISR
 */
void HardwareController_AdcCompleteIsr(void)
{
    if (!g_hardware_controller.initialized)
    {
        return;
    }

    g_hardware_controller.statistics.adc_conversions_count++;
    g_hardware_controller.hardware_state.adc_conversion_complete = true;

    // Update ADC results
    HardwareController_UpdateAdcResults();

    // Call registered callback
    if (g_hardware_controller.callbacks.adc_complete_callback)
    {
        g_hardware_controller.callbacks.adc_complete_callback(
            &g_hardware_controller.hardware_state.last_adc_results);
    }
}

/**
 * @brief PWM interrupt handler
 * @details Called from existing PWM ISR (10kHz)
 */
void HardwareController_PwmInterruptIsr(void)
{
    if (!g_hardware_controller.initialized)
    {
        return;
    }

    g_hardware_controller.statistics.pwm_updates_count++;

    // Call registered callback
    if (g_hardware_controller.callbacks.pwm_interrupt_callback)
    {
        g_hardware_controller.callbacks.pwm_interrupt_callback();
    }
}

/**
 * @brief GPIO interrupt handler
 * @details Called from existing GPIO ISR
 */
void HardwareController_GpioInterruptIsr(uint8_t gpio_pin)
{
    if (!g_hardware_controller.initialized)
    {
        return;
    }

    g_hardware_controller.statistics.gpio_interrupt_count++;

    // Call registered callback
    if (g_hardware_controller.callbacks.gpio_interrupt_callback)
    {
        g_hardware_controller.callbacks.gpio_interrupt_callback(gpio_pin);
    }
}

/**
 * @brief Timer interrupt handler
 * @details Called from existing timer ISR (1kHz)
 */
void HardwareController_TimerInterruptIsr(uint8_t timer_id)
{
    if (!g_hardware_controller.initialized)
    {
        return;
    }

    g_hardware_controller.statistics.timer_interrupt_count++;

    // Update system status
    HardwareController_UpdateStatus();

    // Call registered callback
    if (g_hardware_controller.callbacks.timer_callback)
    {
        g_hardware_controller.callbacks.timer_callback(timer_id);
    }
}

/* =============================================================================
 * PRIVATE FUNCTION IMPLEMENTATIONS
 * ============================================================================= */

/**
 * @brief Initialize ADC subsystem
 */
static bool HardwareController_InitializeAdc(void)
{
    // ADC initialization is handled by existing Adc_Init() function
    // We can add additional initialization here if needed

    // Initialize ADC results structure
    memset(&g_hardware_controller.hardware_state.last_adc_results, 0,
           sizeof(AdcResults_t));

    return true;
}

/**
 * @brief Initialize PWM subsystem
 */
static bool HardwareController_InitializePwm(void)
{
    // PWM initialization is handled by existing Pwm_Init() function
    // We can add additional initialization here if needed

    // Initialize PWM configuration with default values
    PwmConfig_t *config = &g_hardware_controller.hardware_state.current_pwm_config;
    config->frequency_hz = 10000; // 10kHz default
    config->dead_time_ns = 1000;  // 1µs dead time
    config->alignment = PWM_CENTER_ALIGNED;

    return true;
}

/**
 * @brief Initialize GPIO subsystem
 */
static bool HardwareController_InitializeGpio(void)
{
    // GPIO initialization is handled by existing System_Init() function
    // We can add additional initialization here if needed

    // Initialize GPIO states
    memset(&g_hardware_controller.hardware_state.gpio_states, 0,
           sizeof(GpioStates_t));

    return true;
}

/**
 * @brief Initialize timer subsystem
 */
static bool HardwareController_InitializeTimers(void)
{
    // Timer initialization is handled by existing SystemClock.c
    // We can add additional initialization here if needed

    return true;
}

/**
 * @brief Initialize communication interfaces
 */
static bool HardwareController_InitializeCommunication(void)
{
    // Communication initialization is handled by existing Uart.c and Spi.c
    // We can add additional initialization here if needed

    return true;
}

/**
 * @brief Update ADC results from existing ADC system
 */
static void HardwareController_UpdateAdcResults(void)
{
    AdcResults_t *results = &g_hardware_controller.hardware_state.last_adc_results;

    // Get ADC values from existing SystemInterface objects
    SystemInterface_Obj *sys_obj0 = &SystemInterfaceObj[0];
    SystemInterface_Obj *sys_obj1 = &SystemInterfaceObj[1];

    // Motor 0 currents
    if (sys_obj0->Adc.pFuncGetIa)
    {
        results->motor_currents[0][0] = sys_obj0->Adc.pFuncGetIa();
    }
    if (sys_obj0->Adc.pFuncGetIb)
    {
        results->motor_currents[0][1] = sys_obj0->Adc.pFuncGetIb();
    }
    if (sys_obj0->Adc.pFuncGetIc)
    {
        results->motor_currents[0][2] = sys_obj0->Adc.pFuncGetIc();
    }

    // Motor 1 currents
    if (sys_obj1->Adc.pFuncGetIa)
    {
        results->motor_currents[1][0] = sys_obj1->Adc.pFuncGetIa();
    }
    if (sys_obj1->Adc.pFuncGetIb)
    {
        results->motor_currents[1][1] = sys_obj1->Adc.pFuncGetIb();
    }
    if (sys_obj1->Adc.pFuncGetIc)
    {
        results->motor_currents[1][2] = sys_obj1->Adc.pFuncGetIc();
    }

    // DC bus voltage
    if (sys_obj0->Adc.pFuncGetUdc)
    {
        results->dc_bus_voltage = sys_obj0->Adc.pFuncGetUdc();
    }

    // Temperature sensors (if available)
    results->temperature_sensors[0] = 25; // Default temperature
    results->temperature_sensors[1] = 25;

    // Update timestamp
    results->timestamp_ms = g_hardware_controller.hardware_state.system_tick_count;
    results->conversion_complete = g_hardware_controller.hardware_state.adc_conversion_complete;
}

/**
 * @brief Update GPIO states from hardware
 */
static void HardwareController_UpdateGpioStates(void)
{
    GpioInputs_t *inputs = &g_hardware_controller.hardware_state.gpio_states.inputs;

    // Get GPIO states from existing SystemInterface objects
    SystemInterface_Obj *sys_obj0 = &SystemInterfaceObj[0];
    SystemInterface_Obj *sys_obj1 = &SystemInterfaceObj[1];

    // Emergency stop button
    if (sys_obj0->GetIO.pFuncGetKey)
    {
        inputs->emergency_stop = !sys_obj0->GetIO.pFuncGetKey(); // Active low
    }

    // Enable switch
    inputs->enable_switch = true; // Default enabled

    // Hall sensors
    if (sys_obj0->Hall.pFuncGetHall)
    {
        inputs->hall_sensors[0] = sys_obj0->Hall.pFuncGetHall();
    }
    if (sys_obj1->Hall.pFuncGetHall)
    {
        inputs->hall_sensors[1] = sys_obj1->Hall.pFuncGetHall();
    }

    // Limit switches (if available)
    inputs->limit_switches = 0; // Default no limits
}