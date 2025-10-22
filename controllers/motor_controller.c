/**
 * @file motor_controller.c
 * @brief Motor Control Subsystem Implementation
 * @author Enhanced by AI Assistant
 * @version v2.0.0
 * @date 2025
 *
 * Motor control subsystem that manages all motor operations including
 * FOC control, Hall sensor processing, and motor state management.
 * Integrates with existing motor control code while providing clean API.
 */

#include "controllers/motor_controller.h"
#include "SystemDefine.h"
#include "MotorDrive.h"
#include "Hall.h"
#include "Svpwm.h"
#include "CurrentLoop.h"
#include "SpeedCtrl.h"
#include "SystemInterface.h"
#include <string.h>

/* =============================================================================
 * PRIVATE VARIABLES AND STATE
 * ============================================================================= */

/**
 * @brief Motor controller instance data
 */
typedef struct
{
    MotorParameters_t parameters;
    MotorControlConfig_t config;
    MotorState_t current_state;
    MotorFeedback_t feedback;
    MotorFaultFlags_t fault_flags;
    bool initialized;
    bool enabled;
    uint32_t state_change_time_ms;
} MotorControllerInstance_t;

/**
 * @brief Motor controller global state
 */
typedef struct
{
    MotorControllerInstance_t motors[MOTOR_MAX_COUNT];
    bool system_initialized;

    struct
    {
        MotorStateChangeCallback_t state_callback;
        MotorFaultCallback_t fault_callback;
    } callbacks;

    struct
    {
        uint32_t control_loop_count;
        uint32_t fault_check_count;
        uint32_t last_update_ms;
    } statistics;

} MotorControllerState_t;

/** Global motor controller state */
static MotorControllerState_t g_motor_controller = {0};

/** External references to existing motor objects */
extern Motor_Obj motor_I[2];
extern SystemInterface_Obj SystemInterfaceObj[2];

/* =============================================================================
 * PRIVATE FUNCTION DECLARATIONS
 * ============================================================================= */

static bool MotorController_InitializeMotorHardware(uint8_t motor_id);
static void MotorController_UpdateMotorState(uint8_t motor_id);
static void MotorController_UpdateMotorFeedback(uint8_t motor_id);
static void MotorController_CheckMotorFaults(uint8_t motor_id);
static bool MotorController_ValidateParameters(const MotorParameters_t *parameters);
static bool MotorController_ValidateConfig(const MotorControlConfig_t *config);
static void MotorController_ApplyParameters(uint8_t motor_id, const MotorParameters_t *parameters);
static void MotorController_ApplyConfig(uint8_t motor_id, const MotorControlConfig_t *config);
static void MotorController_NotifyStateChange(uint8_t motor_id, MotorState_t old_state, MotorState_t new_state);

/* =============================================================================
 * PUBLIC FUNCTION IMPLEMENTATIONS
 * ============================================================================= */

/**
 * @brief Initialize motor controller subsystem
 * @return bool True if initialization successful
 */
bool MotorController_Initialize(void)
{
    // Initialize controller state
    memset(&g_motor_controller, 0, sizeof(MotorControllerState_t));

    // Initialize each motor instance
    for (uint8_t motor_id = 0; motor_id < MOTOR_MAX_COUNT; motor_id++)
    {
        MotorControllerInstance_t *motor = &g_motor_controller.motors[motor_id];

        motor->current_state = MOTOR_STATE_IDLE;
        motor->initialized = false;
        motor->enabled = false;

        // Initialize hardware for this motor
        if (!MotorController_InitializeMotorHardware(motor_id))
        {
            return false;
        }

        motor->initialized = true;
    }

    // Initialize existing motor drive system (integrates with legacy code)
    MotorDrive_Init(&SystemInterfaceObj[0], &SystemInterfaceObj[1]);

    g_motor_controller.system_initialized = true;
    return true;
}

/**
 * @brief Configure motor parameters and control settings
 * @param motor_id Motor identifier (0 or 1)
 * @param parameters Motor parameters structure
 * @param config Control configuration structure
 * @return bool True if configuration successful
 */
bool MotorController_Configure(uint8_t motor_id,
                               const MotorParameters_t *parameters,
                               const MotorControlConfig_t *config)
{
    if (!MOTOR_IS_VALID_ID(motor_id) || !parameters || !config)
    {
        return false;
    }

    if (!g_motor_controller.system_initialized)
    {
        return false;
    }

    // Validate parameters
    if (!MotorController_ValidateParameters(parameters) ||
        !MotorController_ValidateConfig(config))
    {
        return false;
    }

    MotorControllerInstance_t *motor = &g_motor_controller.motors[motor_id];

    // Store configuration
    memcpy(&motor->parameters, parameters, sizeof(MotorParameters_t));
    memcpy(&motor->config, config, sizeof(MotorControlConfig_t));

    // Apply configuration to existing motor objects
    MotorController_ApplyParameters(motor_id, parameters);
    MotorController_ApplyConfig(motor_id, config);

    return true;
}

/**
 * @brief Execute motor control command
 * @param motor_id Motor identifier (0 or 1)
 * @param command Motor command structure
 * @return bool True if command executed successfully
 */
bool MotorController_ExecuteCommand(uint8_t motor_id, const MotorCommand_t *command)
{
    if (!MOTOR_IS_VALID_ID(motor_id) || !command)
    {
        return false;
    }

    MotorControllerInstance_t *motor = &g_motor_controller.motors[motor_id];

    if (!motor->initialized)
    {
        return false;
    }

    // Handle emergency stop
    if (command->emergency_stop)
    {
        MotorController_EmergencyStopAll();
        return true;
    }

    // Handle enable/disable
    if (command->enable != motor->enabled)
    {
        if (command->enable)
        {
            // Enable motor
            MotorState_t old_state = motor->current_state;
            motor->current_state = MOTOR_STATE_STARTING;
            motor->enabled = true;

            // Use existing motor control functions
            StartStopControl(motor_id, 1);

            MotorController_NotifyStateChange(motor_id, old_state, motor->current_state);
        }
        else
        {
            // Disable motor
            MotorState_t old_state = motor->current_state;
            motor->current_state = MOTOR_STATE_STOPPING;
            motor->enabled = false;

            // Use existing motor control functions
            StartStopControl(motor_id, 0);

            MotorController_NotifyStateChange(motor_id, old_state, motor->current_state);
        }
    }

    // Handle target value setting
    if (motor->enabled)
    {
        switch (motor->config.control_mode)
        {
        case MOTOR_CONTROL_SPEED:
            SpeedControl(motor_id, command->target_value);
            break;

        case MOTOR_CONTROL_TORQUE:
            // Implement torque control using existing current loop
            // Convert torque percentage to current command
            int32_t torque_current = (command->target_value * motor->parameters.max_current_ma) / 100;
            motor_I[motor_id].CurLoop.IqTarget = torque_current;
            break;

        default:
            return false;
        }
    }

    return true;
}

/**
 * @brief Get motor feedback data
 * @param motor_id Motor identifier (0 or 1)
 * @param feedback Pointer to feedback structure to populate
 * @return bool True if feedback data retrieved successfully
 */
bool MotorController_GetFeedback(uint8_t motor_id, MotorFeedback_t *feedback)
{
    if (!MOTOR_IS_VALID_ID(motor_id) || !feedback)
    {
        return false;
    }

    MotorControllerInstance_t *motor = &g_motor_controller.motors[motor_id];

    if (!motor->initialized)
    {
        return false;
    }

    // Update feedback from existing motor objects
    MotorController_UpdateMotorFeedback(motor_id);

    // Copy feedback data
    memcpy(feedback, &motor->feedback, sizeof(MotorFeedback_t));

    return true;
}

/**
 * @brief Get motor fault status
 * @param motor_id Motor identifier (0 or 1)
 * @param faults Pointer to fault flags structure to populate
 * @return bool True if fault data retrieved successfully
 */
bool MotorController_GetFaultStatus(uint8_t motor_id, MotorFaultFlags_t *faults)
{
    if (!MOTOR_IS_VALID_ID(motor_id) || !faults)
    {
        return false;
    }

    MotorControllerInstance_t *motor = &g_motor_controller.motors[motor_id];

    // Update fault status from existing motor objects
    MotorController_CheckMotorFaults(motor_id);

    // Copy fault flags
    memcpy(faults, &motor->fault_flags, sizeof(MotorFaultFlags_t));

    return true;
}

/**
 * @brief Clear motor faults
 * @param motor_id Motor identifier (0 or 1)
 * @return bool True if faults cleared successfully
 */
bool MotorController_ClearFaults(uint8_t motor_id)
{
    if (!MOTOR_IS_VALID_ID(motor_id))
    {
        return false;
    }

    MotorControllerInstance_t *motor = &g_motor_controller.motors[motor_id];

    // Clear fault flags
    motor->fault_flags.all = 0;

    // Clear faults in existing motor objects
    motor_I[motor_id].Flag.Arr[0] = 0;

    // If motor was in fault state, transition to idle
    if (motor->current_state == MOTOR_STATE_FAULT)
    {
        MotorState_t old_state = motor->current_state;
        motor->current_state = MOTOR_STATE_IDLE;
        MotorController_NotifyStateChange(motor_id, old_state, motor->current_state);
    }

    return true;
}

/**
 * @brief Emergency stop all motors
 */
void MotorController_EmergencyStopAll(void)
{
    for (uint8_t motor_id = 0; motor_id < MOTOR_MAX_COUNT; motor_id++)
    {
        MotorControllerInstance_t *motor = &g_motor_controller.motors[motor_id];

        if (motor->initialized)
        {
            MotorState_t old_state = motor->current_state;
            motor->current_state = MOTOR_STATE_FAULT;
            motor->enabled = false;

            // Use existing emergency stop functions
            StartStopControl(motor_id, 0);
            AllPwmShut(motor_id);

            MotorController_NotifyStateChange(motor_id, old_state, motor->current_state);
        }
    }
}

/**
 * @brief Update motor control algorithms
 * @details Called from high-frequency interrupt (10kHz)
 */
void MotorController_UpdateControlLoops(void)
{
    if (!g_motor_controller.system_initialized)
    {
        return;
    }

    // Update control loop statistics
    g_motor_controller.statistics.control_loop_count++;

    // The existing FOC control is handled by MotorPwm_Isr_I function
    // We can add additional control logic here if needed

    // Update motor states based on current operation
    for (uint8_t motor_id = 0; motor_id < MOTOR_MAX_COUNT; motor_id++)
    {
        MotorController_UpdateMotorState(motor_id);
    }
}

/**
 * @brief Monitor motor conditions and detect faults
 */
void MotorController_MonitorConditions(void)
{
    if (!g_motor_controller.system_initialized)
    {
        return;
    }

    g_motor_controller.statistics.fault_check_count++;
    g_motor_controller.statistics.last_update_ms = HardwareController_GetTimestampMs();

    // Check each motor for fault conditions
    for (uint8_t motor_id = 0; motor_id < MOTOR_MAX_COUNT; motor_id++)
    {
        MotorController_CheckMotorFaults(motor_id);
        MotorController_UpdateMotorFeedback(motor_id);
    }
}

/**
 * @brief Set motor control mode
 * @param motor_id Motor identifier (0 or 1)
 * @param mode New control mode
 * @return bool True if mode set successfully
 */
bool MotorController_SetControlMode(uint8_t motor_id, MotorControlMode_t mode)
{
    if (!MOTOR_IS_VALID_ID(motor_id))
    {
        return false;
    }

    MotorControllerInstance_t *motor = &g_motor_controller.motors[motor_id];

    if (!motor->initialized)
    {
        return false;
    }

    motor->config.control_mode = mode;
    return true;
}

/**
 * @brief Get motor controller state
 * @param motor_id Motor identifier (0 or 1)
 * @return MotorState_t Current motor state
 */
MotorState_t MotorController_GetState(uint8_t motor_id)
{
    if (!MOTOR_IS_VALID_ID(motor_id))
    {
        return MOTOR_STATE_FAULT;
    }

    return g_motor_controller.motors[motor_id].current_state;
}

/**
 * @brief Enable/disable motor
 * @param motor_id Motor identifier (0 or 1)
 * @param enable Enable flag
 * @return bool True if command successful
 */
bool MotorController_Enable(uint8_t motor_id, bool enable)
{
    MotorCommand_t command = {0};
    command.enable = enable;
    command.target_value = 0;
    command.direction = MOTOR_DIRECTION_FORWARD;

    return MotorController_ExecuteCommand(motor_id, &command);
}

/**
 * @brief Set motor speed target
 * @param motor_id Motor identifier (0 or 1)
 * @param speed_rpm Target speed in RPM
 * @return bool True if command successful
 */
bool MotorController_SetSpeed(uint8_t motor_id, int32_t speed_rpm)
{
    if (!MOTOR_IS_VALID_SPEED(speed_rpm))
    {
        return false;
    }

    MotorCommand_t command = {0};
    command.enable = true;
    command.target_value = speed_rpm;
    command.direction = (speed_rpm >= 0) ? MOTOR_DIRECTION_FORWARD : MOTOR_DIRECTION_REVERSE;

    return MotorController_ExecuteCommand(motor_id, &command);
}

/**
 * @brief Set motor torque target
 * @param motor_id Motor identifier (0 or 1)
 * @param torque_percent Target torque in percentage (-100 to +100)
 * @return bool True if command successful
 */
bool MotorController_SetTorque(uint8_t motor_id, int32_t torque_percent)
{
    if (torque_percent < -100 || torque_percent > 100)
    {
        return false;
    }

    MotorCommand_t command = {0};
    command.enable = true;
    command.target_value = torque_percent;
    command.direction = (torque_percent >= 0) ? MOTOR_DIRECTION_FORWARD : MOTOR_DIRECTION_REVERSE;

    return MotorController_ExecuteCommand(motor_id, &command);
}

/* =============================================================================
 * CALLBACK REGISTRATION FUNCTIONS
 * ============================================================================= */

/**
 * @brief Register motor state change callback
 */
bool MotorController_RegisterStateCallback(MotorStateChangeCallback_t callback)
{
    g_motor_controller.callbacks.state_callback = callback;
    return true;
}

/**
 * @brief Register motor fault callback
 */
bool MotorController_RegisterFaultCallback(MotorFaultCallback_t callback)
{
    g_motor_controller.callbacks.fault_callback = callback;
    return true;
}

/* =============================================================================
 * PRIVATE FUNCTION IMPLEMENTATIONS
 * ============================================================================= */

/**
 * @brief Initialize motor hardware
 */
static bool MotorController_InitializeMotorHardware(uint8_t motor_id)
{
    // Motor hardware initialization is handled by existing System_Init()
    // We can add motor-specific initialization here if needed
    return true;
}

/**
 * @brief Update motor state based on current conditions
 */
static void MotorController_UpdateMotorState(uint8_t motor_id)
{
    MotorControllerInstance_t *motor = &g_motor_controller.motors[motor_id];
    MotorState_t old_state = motor->current_state;
    MotorState_t new_state = old_state;

    // Get current motor status from existing motor objects
    Motor_Obj *motor_obj = &motor_I[motor_id];

    // Determine new state based on motor object state and flags
    if (motor_obj->Flag.Bits.SwOverCurrent || motor_obj->Flag.Bits.OverVolatage ||
        motor_obj->Flag.Bits.LackVolatage)
    {
        new_state = MOTOR_STATE_FAULT;
    }
    else if (motor->enabled && motor_obj->Flag.Bits.MotorStarted)
    {
        new_state = MOTOR_STATE_RUNNING;
    }
    else if (motor->enabled && !motor_obj->Flag.Bits.MotorStarted)
    {
        new_state = MOTOR_STATE_STARTING;
    }
    else
    {
        new_state = MOTOR_STATE_IDLE;
    }

    // Update state if changed
    if (new_state != old_state)
    {
        motor->current_state = new_state;
        motor->state_change_time_ms = HardwareController_GetTimestampMs();
        MotorController_NotifyStateChange(motor_id, old_state, new_state);
    }
}

/**
 * @brief Update motor feedback data from existing motor objects
 */
static void MotorController_UpdateMotorFeedback(uint8_t motor_id)
{
    MotorControllerInstance_t *motor = &g_motor_controller.motors[motor_id];
    Motor_Obj *motor_obj = &motor_I[motor_id];

    // Update feedback data from existing motor objects
    motor->feedback.motor_id = motor_id;
    motor->feedback.actual_speed_rpm = motor_obj->DataObj.MotorSpeed;
    motor->feedback.target_speed_rpm = motor_obj->SpeedObj.Speed_Target;

    // Phase currents
    motor->feedback.phase_currents[0] = motor_obj->DataObj.Ia_Q15;
    motor->feedback.phase_currents[1] = motor_obj->DataObj.Ib_Q15;
    motor->feedback.phase_currents[2] = motor_obj->DataObj.Ic_Q15;

    motor->feedback.dc_bus_voltage = motor_obj->DataObj.Udc_Real;
    motor->feedback.rotor_angle = motor_obj->BrushlessObj.EleAng_Q15;
    motor->feedback.current_state = motor->current_state;

// Hall sensor state (if available)
#ifdef HALL_FOR_ANGLE_T
    motor->feedback.hall_state = GetIO_GetHall(motor_id);
#endif

    // Motor temperature (placeholder - implement if temperature sensor available)
    motor->feedback.motor_temperature = 25; // Default room temperature
}

/**
 * @brief Check motor fault conditions
 */
static void MotorController_CheckMotorFaults(uint8_t motor_id)
{
    MotorControllerInstance_t *motor = &g_motor_controller.motors[motor_id];
    Motor_Obj *motor_obj = &motor_I[motor_id];

    MotorFaultFlags_t old_faults = motor->fault_flags;
    motor->fault_flags.all = 0;

    // Check fault flags from existing motor objects
    if (motor_obj->Flag.Bits.SwOverCurrent)
    {
        motor->fault_flags.bits.overcurrent = 1;
    }

    if (motor_obj->Flag.Bits.OverVolatage)
    {
        motor->fault_flags.bits.overvoltage = 1;
    }

    if (motor_obj->Flag.Bits.LackVolatage)
    {
        motor->fault_flags.bits.undervoltage = 1;
    }

    // Add additional fault checks here
    // Check for stall condition
    if (motor->enabled && motor_obj->Flag.Bits.MotorStarted)
    {
        uint32_t current_time = HardwareController_GetTimestampMs();
        if (motor->feedback.actual_speed_rpm < 50 &&
            (current_time - motor->state_change_time_ms) > 2000)
        {
            motor->fault_flags.bits.stall_detected = 1;
        }
    }

    // Notify if new faults detected
    if (motor->fault_flags.all != old_faults.all &&
        g_motor_controller.callbacks.fault_callback)
    {
        g_motor_controller.callbacks.fault_callback(motor_id, motor->fault_flags);
    }
}

/**
 * @brief Validate motor parameters
 */
static bool MotorController_ValidateParameters(const MotorParameters_t *parameters)
{
    if (!parameters)
        return false;

    return (parameters->pole_pairs > 0 && parameters->pole_pairs <= 50) &&
           (parameters->resistance_ohms > 0 && parameters->resistance_ohms < 100) &&
           (parameters->inductance_henry > 0 && parameters->inductance_henry < 1) &&
           (parameters->max_speed_rpm > 0 && parameters->max_speed_rpm <= MOTOR_MAX_SPEED_RPM) &&
           (parameters->max_current_ma > 0 && parameters->max_current_ma <= 50000);
}

/**
 * @brief Validate motor control configuration
 */
static bool MotorController_ValidateConfig(const MotorControlConfig_t *config)
{
    if (!config)
        return false;

    return (config->speed_controller.kp > 0) &&
           (config->speed_controller.ki >= 0) &&
           (config->current_controller.kp > 0) &&
           (config->current_controller.ki >= 0) &&
           (config->pwm_config.pwm_frequency_hz >= 1000 &&
            config->pwm_config.pwm_frequency_hz <= 100000);
}

/**
 * @brief Apply motor parameters to existing motor objects
 */
static void MotorController_ApplyParameters(uint8_t motor_id, const MotorParameters_t *parameters)
{
    Motor_Obj *motor_obj = &motor_I[motor_id];

    // Apply parameters to existing motor object structure
    motor_obj->Plate.Motor_pole = parameters->pole_pairs;
    // Add other parameter mappings as needed
}

/**
 * @brief Apply control configuration to existing motor objects
 */
static void MotorController_ApplyConfig(uint8_t motor_id, const MotorControlConfig_t *config)
{
    Motor_Obj *motor_obj = &motor_I[motor_id];

    // Apply control configuration to existing motor object structure
    motor_obj->MotorBaseObj.Fun_SpdKp = config->speed_controller.kp;
    motor_obj->MotorBaseObj.Fun_SpdKi = config->speed_controller.ki;
    motor_obj->MotorBaseObj.Fun_DKp_Q13 = config->current_controller.kp;
    motor_obj->MotorBaseObj.Fun_DKi_Q15 = config->current_controller.ki;
    motor_obj->MotorBaseObj.Fun_QKp_Q13 = config->current_controller.kp;
    motor_obj->MotorBaseObj.Fun_QKi_Q15 = config->current_controller.ki;
    motor_obj->Plate.Pwm_freq = config->pwm_config.pwm_frequency_hz;
}

/**
 * @brief Notify state change callback
 */
static void MotorController_NotifyStateChange(uint8_t motor_id, MotorState_t old_state, MotorState_t new_state)
{
    if (g_motor_controller.callbacks.state_callback)
    {
        g_motor_controller.callbacks.state_callback(motor_id, old_state, new_state);
    }
}