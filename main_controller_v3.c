/**
 * @file main_controller_v3.c
 * @brief Main System Controller - Modular Architecture Implementation
 * @author Enhanced by AI Assistant
 * @version v3.0.0
 * @date 2025
 *
 * Redesigned main controller that coordinates multiple specialized sub-controllers
 * for better modularity, scalability, and maintainability. Follows DRY principle
 * and enables easy system extension.
 */

#include "main_controller.h"
#include "SystemDefine.h"
#include <string.h>

/* =============================================================================
 * GLOBAL VARIABLES AND STATE MANAGEMENT
 * ============================================================================= */

/**
 * @brief Main system controller state structure (simplified)
 */
typedef struct
{
    SystemState_t current_state; /**< Current system operational state */
    SystemMode_t operating_mode; /**< Current operating mode */

    struct
    {
        uint32_t boot_time_ms;   /**< System boot timestamp */
        uint32_t uptime_ms;      /**< System uptime counter */
        uint32_t last_update_ms; /**< Last update timestamp */
    } timing;

    struct
    {
        bool system_initialized;    /**< System initialization complete */
        bool sub_controllers_ready; /**< All sub-controllers ready */
        bool emergency_stop_active; /**< Emergency stop state */
    } status;

    struct
    {
        MotorStateChangeCallback_t motor_state_callback;
        SafetyStateChangeCallback_t safety_state_callback;
        MessageReceivedCallback_t comm_message_callback;
        FaultDetectionCallback_t fault_callback;
    } callbacks;

} MainController_t;

/** Global main controller instance */
static MainController_t g_main_controller = {0};

/* =============================================================================
 * PRIVATE FUNCTION DECLARATIONS
 * ============================================================================= */

static bool MainController_InitializeSubControllers(void);
static void MainController_UpdateSubControllers(void);
static void MainController_HandleSubControllerCallbacks(void);
static void MainController_CoordinateMotors(void);
static void MainController_ProcessSystemCommands(void);
static void MainController_UpdateSystemState(void);

/* Sub-controller callback handlers */
static void MainController_OnMotorStateChange(uint8_t motor_id, MotorState_t old_state, MotorState_t new_state);
static void MainController_OnSafetyStateChange(SafetyState_t old_state, SafetyState_t new_state);
static void MainController_OnCommunicationMessage(CommInterface_t interface, const CommMessage_t *message);
static void MainController_OnFaultDetected(const FaultInfo_t *fault_info);

/* =============================================================================
 * PUBLIC FUNCTION IMPLEMENTATIONS
 * ============================================================================= */

/**
 * @brief Initialize the main system controller
 * @details Performs modular initialization of all sub-controllers
 * @return SystemResult_t SUCCESS if initialization completed successfully
 */
SystemResult_t SystemController_Initialize(void)
{
    // Initialize main controller state
    memset(&g_main_controller, 0, sizeof(MainController_t));
    g_main_controller.current_state = SYSTEM_STATE_INITIALIZING;
    g_main_controller.operating_mode = SYSTEM_MODE_STANDBY;
    g_main_controller.timing.boot_time_ms = HardwareController_GetTimestampMs();

    // Initialize sub-controllers in proper order
    if (!MainController_InitializeSubControllers())
    {
        g_main_controller.current_state = SYSTEM_STATE_FAULT;
        return SYSTEM_ERROR_HARDWARE;
    }

    // Register callbacks with sub-controllers
    MainController_HandleSubControllerCallbacks();

    // Mark system as initialized
    g_main_controller.status.system_initialized = true;
    g_main_controller.status.sub_controllers_ready = true;
    g_main_controller.current_state = SYSTEM_STATE_READY;

    return SYSTEM_SUCCESS;
}

/**
 * @brief Main system controller execution loop
 * @details Coordinates all sub-controllers and manages system state
 */
void SystemController_Execute(void)
{
    // Ensure system is properly initialized
    if (!g_main_controller.status.system_initialized)
    {
        return;
    }

    // Update timing information
    g_main_controller.timing.uptime_ms =
        HardwareController_GetTimestampMs() - g_main_controller.timing.boot_time_ms;
    g_main_controller.timing.last_update_ms = HardwareController_GetTimestampMs();

    // Update all sub-controllers (they handle their own responsibilities)
    MainController_UpdateSubControllers();

    // Coordinate motor operations based on current mode
    MainController_CoordinateMotors();

    // Process any system-level commands
    MainController_ProcessSystemCommands();

    // Update main system state based on sub-controller states
    MainController_UpdateSystemState();

    // Feed safety watchdog to prevent timeout
    SafetyController_FeedWatchdog();
}

/**
 * @brief Set system operating mode
 * @param mode New operating mode to set
 * @return SystemResult_t SUCCESS if mode change is valid
 */
SystemResult_t SystemController_SetMode(SystemMode_t mode)
{
    // Check if system is in valid state for mode change
    SafetyState_t safety_state = SafetyController_GetState();
    if (safety_state >= SAFETY_STATE_FAULT)
    {
        return SYSTEM_ERROR_FAULT_ACTIVE;
    }

    // Validate mode transition
    switch (mode)
    {
    case SYSTEM_MODE_STANDBY:
        // Stop all motors safely
        MotorController_Enable(0, false);
        MotorController_Enable(1, false);
        break;

    case SYSTEM_MODE_SPEED_CONTROL:
        // Configure both motors for speed control
        MotorController_SetControlMode(0, MOTOR_CONTROL_SPEED);
        MotorController_SetControlMode(1, MOTOR_CONTROL_SPEED);
        break;

    case SYSTEM_MODE_TORQUE_CONTROL:
        // Configure both motors for torque control
        MotorController_SetControlMode(0, MOTOR_CONTROL_TORQUE);
        MotorController_SetControlMode(1, MOTOR_CONTROL_TORQUE);
        break;

    case SYSTEM_MODE_DUAL_MOTOR_SYNC:
        // Configure synchronized operation
        MotorController_SetControlMode(0, MOTOR_CONTROL_SPEED);
        MotorController_SetControlMode(1, MOTOR_CONTROL_SPEED);
        break;

    default:
        return SYSTEM_ERROR_INVALID_PARAMETER;
    }

    g_main_controller.operating_mode = mode;
    return SYSTEM_SUCCESS;
}

/**
 * @brief Control individual motor operation
 * @param motor_id Motor identifier (0 or 1)
 * @param enable Enable/disable flag
 * @param target_value Target speed (RPM) or torque based on current mode
 * @return SystemResult_t SUCCESS if command executed successfully
 */
SystemResult_t SystemController_ControlMotor(uint8_t motor_id, bool enable, int32_t target_value)
{
    // Validate parameters
    if (!MOTOR_IS_VALID_ID(motor_id))
    {
        return SYSTEM_ERROR_INVALID_PARAMETER;
    }

    // Check safety state
    SafetyState_t safety_state = SafetyController_GetState();
    if (safety_state >= SAFETY_STATE_FAULT && enable)
    {
        return SYSTEM_ERROR_FAULT_ACTIVE;
    }

    // Create motor command based on operating mode
    MotorCommand_t command = {0};
    command.enable = enable;
    command.target_value = target_value;
    command.direction = (target_value >= 0) ? MOTOR_DIRECTION_FORWARD : MOTOR_DIRECTION_REVERSE;
    command.emergency_stop = g_main_controller.status.emergency_stop_active;

    // Execute command through motor controller
    bool success = MotorController_ExecuteCommand(motor_id, &command);

    return success ? SYSTEM_SUCCESS : SYSTEM_ERROR_HARDWARE;
}

/**
 * @brief Trigger emergency stop for all motors
 */
void SystemController_EmergencyStop(void)
{
    g_main_controller.status.emergency_stop_active = true;
    g_main_controller.current_state = SYSTEM_STATE_FAULT;

    // Trigger emergency stop in all relevant sub-controllers
    MotorController_EmergencyStopAll();
    SafetyController_ActivateEmergencyStop();
    HardwareController_EmergencyDisablePwm();

    // Send emergency stop notification
    CommunicationManager_SendFaultNotification(FAULT_SAFETY_EMERGENCY_STOP, 0xFF);
}

/**
 * @brief Get current system status information
 * @param status Pointer to status structure to populate
 * @return SystemResult_t SUCCESS if status retrieved successfully
 */
SystemResult_t SystemController_GetStatus(SystemStatus_t *status)
{
    if (status == NULL)
    {
        return SYSTEM_ERROR_INVALID_PARAMETER;
    }

    // Populate basic system status
    status->current_state = g_main_controller.current_state;
    status->operating_mode = g_main_controller.operating_mode;
    status->uptime_ms = g_main_controller.timing.uptime_ms;
    status->emergency_stop_active = g_main_controller.status.emergency_stop_active;

    // Get motor feedback from motor controller
    MotorFeedback_t motor_feedback;

    if (MotorController_GetFeedback(0, &motor_feedback))
    {
        status->motor1_enabled = (MotorController_GetState(0) == MOTOR_STATE_RUNNING);
        status->motor1_actual_speed = motor_feedback.actual_speed_rpm;
        status->motor1_target_speed = motor_feedback.target_speed_rpm;
        status->motor1_current = motor_feedback.phase_currents[0]; // Phase A current
        status->system_voltage = motor_feedback.dc_bus_voltage;
    }

    if (MotorController_GetFeedback(1, &motor_feedback))
    {
        status->motor2_enabled = (MotorController_GetState(1) == MOTOR_STATE_RUNNING);
        status->motor2_actual_speed = motor_feedback.actual_speed_rpm;
        status->motor2_target_speed = motor_feedback.target_speed_rpm;
        status->motor2_current = motor_feedback.phase_currents[0]; // Phase A current
    }

    // Get fault count from safety controller
    status->fault_count = SafetyController_GetActiveFaultCount();

    return SYSTEM_SUCCESS;
}

/**
 * @brief Get system uptime in milliseconds
 * @return uint32_t System uptime in milliseconds
 */
uint32_t SystemController_GetUptime(void)
{
    return g_main_controller.timing.uptime_ms;
}

/**
 * @brief Check if system is in fault state
 * @return bool True if system has active faults
 */
bool SystemController_HasFaults(void)
{
    return (g_main_controller.current_state == SYSTEM_STATE_FAULT) ||
           (SafetyController_GetActiveFaultCount() > 0);
}

/* =============================================================================
 * PRIVATE FUNCTION IMPLEMENTATIONS
 * ============================================================================= */

/**
 * @brief Initialize all sub-controllers in proper order
 * @return bool True if all sub-controllers initialized successfully
 */
static bool MainController_InitializeSubControllers(void)
{
    // 1. Initialize hardware controller first (lowest level)
    HardwareConfig_t hw_config = {
        .clocks = {
            .system_clock_hz = HW_DEFAULT_SYSTEM_CLOCK_HZ,
            .pwm_frequency_hz = HW_DEFAULT_PWM_FREQUENCY_HZ,
            .adc_clock_hz = 12000000, // 12 MHz ADC clock
        },
        .adc_config = {
            .continuous_mode = true,
            .dma_enabled = true,
            .sample_time = 3, // 13.5 cycles
            .injected_mode = true,
        },
        .pwm_config = {
            .complementary_mode = true,
            .center_aligned = true,
            .resolution_bits = 12,
            .fault_protection = true,
        }};

    if (!HardwareController_Initialize(&hw_config))
    {
        return false;
    }

    // 2. Initialize safety controller (monitors hardware)
    SafetyParameters_t safety_params = {
        .electrical = {
            .overcurrent_limit_ma = SAFETY_DEFAULT_OVERCURRENT_MA,
            .overcurrent_time_ms = 100,
            .overvoltage_limit_mv = SAFETY_DEFAULT_OVERVOLTAGE_MV,
            .undervoltage_limit_mv = SAFETY_DEFAULT_UNDERVOLTAGE_MV,
            .voltage_time_ms = 500,
        },
        .thermal = {
            .overtemp_limit_c = 85,
            .overtemp_time_ms = 1000,
            .temp_hysteresis_c = 5,
        },
        .mechanical = {
            .max_speed_rpm = MOTOR_MAX_SPEED_RPM,
            .overspeed_time_ms = 200,
            .stall_current_ma = 12000,
            .stall_time_ms = 2000,
        },
        .communication = {
            .watchdog_timeout_ms = 5000,
            .comm_timeout_ms = 2000,
            .heartbeat_timeout_ms = 10000,
        }};

    if (!SafetyController_Initialize(&safety_params))
    {
        return false;
    }

    // 3. Initialize motor controller (uses hardware and safety)
    if (!MotorController_Initialize())
    {
        return false;
    }

    // Configure motor parameters
    MotorParameters_t motor_params = {
        .pole_pairs = 2,               // MOTOR_POPAIRS1 from UserParam.h
        .resistance_ohms = 0.458f,     // RS1/2 from UserParam.h
        .inductance_henry = 0.000925f, // LD1 from UserParam.h
        .flux_linkage = 0.01374f,      // FI1 from UserParam.h
        .max_speed_rpm = 5000,
        .max_current_ma = 15000,
        .encoder_resolution = 4096,
    };

    MotorControlConfig_t motor_config = {
        .control_mode = MOTOR_CONTROL_SPEED,
        .direction = MOTOR_DIRECTION_FORWARD,
        .speed_controller = {
            .kp = 2000, // SpdKp1 from UserParam.h
            .ki = 1000, // SpdKi1 from UserParam.h
            .kd = 0,
            .output_limit = 10000, // SpdOutMax1 from UserParam.h
        },
        .current_controller = {
            .kp = 2500,            // DKp1 from UserParam.h
            .ki = 1000,            // DKi1 from UserParam.h
            .output_limit = 15000, // DOutMax1 from UserParam.h
        },
        .pwm_config = {
            .pwm_frequency_hz = 20000, // PWM_FREQUENCY1 from UserParam.h
            .dead_time_ns = 1000,
            .brake_on_stop = true,
        }};

    // Configure both motors
    for (uint8_t motor_id = 0; motor_id < 2; motor_id++)
    {
        motor_params.motor_id = motor_id;
        if (!MotorController_Configure(motor_id, &motor_params, &motor_config))
        {
            return false;
        }
    }

    // 4. Initialize communication manager
    if (!CommunicationManager_Initialize())
    {
        return false;
    }

    // 5. Initialize diagnostics system
    if (!SystemDiagnostics_Initialize())
    {
        return false;
    }

    return true;
}

/**
 * @brief Update all sub-controllers
 */
static void MainController_UpdateSubControllers(void)
{
    // Update hardware controller (reads ADC, updates GPIO, etc.)
    // Hardware controller is typically updated by interrupts, but we can
    // call maintenance functions here if needed

    // Update safety controller with current monitoring data
    SafetyMonitoringData_t safety_data = {0};

    // Get current measurements from hardware controller
    AdcResults_t adc_results;
    if (HardwareController_GetAdcResults(&adc_results))
    {
        safety_data.measurements.current_ma[0] =
            HardwareController_AdcToCurrent(adc_results.raw_values[ADC_CHANNEL_MOTOR1_IA], 5, 10);
        safety_data.measurements.current_ma[1] =
            HardwareController_AdcToCurrent(adc_results.raw_values[ADC_CHANNEL_MOTOR2_IA], 5, 10);
        safety_data.measurements.dc_voltage_mv =
            HardwareController_AdcToVoltage(adc_results.raw_values[ADC_CHANNEL_DC_VOLTAGE]) * 26; // Voltage divider
        safety_data.measurements.temperature_c =
            HardwareController_AdcToVoltage(adc_results.raw_values[ADC_CHANNEL_TEMPERATURE]) / 10; // 10mV/°C
    }

    // Get motor status
    safety_data.motor_status.enabled[0] = (MotorController_GetState(0) == MOTOR_STATE_RUNNING);
    safety_data.motor_status.enabled[1] = (MotorController_GetState(1) == MOTOR_STATE_RUNNING);

    MotorFeedback_t feedback;
    if (MotorController_GetFeedback(0, &feedback))
    {
        safety_data.motor_status.speed_rpm[0] = feedback.actual_speed_rpm;
    }
    if (MotorController_GetFeedback(1, &feedback))
    {
        safety_data.motor_status.speed_rpm[1] = feedback.actual_speed_rpm;
    }

    // Get signal states
    safety_data.signals.emergency_stop_pressed =
        (HardwareController_GetGpioState(GPIO_PIN_EMERGENCY_STOP) == GPIO_STATE_LOW);
    safety_data.signals.enable_signal =
        (HardwareController_GetGpioState(GPIO_PIN_BUTTON_USER) == GPIO_STATE_HIGH);

    SafetyController_Update(&safety_data);

    // Update motor controller (control loops are typically interrupt-driven)
    MotorController_MonitorConditions();

    // Update communication manager
    CommunicationManager_ProcessCommands();

    // Update diagnostics system
    SystemDiagnostics_Update();
}

/**
 * @brief Register callbacks with sub-controllers
 */
static void MainController_HandleSubControllerCallbacks(void)
{
    // Register with motor controller
    MotorController_RegisterStateCallback(MainController_OnMotorStateChange);
    MotorController_RegisterFaultCallback(NULL); // Handle through safety controller

    // Register with safety controller
    SafetyController_RegisterStateCallback(MainController_OnSafetyStateChange);
    SafetyController_RegisterFaultCallback(MainController_OnFaultDetected);

    // Register with communication manager
    CommunicationManager_RegisterMessageCallback(MainController_OnCommunicationMessage);
}

/**
 * @brief Coordinate motor operations based on operating mode
 */
static void MainController_CoordinateMotors(void)
{
    if (g_main_controller.operating_mode == SYSTEM_MODE_DUAL_MOTOR_SYNC)
    {
        // Implement synchronized motor operation
        MotorFeedback_t motor1_feedback, motor2_feedback;

        if (MotorController_GetFeedback(0, &motor1_feedback) &&
            MotorController_GetFeedback(1, &motor2_feedback))
        {

            // Simple synchronization: average the target speeds
            int32_t avg_target = (motor1_feedback.target_speed_rpm + motor2_feedback.target_speed_rpm) / 2;

            // Apply synchronized target to both motors
            MotorController_SetSpeed(0, avg_target);
            MotorController_SetSpeed(1, avg_target);
        }
    }
}

/**
 * @brief Process system-level commands
 */
static void MainController_ProcessSystemCommands(void)
{
    // Check for emergency stop button
    if (HardwareController_GetGpioState(GPIO_PIN_EMERGENCY_STOP) == GPIO_STATE_LOW)
    {
        if (!g_main_controller.status.emergency_stop_active)
        {
            SystemController_EmergencyStop();
        }
    }

    // Handle other system-level logic here
}

/**
 * @brief Update main system state based on sub-controller states
 */
static void MainController_UpdateSystemState(void)
{
    SafetyState_t safety_state = SafetyController_GetState();

    // Update system state based on safety state
    switch (safety_state)
    {
    case SAFETY_STATE_SAFE:
        if (g_main_controller.current_state == SYSTEM_STATE_FAULT &&
            !g_main_controller.status.emergency_stop_active)
        {
            g_main_controller.current_state = SYSTEM_STATE_READY;
        }
        break;

    case SAFETY_STATE_WARNING:
        // Stay in current state but log warning
        break;

    case SAFETY_STATE_FAULT:
    case SAFETY_STATE_CRITICAL_FAULT:
    case SAFETY_STATE_EMERGENCY_STOP:
        g_main_controller.current_state = SYSTEM_STATE_FAULT;
        break;
    }

    // Check if any motors are running
    bool any_motor_running = (MotorController_GetState(0) == MOTOR_STATE_RUNNING) ||
                             (MotorController_GetState(1) == MOTOR_STATE_RUNNING);

    if (g_main_controller.current_state == SYSTEM_STATE_READY && any_motor_running)
    {
        g_main_controller.current_state = SYSTEM_STATE_RUNNING;
    }
    else if (g_main_controller.current_state == SYSTEM_STATE_RUNNING && !any_motor_running)
    {
        g_main_controller.current_state = SYSTEM_STATE_READY;
    }
}

/* =============================================================================
 * CALLBACK IMPLEMENTATIONS
 * ============================================================================= */

/**
 * @brief Handle motor state changes
 */
static void MainController_OnMotorStateChange(uint8_t motor_id, MotorState_t old_state, MotorState_t new_state)
{
    // Log state change
    SystemDiagnostics_LogMessage(DIAG_SEVERITY_INFO,
                                 "Motor %d state changed: %d -> %d", motor_id, old_state, new_state);

    // Handle specific state transitions
    if (new_state == MOTOR_STATE_FAULT)
    {
        // Motor fault detected - let safety controller handle it
    }

    // Call registered callback if available
    if (g_main_controller.callbacks.motor_state_callback)
    {
        g_main_controller.callbacks.motor_state_callback(motor_id, old_state, new_state);
    }
}

/**
 * @brief Handle safety state changes
 */
static void MainController_OnSafetyStateChange(SafetyState_t old_state, SafetyState_t new_state)
{
    // Log state change
    SystemDiagnostics_LogMessage(DIAG_SEVERITY_WARNING,
                                 "Safety state changed: %d -> %d", old_state, new_state);

    // Handle critical safety states
    if (new_state >= SAFETY_STATE_CRITICAL_FAULT)
    {
        SystemController_EmergencyStop();
    }

    // Call registered callback if available
    if (g_main_controller.callbacks.safety_state_callback)
    {
        g_main_controller.callbacks.safety_state_callback(old_state, new_state);
    }
}

/**
 * @brief Handle communication messages
 */
static void MainController_OnCommunicationMessage(CommInterface_t interface, const CommMessage_t *message)
{
    // Process system-level commands
    if (message->type == MSG_TYPE_MOTOR_COMMAND)
    {
        uint8_t motor_id = message->payload[0];
        uint8_t command = message->payload[1];
        int32_t value = *((int32_t *)&message->payload[2]);

        switch (command)
        {
        case 0x01: // Start motor
            SystemController_ControlMotor(motor_id, true, value);
            break;
        case 0x02: // Stop motor
            SystemController_ControlMotor(motor_id, false, 0);
            break;
        case 0xFF: // Emergency stop
            SystemController_EmergencyStop();
            break;
        }
    }

    // Call registered callback if available
    if (g_main_controller.callbacks.comm_message_callback)
    {
        g_main_controller.callbacks.comm_message_callback(interface, message);
    }
}

/**
 * @brief Handle detected faults
 */
static void MainController_OnFaultDetected(const FaultInfo_t *fault_info)
{
    // Log fault
    SystemDiagnostics_LogMessage(DIAG_SEVERITY_ERROR,
                                 "Fault detected: Code=0x%04X, Motor=%d, Severity=%d",
                                 fault_info->fault_code, fault_info->motor_id, fault_info->severity);

    // Send fault notification via communication
    CommunicationManager_SendFaultNotification(fault_info->fault_code, fault_info->motor_id);

    // Take action based on fault severity
    if (fault_info->severity >= FAULT_SEVERITY_MAJOR)
    {
        if (fault_info->motor_id < 2)
        {
            // Stop specific motor
            MotorController_Enable(fault_info->motor_id, false);
        }
        else
        {
            // System fault - emergency stop
            SystemController_EmergencyStop();
        }
    }

    // Call registered callback if available
    if (g_main_controller.callbacks.fault_callback)
    {
        g_main_controller.callbacks.fault_callback(
            fault_info->fault_code,
            fault_info->motor_id,
            DIAG_SEVERITY_CRITICAL);
    }
}