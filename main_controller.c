/**
 * @file main_controller.c
 * @brief Main System Controller - Central coordination of dual motor control system
 * @author Enhanced by AI Assistant
 * @version v2.0.0
 * @date 2025
 *
 * This file provides unified system control for the N32G430 dual brushless motor
 * control system with Hall sensor feedback and FOC control algorithms.
 *
 * Key Features:
 * - Dual motor coordination and synchronization
 * - System state management and fault handling
 * - Communication interface management (UART/SPI/OLED)
 * - Real-time performance monitoring and diagnostics
 * - Configurable operating modes (speed/torque control)
 */

#include "main_controller.h"
#include "SystemDefine.h"
#include "MotorDrive.h"
#include "system_diagnostics.h"
#include "communication_manager.h"

/* =============================================================================
 * GLOBAL VARIABLES AND STATE MANAGEMENT
 * ============================================================================= */

/**
 * @brief Main system controller state structure
 */
typedef struct
{
    SystemState_t current_state; /**< Current system operational state */
    SystemMode_t operating_mode; /**< Current operating mode */

    struct
    {
        uint32_t boot_time_ms;      /**< System boot timestamp */
        uint32_t uptime_ms;         /**< System uptime counter */
        uint32_t last_heartbeat_ms; /**< Last heartbeat timestamp */
        uint16_t fault_count;       /**< Total fault occurrence count */
        uint8_t restart_count;      /**< System restart counter */
    } diagnostics;

    struct
    {
        bool motor1_enabled;       /**< Motor 1 enable status */
        bool motor2_enabled;       /**< Motor 2 enable status */
        bool emergency_stop;       /**< Emergency stop flag */
        bool system_initialized;   /**< System initialization complete flag */
        bool communication_active; /**< Communication interface active flag */
    } status_flags;

    struct
    {
        int32_t motor1_target_speed;  /**< Motor 1 target speed (RPM) */
        int32_t motor2_target_speed;  /**< Motor 2 target speed (RPM) */
        int32_t motor1_target_torque; /**< Motor 1 target torque */
        int32_t motor2_target_torque; /**< Motor 2 target torque */
    } control_targets;

} SystemController_t;

/** Global system controller instance */
static SystemController_t g_system_controller = {0};

/** External motor objects from MotorDrive.c */
extern Motor_Obj motor_I[2];
extern SystemInterface_Obj SystemInterfaceObj[2];

/* =============================================================================
 * PRIVATE FUNCTION DECLARATIONS
 * ============================================================================= */

static void SystemController_InitializeHardware(void);
static void SystemController_InitializeMotors(void);
static void SystemController_StateMachine(void);
static void SystemController_UpdateDiagnostics(void);
static void SystemController_HandleFaults(void);
static void SystemController_ProcessCommands(void);
static void SystemController_UpdateControlTargets(void);
static void SystemController_MonitorPerformance(void);

/* =============================================================================
 * PUBLIC FUNCTION IMPLEMENTATIONS
 * ============================================================================= */

/**
 * @brief Initialize the main system controller
 * @details Performs complete system initialization including hardware setup,
 *          motor initialization, communication interfaces, and diagnostics
 * @return SystemResult_t SUCCESS if initialization completed successfully
 */
SystemResult_t SystemController_Initialize(void)
{
    // Initialize system state
    g_system_controller.current_state = SYSTEM_STATE_INITIALIZING;
    g_system_controller.operating_mode = SYSTEM_MODE_STANDBY;
    g_system_controller.diagnostics.boot_time_ms = HardwareController_GetTimestampMs();

    // Initialize hardware layers
    SystemController_InitializeHardware();

    // Initialize motor control systems
    SystemController_InitializeMotors();

    // Initialize communication interfaces
    CommunicationManager_Initialize();

    // Initialize diagnostics system
    SystemDiagnostics_Initialize();

    // Set system as initialized
    g_system_controller.status_flags.system_initialized = true;
    g_system_controller.current_state = SYSTEM_STATE_READY;

    return SYSTEM_SUCCESS;
}

/**
 * @brief Main system controller execution loop
 * @details Central control loop that manages system state, processes commands,
 *          monitors performance, and coordinates motor operations
 * @note This function should be called from the main application loop
 */
void SystemController_Execute(void)
{
    // Ensure system is properly initialized
    if (!g_system_controller.status_flags.system_initialized)
    {
        return;
    }

    // Update system diagnostics
    SystemController_UpdateDiagnostics();

    // Execute state machine
    SystemController_StateMachine();

    // Process incoming commands
    SystemController_ProcessCommands();

    // Update motor control targets
    SystemController_UpdateControlTargets();

    // Handle any detected faults
    SystemController_HandleFaults();

    // Monitor system performance
    SystemController_MonitorPerformance();

    // Execute motor control main loop
    MotorMain_Circle();
}

/**
 * @brief Set system operating mode
 * @param mode New operating mode to set
 * @return SystemResult_t SUCCESS if mode change is valid
 */
SystemResult_t SystemController_SetMode(SystemMode_t mode)
{
    // Validate mode transition
    if (g_system_controller.current_state != SYSTEM_STATE_READY &&
        g_system_controller.current_state != SYSTEM_STATE_RUNNING)
    {
        return SYSTEM_ERROR_INVALID_STATE;
    }

    switch (mode)
    {
    case SYSTEM_MODE_STANDBY:
        // Stop both motors safely
        SystemController_EmergencyStop();
        break;

    case SYSTEM_MODE_SPEED_CONTROL:
        // Configure motors for speed control
        g_system_controller.operating_mode = mode;
        break;

    case SYSTEM_MODE_TORQUE_CONTROL:
        // Configure motors for torque control
        g_system_controller.operating_mode = mode;
        break;

    case SYSTEM_MODE_DUAL_MOTOR_SYNC:
        // Configure synchronized dual motor operation
        g_system_controller.operating_mode = mode;
        break;

    default:
        return SYSTEM_ERROR_INVALID_PARAMETER;
    }

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
    if (motor_id > 1)
    {
        return SYSTEM_ERROR_INVALID_PARAMETER;
    }

    // Check system state
    if (g_system_controller.current_state == SYSTEM_STATE_FAULT)
    {
        return SYSTEM_ERROR_FAULT_ACTIVE;
    }

    // Update motor enable status
    if (motor_id == 0)
    {
        g_system_controller.status_flags.motor1_enabled = enable;
        if (g_system_controller.operating_mode == SYSTEM_MODE_SPEED_CONTROL)
        {
            g_system_controller.control_targets.motor1_target_speed = target_value;
        }
        else
        {
            g_system_controller.control_targets.motor1_target_torque = target_value;
        }
    }
    else
    {
        g_system_controller.status_flags.motor2_enabled = enable;
        if (g_system_controller.operating_mode == SYSTEM_MODE_SPEED_CONTROL)
        {
            g_system_controller.control_targets.motor2_target_speed = target_value;
        }
        else
        {
            g_system_controller.control_targets.motor2_target_torque = target_value;
        }
    }

    // Apply motor control command
    StartStopControl(motor_id, enable ? 1 : 0);

    if (g_system_controller.operating_mode == SYSTEM_MODE_SPEED_CONTROL && enable)
    {
        SpeedControl(motor_id, target_value);
    }

    return SYSTEM_SUCCESS;
}

/**
 * @brief Trigger emergency stop for all motors
 * @details Immediately stops both motors and transitions system to safe state
 */
void SystemController_EmergencyStop(void)
{
    // Set emergency stop flag
    g_system_controller.status_flags.emergency_stop = true;

    // Immediately disable both motors
    StartStopControl(0, 0);
    StartStopControl(1, 0);

    // Force motor shutdown
    AllPwmShut(0);
    AllPwmShut(1);

    // Update status flags
    g_system_controller.status_flags.motor1_enabled = false;
    g_system_controller.status_flags.motor2_enabled = false;

    // Transition to fault state
    g_system_controller.current_state = SYSTEM_STATE_FAULT;
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

    // Populate status structure
    status->current_state = g_system_controller.current_state;
    status->operating_mode = g_system_controller.operating_mode;
    status->uptime_ms = g_system_controller.diagnostics.uptime_ms;
    status->fault_count = g_system_controller.diagnostics.fault_count;

    status->motor1_enabled = g_system_controller.status_flags.motor1_enabled;
    status->motor2_enabled = g_system_controller.status_flags.motor2_enabled;
    status->emergency_stop_active = g_system_controller.status_flags.emergency_stop;

    // Get motor feedback data
    status->motor1_actual_speed = motor_I[0].DataObj.MotorSpeed;
    status->motor2_actual_speed = motor_I[1].DataObj.MotorSpeed;

    status->motor1_current = motor_I[0].DataObj.Ia_Q15;
    status->motor2_current = motor_I[1].DataObj.Ia_Q15;

    status->system_voltage = motor_I[0].DataObj.Udc_Real;

    return SYSTEM_SUCCESS;
}

/* =============================================================================
 * PRIVATE FUNCTION IMPLEMENTATIONS
 * ============================================================================= */

/**
 * @brief Initialize hardware components
 * @details Sets up clocks, GPIO, ADC, PWM, timers and other peripherals
 */
static void SystemController_InitializeHardware(void)
{
    // Initialize system hardware (from existing System_Init)
    System_Init();
}

/**
 * @brief Initialize motor control systems
 * @details Configure motor parameters, FOC algorithms, and sensor interfaces
 */
static void SystemController_InitializeMotors(void)
{
    // Initialize motor drive system (from existing MotorDrive_Init)
    MotorDrive_Init(&SystemInterfaceObj[0], &SystemInterfaceObj[1]);
}

/**
 * @brief Execute main system state machine
 * @details Manages transitions between operational states based on conditions
 */
static void SystemController_StateMachine(void)
{
    switch (g_system_controller.current_state)
    {
    case SYSTEM_STATE_INITIALIZING:
        // Handled in initialization
        break;

    case SYSTEM_STATE_READY:
        // Check for motor start requests
        if (g_system_controller.status_flags.motor1_enabled ||
            g_system_controller.status_flags.motor2_enabled)
        {
            g_system_controller.current_state = SYSTEM_STATE_RUNNING;
        }
        break;

    case SYSTEM_STATE_RUNNING:
        // Check for stop conditions
        if (!g_system_controller.status_flags.motor1_enabled &&
            !g_system_controller.status_flags.motor2_enabled)
        {
            g_system_controller.current_state = SYSTEM_STATE_READY;
        }
        break;

    case SYSTEM_STATE_FAULT:
        // Check for fault clearance
        if (!g_system_controller.status_flags.emergency_stop)
        {
            // Allow recovery to ready state after fault acknowledgment
            g_system_controller.current_state = SYSTEM_STATE_READY;
        }
        break;

    case SYSTEM_STATE_SHUTDOWN:
        // System is shutting down - no state transitions
        break;
    }
}

/**
 * @brief Update system diagnostics counters and timers
 */
static void SystemController_UpdateDiagnostics(void)
{
    uint32_t current_time = HardwareController_GetTimestampMs();
    g_system_controller.diagnostics.uptime_ms =
        current_time - g_system_controller.diagnostics.boot_time_ms;
    g_system_controller.diagnostics.last_heartbeat_ms = current_time;
}

/**
 * @brief Handle detected system faults
 */
static void SystemController_HandleFaults(void)
{
    // Check motor fault flags
    if (motor_I[0].Flag.Bits.SwOverCurrent || motor_I[1].Flag.Bits.SwOverCurrent)
    {
        g_system_controller.diagnostics.fault_count++;
        SystemController_EmergencyStop();
    }

    if (motor_I[0].Flag.Bits.OverVolatage || motor_I[1].Flag.Bits.OverVolatage ||
        motor_I[0].Flag.Bits.LackVolatage || motor_I[1].Flag.Bits.LackVolatage)
    {
        g_system_controller.diagnostics.fault_count++;
        SystemController_EmergencyStop();
    }
}

/**
 * @brief Process incoming commands from communication interfaces
 */
static void SystemController_ProcessCommands(void)
{
    // Process UART commands
    CommunicationManager_ProcessCommands();

    // Process button inputs
    uint16_t button_state = GetIO_Button1();
    if (button_state)
    {
        // Handle button press - toggle emergency stop
        if (g_system_controller.status_flags.emergency_stop)
        {
            g_system_controller.status_flags.emergency_stop = false;
        }
        else
        {
            SystemController_EmergencyStop();
        }
    }
}

/**
 * @brief Update motor control targets based on operating mode
 */
static void SystemController_UpdateControlTargets(void)
{
    if (g_system_controller.current_state == SYSTEM_STATE_RUNNING)
    {

        if (g_system_controller.operating_mode == SYSTEM_MODE_SPEED_CONTROL)
        {
            // Update speed targets
            if (g_system_controller.status_flags.motor1_enabled)
            {
                SpeedControl(0, g_system_controller.control_targets.motor1_target_speed);
            }
            if (g_system_controller.status_flags.motor2_enabled)
            {
                SpeedControl(1, g_system_controller.control_targets.motor2_target_speed);
            }
        }

        // Handle synchronized dual motor mode
        if (g_system_controller.operating_mode == SYSTEM_MODE_DUAL_MOTOR_SYNC)
        {
            // Implement motor synchronization logic here
            int32_t avg_target = (g_system_controller.control_targets.motor1_target_speed +
                                  g_system_controller.control_targets.motor2_target_speed) /
                                 2;

            SpeedControl(0, avg_target);
            SpeedControl(1, avg_target);
        }
    }
}

/**
 * @brief Monitor system performance metrics
 */
static void SystemController_MonitorPerformance(void)
{
    // Update performance counters and statistics
    // This can be expanded to include cycle time measurement,
    // memory usage monitoring, etc.
}

/**
 * @brief Get system uptime in milliseconds
 * @return uint32_t System uptime in milliseconds
 */
uint32_t SystemController_GetUptime(void)
{
    return g_system_controller.diagnostics.uptime_ms;
}

/**
 * @brief Check if system is in fault state
 * @return bool True if system has active faults
 */
bool SystemController_HasFaults(void)
{
    return (g_system_controller.current_state == SYSTEM_STATE_FAULT);
}