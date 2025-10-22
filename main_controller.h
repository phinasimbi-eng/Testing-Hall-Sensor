/**
 * @file main_controller.h
 * @brief Main System Controller Header - Central coordination interface
 * @author Enhanced by AI Assistant
 * @version v3.0.0
 * @date 2025
 *
 * Provides unified API for controlling the N32G430 dual brushless motor
 * control system with Hall sensor feedback and FOC algorithms.
 *
 * This controller coordinates multiple sub-controllers for modular operation:
 * - Motor Controller: Motor-specific control and algorithms
 * - Hardware Controller: Low-level hardware abstraction
 * - Safety Controller: Fault detection and safety management
 * - Communication Manager: Interface management (UART/SPI/OLED)
 * - System Diagnostics: Performance monitoring and diagnostics
 */

#ifndef MAIN_CONTROLLER_H
#define MAIN_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

/* Include sub-controller interfaces */
#include "controllers/motor_controller.h"
#include "controllers/hardware_controller.h"
#include "controllers/safety_controller.h"
#include "communication_manager.h"
#include "system_diagnostics.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /* =============================================================================
     * TYPE DEFINITIONS AND ENUMERATIONS
     * ============================================================================= */

    /**
     * @brief System operational states
     */
    typedef enum
    {
        SYSTEM_STATE_INITIALIZING = 0, /**< System is initializing */
        SYSTEM_STATE_READY,            /**< System ready, motors stopped */
        SYSTEM_STATE_RUNNING,          /**< System running, motors active */
        SYSTEM_STATE_FAULT,            /**< System in fault condition */
        SYSTEM_STATE_SHUTDOWN          /**< System shutting down */
    } SystemState_t;

    /**
     * @brief System operating modes
     */
    typedef enum
    {
        SYSTEM_MODE_STANDBY = 0,     /**< Standby mode, no motor operation */
        SYSTEM_MODE_SPEED_CONTROL,   /**< Speed control mode */
        SYSTEM_MODE_TORQUE_CONTROL,  /**< Torque control mode */
        SYSTEM_MODE_DUAL_MOTOR_SYNC, /**< Synchronized dual motor operation */
        SYSTEM_MODE_DIAGNOSTIC       /**< Diagnostic/test mode */
    } SystemMode_t;

    /**
     * @brief System operation results
     */
    typedef enum
    {
        SYSTEM_SUCCESS = 0,             /**< Operation successful */
        SYSTEM_ERROR_INVALID_PARAMETER, /**< Invalid parameter provided */
        SYSTEM_ERROR_INVALID_STATE,     /**< Invalid system state for operation */
        SYSTEM_ERROR_FAULT_ACTIVE,      /**< System fault prevents operation */
        SYSTEM_ERROR_COMMUNICATION,     /**< Communication interface error */
        SYSTEM_ERROR_HARDWARE,          /**< Hardware error detected */
        SYSTEM_ERROR_TIMEOUT            /**< Operation timeout */
    } SystemResult_t;

    /**
     * @brief System status information structure
     */
    typedef struct
    {
        SystemState_t current_state; /**< Current system state */
        SystemMode_t operating_mode; /**< Current operating mode */

        uint32_t uptime_ms;   /**< System uptime in milliseconds */
        uint16_t fault_count; /**< Total fault occurrence count */

        bool motor1_enabled;        /**< Motor 1 enable status */
        bool motor2_enabled;        /**< Motor 2 enable status */
        bool emergency_stop_active; /**< Emergency stop status */

        int32_t motor1_target_speed; /**< Motor 1 target speed (RPM) */
        int32_t motor2_target_speed; /**< Motor 2 target speed (RPM) */
        int32_t motor1_actual_speed; /**< Motor 1 actual speed (RPM) */
        int32_t motor2_actual_speed; /**< Motor 2 actual speed (RPM) */

        int16_t motor1_current; /**< Motor 1 current (Q15 format) */
        int16_t motor2_current; /**< Motor 2 current (Q15 format) */

        uint16_t system_voltage; /**< System DC bus voltage */

    } SystemStatus_t;

/* =============================================================================
 * CONFIGURATION CONSTANTS
 * ============================================================================= */

/** Maximum allowed motor speed (RPM) */
#define SYSTEM_MAX_MOTOR_SPEED_RPM (3000)

/** Minimum allowed motor speed (RPM) */
#define SYSTEM_MIN_MOTOR_SPEED_RPM (100)

/** Maximum allowed torque command */
#define SYSTEM_MAX_TORQUE_COMMAND (30000)

/** System heartbeat timeout (ms) */
#define SYSTEM_HEARTBEAT_TIMEOUT_MS (5000)

/** Emergency stop debounce time (ms) */
#define EMERGENCY_STOP_DEBOUNCE_MS (100)

    /* =============================================================================
     * PUBLIC FUNCTION DECLARATIONS
     * ============================================================================= */

    /**
     * @brief Initialize the main system controller
     * @details Performs complete system initialization including hardware setup,
     *          motor initialization, communication interfaces, and diagnostics
     * @return SystemResult_t SUCCESS if initialization completed successfully
     */
    SystemResult_t SystemController_Initialize(void);

    /**
     * @brief Main system controller execution loop
     * @details Central control loop that manages system state, processes commands,
     *          monitors performance, and coordinates motor operations
     * @note This function should be called from the main application loop
     */
    void SystemController_Execute(void);

    /**
     * @brief Set system operating mode
     * @param mode New operating mode to set
     * @return SystemResult_t SUCCESS if mode change is valid
     */
    SystemResult_t SystemController_SetMode(SystemMode_t mode);

    /**
     * @brief Control individual motor operation
     * @param motor_id Motor identifier (0 or 1)
     * @param enable Enable/disable flag
     * @param target_value Target speed (RPM) or torque based on current mode
     * @return SystemResult_t SUCCESS if command executed successfully
     */
    SystemResult_t SystemController_ControlMotor(uint8_t motor_id, bool enable, int32_t target_value);

    /**
     * @brief Trigger emergency stop for all motors
     * @details Immediately stops both motors and transitions system to safe state
     */
    void SystemController_EmergencyStop(void);

    /**
     * @brief Clear emergency stop condition
     * @details Allows system to recover from emergency stop state
     * @return SystemResult_t SUCCESS if cleared successfully
     */
    SystemResult_t SystemController_ClearEmergencyStop(void);

    /**
     * @brief Get current system status information
     * @param status Pointer to status structure to populate
     * @return SystemResult_t SUCCESS if status retrieved successfully
     */
    SystemResult_t SystemController_GetStatus(SystemStatus_t *status);

    /**
     * @brief Get system uptime in milliseconds
     * @return uint32_t System uptime in milliseconds
     */
    uint32_t SystemController_GetUptime(void);

    /**
     * @brief Check if system is in fault state
     * @return bool True if system has active faults
     */
    bool SystemController_HasFaults(void);

    /**
     * @brief Reset system fault conditions
     * @details Attempts to clear fault conditions and return to ready state
     * @return SystemResult_t SUCCESS if faults cleared successfully
     */
    SystemResult_t SystemController_ResetFaults(void);

    /* =============================================================================
     * CALLBACK FUNCTION TYPES
     * ============================================================================= */

    /**
     * @brief System state change callback function type
     * @param old_state Previous system state
     * @param new_state New system state
     */
    typedef void (*SystemStateChangeCallback_t)(SystemState_t old_state, SystemState_t new_state);

    /**
     * @brief Fault detection callback function type
     * @param fault_code Detected fault code
     * @param motor_id Motor that generated the fault (0xFF for system faults)
     */
    typedef void (*SystemFaultCallback_t)(uint16_t fault_code, uint8_t motor_id);

    /**
     * @brief Register system state change callback
     * @param callback Callback function to register
     * @return SystemResult_t SUCCESS if callback registered successfully
     */
    SystemResult_t SystemController_RegisterStateCallback(SystemStateChangeCallback_t callback);

    /**
     * @brief Register fault detection callback
     * @param callback Callback function to register
     * @return SystemResult_t SUCCESS if callback registered successfully
     */
    SystemResult_t SystemController_RegisterFaultCallback(SystemFaultCallback_t callback);

/* =============================================================================
 * UTILITY MACROS
 * ============================================================================= */

/** Convert RPM to internal speed units */
#define RPM_TO_INTERNAL(rpm) ((int32_t)((rpm) * 65536 / 60))

/** Convert internal speed units to RPM */
#define INTERNAL_TO_RPM(internal) ((int32_t)((internal) * 60 / 65536))

/** Check if motor ID is valid */
#define IS_VALID_MOTOR_ID(id) ((id) <= 1)

/** Check if speed is within valid range */
#define IS_VALID_SPEED(speed) (((speed) >= SYSTEM_MIN_MOTOR_SPEED_RPM) && \
                               ((speed) <= SYSTEM_MAX_MOTOR_SPEED_RPM))

#ifdef __cplusplus
}
#endif

#endif /* MAIN_CONTROLLER_H */