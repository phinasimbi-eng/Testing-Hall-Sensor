/**
 * @file safety_controller.h
 * @brief Safety and Fault Management Controller
 * @author Enhanced by AI Assistant
 * @version v2.0.0
 * @date 2025
 *
 * Dedicated safety controller that monitors system conditions, detects faults,
 * and ensures safe operation of the motor control system. This controller
 * implements safety-critical functions and fault management.
 */

#ifndef SAFETY_CONTROLLER_H
#define SAFETY_CONTROLLER_H

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
     * @brief Safety states
     */
    typedef enum
    {
        SAFETY_STATE_SAFE = 0,       /**< System is safe to operate */
        SAFETY_STATE_WARNING,        /**< Warning conditions detected */
        SAFETY_STATE_FAULT,          /**< Fault conditions detected */
        SAFETY_STATE_CRITICAL_FAULT, /**< Critical fault - immediate shutdown */
        SAFETY_STATE_EMERGENCY_STOP  /**< Emergency stop activated */
    } SafetyState_t;

    /**
     * @brief Fault severity levels
     */
    typedef enum
    {
        FAULT_SEVERITY_INFO = 0, /**< Informational - no action required */
        FAULT_SEVERITY_WARNING,  /**< Warning - monitor condition */
        FAULT_SEVERITY_MINOR,    /**< Minor fault - reduce performance */
        FAULT_SEVERITY_MAJOR,    /**< Major fault - stop operation */
        FAULT_SEVERITY_CRITICAL  /**< Critical fault - emergency shutdown */
    } FaultSeverity_t;

    /**
     * @brief Safety monitoring parameters
     */
    typedef struct
    {
        struct
        {
            uint16_t overcurrent_limit_ma;  /**< Overcurrent limit (mA) */
            uint16_t overcurrent_time_ms;   /**< Overcurrent time threshold (ms) */
            uint16_t overvoltage_limit_mv;  /**< Overvoltage limit (mV) */
            uint16_t undervoltage_limit_mv; /**< Undervoltage limit (mV) */
            uint16_t voltage_time_ms;       /**< Voltage fault time threshold (ms) */
        } electrical;

        struct
        {
            uint16_t overtemp_limit_c;  /**< Overtemperature limit (°C) */
            uint16_t overtemp_time_ms;  /**< Overtemperature time threshold (ms) */
            uint16_t temp_hysteresis_c; /**< Temperature hysteresis (°C) */
        } thermal;

        struct
        {
            uint16_t max_speed_rpm;     /**< Maximum allowed speed (RPM) */
            uint16_t overspeed_time_ms; /**< Overspeed time threshold (ms) */
            uint16_t stall_current_ma;  /**< Stall current threshold (mA) */
            uint16_t stall_time_ms;     /**< Stall time threshold (ms) */
        } mechanical;

        struct
        {
            uint16_t watchdog_timeout_ms;  /**< Watchdog timeout (ms) */
            uint16_t comm_timeout_ms;      /**< Communication timeout (ms) */
            uint16_t heartbeat_timeout_ms; /**< Heartbeat timeout (ms) */
        } communication;

    } SafetyParameters_t;

    /**
     * @brief Fault information structure
     */
    typedef struct
    {
        uint16_t fault_code;      /**< Unique fault code */
        FaultSeverity_t severity; /**< Fault severity level */
        uint8_t motor_id;         /**< Motor that generated fault (0xFF for system) */
        uint32_t timestamp_ms;    /**< Fault occurrence timestamp */
        uint32_t fault_data;      /**< Additional fault-specific data */
        bool active;              /**< Fault currently active */
        bool latched;             /**< Fault is latched (requires manual clear) */
        char description[64];     /**< Human-readable fault description */
    } FaultInfo_t;

    /**
     * @brief Safety monitoring data
     */
    typedef struct
    {
        struct
        {
            uint16_t current_ma[2]; /**< Motor currents (mA) */
            uint16_t dc_voltage_mv; /**< DC bus voltage (mV) */
            uint16_t temperature_c; /**< System temperature (°C) */
        } measurements;

        struct
        {
            uint32_t speed_rpm[2]; /**< Motor speeds (RPM) */
            bool direction[2];     /**< Motor directions */
            bool enabled[2];       /**< Motor enable states */
        } motor_status;

        struct
        {
            bool emergency_stop_pressed; /**< Emergency stop button state */
            bool enable_signal;          /**< External enable signal */
            bool fault_output_active;    /**< Fault output signal state */
        } signals;

    } SafetyMonitoringData_t;

    /**
     * @brief Safety action configuration
     */
    typedef enum
    {
        SAFETY_ACTION_NONE = 0,           /**< No action */
        SAFETY_ACTION_LOG_ONLY,           /**< Log fault only */
        SAFETY_ACTION_REDUCE_PERFORMANCE, /**< Reduce motor performance */
        SAFETY_ACTION_STOP_MOTOR,         /**< Stop affected motor */
        SAFETY_ACTION_STOP_ALL_MOTORS,    /**< Stop all motors */
        SAFETY_ACTION_EMERGENCY_SHUTDOWN  /**< Emergency system shutdown */
    } SafetyAction_t;

/* =============================================================================
 * CONFIGURATION CONSTANTS
 * ============================================================================= */

/** Maximum number of active faults */
#define SAFETY_MAX_ACTIVE_FAULTS (16)

/** Fault history buffer size */
#define SAFETY_FAULT_HISTORY_SIZE (32)

/** Safety monitoring update rate (Hz) */
#define SAFETY_MONITOR_RATE_HZ (1000)

/** Emergency stop response time (ms) */
#define SAFETY_EMERGENCY_RESPONSE_MS (10)

/** Default overcurrent limit (mA) */
#define SAFETY_DEFAULT_OVERCURRENT_MA (15000)

/** Default overvoltage limit (mV) */
#define SAFETY_DEFAULT_OVERVOLTAGE_MV (60000)

/** Default undervoltage limit (mV) */
#define SAFETY_DEFAULT_UNDERVOLTAGE_MV (18000)

/* =============================================================================
 * FAULT CODE DEFINITIONS
 * ============================================================================= */

/** System fault codes (0x1000-0x1FFF) */
#define FAULT_SYSTEM_OVERVOLTAGE (0x1001)
#define FAULT_SYSTEM_UNDERVOLTAGE (0x1002)
#define FAULT_SYSTEM_OVERTEMPERATURE (0x1003)
#define FAULT_SYSTEM_WATCHDOG_TIMEOUT (0x1004)
#define FAULT_SYSTEM_MEMORY_ERROR (0x1005)
#define FAULT_SYSTEM_COMM_TIMEOUT (0x1006)

/** Motor fault codes (0x2000-0x2FFF) */
#define FAULT_MOTOR_OVERCURRENT (0x2001)
#define FAULT_MOTOR_OVERSPEED (0x2002)
#define FAULT_MOTOR_STALL (0x2003)
#define FAULT_MOTOR_HALL_FAULT (0x2004)
#define FAULT_MOTOR_PHASE_LOSS (0x2005)
#define FAULT_MOTOR_ENCODER_FAULT (0x2006)

/** Hardware fault codes (0x3000-0x3FFF) */
#define FAULT_HW_ADC_ERROR (0x3001)
#define FAULT_HW_PWM_FAULT (0x3002)
#define FAULT_HW_GPIO_FAULT (0x3003)
#define FAULT_HW_CLOCK_FAULT (0x3004)

/** Safety fault codes (0x4000-0x4FFF) */
#define FAULT_SAFETY_EMERGENCY_STOP (0x4001)
#define FAULT_SAFETY_ENABLE_LOST (0x4002)
#define FAULT_SAFETY_WATCHDOG (0x4003)
#define FAULT_SAFETY_CONFIG_ERROR (0x4004)

    /* =============================================================================
     * PUBLIC FUNCTION DECLARATIONS
     * ============================================================================= */

    /**
     * @brief Initialize safety controller
     * @param parameters Safety monitoring parameters
     * @return bool True if initialization successful
     */
    bool SafetyController_Initialize(const SafetyParameters_t *parameters);

    /**
     * @brief Update safety monitoring and fault detection
     * @details Should be called at high frequency (1kHz) from main loop
     * @param monitoring_data Current system monitoring data
     */
    void SafetyController_Update(const SafetyMonitoringData_t *monitoring_data);

    /**
     * @brief Get current safety state
     * @return SafetyState_t Current safety state
     */
    SafetyState_t SafetyController_GetState(void);

    /**
     * @brief Activate emergency stop
     * @details Immediately activates emergency stop procedures
     */
    void SafetyController_ActivateEmergencyStop(void);

    /**
     * @brief Reset emergency stop condition
     * @details Resets emergency stop if conditions are safe
     * @return bool True if emergency stop reset successfully
     */
    bool SafetyController_ResetEmergencyStop(void);

    /**
     * @brief Report a fault condition
     * @param fault_code Unique fault identifier
     * @param severity Fault severity level
     * @param motor_id Motor that generated fault (0xFF for system faults)
     * @param fault_data Additional fault-specific data
     * @param description Human-readable fault description
     * @return bool True if fault reported successfully
     */
    bool SafetyController_ReportFault(uint16_t fault_code,
                                      FaultSeverity_t severity,
                                      uint8_t motor_id,
                                      uint32_t fault_data,
                                      const char *description);

    /**
     * @brief Clear specific fault
     * @param fault_code Fault code to clear
     * @return bool True if fault cleared successfully
     */
    bool SafetyController_ClearFault(uint16_t fault_code);

    /**
     * @brief Clear all non-critical faults
     * @return bool True if faults cleared successfully
     */
    bool SafetyController_ClearAllFaults(void);

    /**
     * @brief Get active fault information
     * @param faults Array to store fault information
     * @param max_faults Maximum number of faults to retrieve
     * @return uint8_t Number of active faults returned
     */
    uint8_t SafetyController_GetActiveFaults(FaultInfo_t *faults, uint8_t max_faults);

    /**
     * @brief Check if specific fault is active
     * @param fault_code Fault code to check
     * @return bool True if fault is currently active
     */
    bool SafetyController_IsFaultActive(uint16_t fault_code);

    /**
     * @brief Get total number of active faults
     * @return uint8_t Number of currently active faults
     */
    uint8_t SafetyController_GetActiveFaultCount(void);

    /**
     * @brief Configure safety parameters
     * @param parameters New safety parameters
     * @return bool True if parameters configured successfully
     */
    bool SafetyController_ConfigureParameters(const SafetyParameters_t *parameters);

    /**
     * @brief Get current safety parameters
     * @param parameters Pointer to structure to populate
     * @return bool True if parameters retrieved successfully
     */
    bool SafetyController_GetParameters(SafetyParameters_t *parameters);

    /**
     * @brief Enable/disable safety monitoring
     * @param enable Enable flag
     * @return bool True if operation successful
     */
    bool SafetyController_Enable(bool enable);

    /**
     * @brief Check if safety monitoring is enabled
     * @return bool True if safety monitoring is enabled
     */
    bool SafetyController_IsEnabled(void);

    /**
     * @brief Perform safety system self-test
     * @return bool True if self-test passed
     */
    bool SafetyController_SelfTest(void);

    /**
     * @brief Get safety statistics
     * @param total_faults Pointer to store total fault count
     * @param uptime_ms Pointer to store safety system uptime
     * @param last_fault_time Pointer to store last fault timestamp
     * @return bool True if statistics retrieved successfully
     */
    bool SafetyController_GetStatistics(uint32_t *total_faults,
                                        uint32_t *uptime_ms,
                                        uint32_t *last_fault_time);

    /**
     * @brief Feed safety watchdog
     * @details Resets the safety watchdog timer to prevent timeout
     */
    void SafetyController_FeedWatchdog(void);

    /* =============================================================================
     * CALLBACK FUNCTION TYPES
     * ============================================================================= */

    /**
     * @brief Safety state change callback function type
     * @param old_state Previous safety state
     * @param new_state New safety state
     */
    typedef void (*SafetyStateChangeCallback_t)(SafetyState_t old_state, SafetyState_t new_state);

    /**
     * @brief Fault occurrence callback function type
     * @param fault_info Information about the fault
     */
    typedef void (*SafetyFaultCallback_t)(const FaultInfo_t *fault_info);

    /**
     * @brief Emergency stop callback function type
     * @param activated True if emergency stop activated, false if deactivated
     */
    typedef void (*SafetyEmergencyStopCallback_t)(bool activated);

    /**
     * @brief Register safety state change callback
     * @param callback Callback function to register
     * @return bool True if callback registered successfully
     */
    bool SafetyController_RegisterStateCallback(SafetyStateChangeCallback_t callback);

    /**
     * @brief Register fault occurrence callback
     * @param callback Callback function to register
     * @return bool True if callback registered successfully
     */
    bool SafetyController_RegisterFaultCallback(SafetyFaultCallback_t callback);

    /**
     * @brief Register emergency stop callback
     * @param callback Callback function to register
     * @return bool True if callback registered successfully
     */
    bool SafetyController_RegisterEmergencyStopCallback(SafetyEmergencyStopCallback_t callback);

/* =============================================================================
 * UTILITY MACROS
 * ============================================================================= */

/** Check if fault code indicates system fault */
#define SAFETY_IS_SYSTEM_FAULT(code) (((code) & 0xF000) == 0x1000)

/** Check if fault code indicates motor fault */
#define SAFETY_IS_MOTOR_FAULT(code) (((code) & 0xF000) == 0x2000)

/** Check if fault code indicates hardware fault */
#define SAFETY_IS_HARDWARE_FAULT(code) (((code) & 0xF000) == 0x3000)

/** Check if fault code indicates safety fault */
#define SAFETY_IS_SAFETY_FAULT(code) (((code) & 0xF000) == 0x4000)

/** Check if fault severity requires immediate shutdown */
#define SAFETY_REQUIRES_SHUTDOWN(severity) ((severity) >= FAULT_SEVERITY_MAJOR)

/** Get fault category from fault code */
#define SAFETY_GET_FAULT_CATEGORY(code) (((code) >> 12) & 0xF)

#ifdef __cplusplus
}
#endif

#endif /* SAFETY_CONTROLLER_H */