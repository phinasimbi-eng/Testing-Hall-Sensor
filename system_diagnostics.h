/**
 * @file system_diagnostics.h
 * @brief System Diagnostics and Performance Monitoring
 * @author Enhanced by AI Assistant
 * @version v2.0.0
 * @date 2025
 *
 * Comprehensive diagnostics system for monitoring motor control performance,
 * fault detection, and system health indicators.
 */

#ifndef SYSTEM_DIAGNOSTICS_H
#define SYSTEM_DIAGNOSTICS_H

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
     * @brief Diagnostic severity levels
     */
    typedef enum
    {
        DIAG_SEVERITY_INFO = 0, /**< Informational message */
        DIAG_SEVERITY_WARNING,  /**< Warning condition */
        DIAG_SEVERITY_ERROR,    /**< Error condition */
        DIAG_SEVERITY_CRITICAL  /**< Critical fault condition */
    } DiagSeverity_t;

    /**
     * @brief System fault codes
     */
    typedef enum
    {
        FAULT_CODE_NONE = 0x0000, /**< No fault */

        // Motor faults (0x1000 - 0x1FFF)
        FAULT_CODE_MOTOR_OVERCURRENT = 0x1001, /**< Motor overcurrent */
        FAULT_CODE_MOTOR_OVERSPEED = 0x1002,   /**< Motor overspeed */
        FAULT_CODE_MOTOR_STALL = 0x1003,       /**< Motor stall detected */
        FAULT_CODE_MOTOR_HALL_FAULT = 0x1004,  /**< Hall sensor fault */
        FAULT_CODE_MOTOR_PHASE_LOSS = 0x1005,  /**< Motor phase loss */

        // System faults (0x2000 - 0x2FFF)
        FAULT_CODE_SYSTEM_OVERVOLTAGE = 0x2001,  /**< System overvoltage */
        FAULT_CODE_SYSTEM_UNDERVOLTAGE = 0x2002, /**< System undervoltage */
        FAULT_CODE_SYSTEM_OVERTEMP = 0x2003,     /**< System overtemperature */
        FAULT_CODE_SYSTEM_WATCHDOG = 0x2004,     /**< Watchdog timeout */

        // Communication faults (0x3000 - 0x3FFF)
        FAULT_CODE_COMM_TIMEOUT = 0x3001,  /**< Communication timeout */
        FAULT_CODE_COMM_CHECKSUM = 0x3002, /**< Communication checksum error */
        FAULT_CODE_COMM_PROTOCOL = 0x3003, /**< Protocol error */

        // Hardware faults (0x4000 - 0x4FFF)
        FAULT_CODE_ADC_FAULT = 0x4001,   /**< ADC conversion fault */
        FAULT_CODE_PWM_FAULT = 0x4002,   /**< PWM generation fault */
        FAULT_CODE_MEMORY_FAULT = 0x4003 /**< Memory corruption */

    } FaultCode_t;

    /**
     * @brief Performance metrics structure
     */
    typedef struct
    {
        uint32_t loop_count;              /**< Main loop execution count */
        uint32_t max_loop_time_us;        /**< Maximum loop execution time (μs) */
        uint32_t avg_loop_time_us;        /**< Average loop execution time (μs) */
        uint32_t cpu_utilization_percent; /**< CPU utilization percentage */

        uint32_t pwm_interrupt_count;  /**< PWM interrupt count */
        uint32_t adc_conversion_count; /**< ADC conversion count */
        uint32_t comm_message_count;   /**< Communication message count */

        uint16_t stack_usage_bytes; /**< Stack usage in bytes */
        uint16_t heap_usage_bytes;  /**< Heap usage in bytes */

    } PerformanceMetrics_t;

    /**
     * @brief Motor diagnostic data structure
     */
    typedef struct
    {
        uint8_t motor_id; /**< Motor identifier */

        struct
        {
            int32_t target_speed_rpm; /**< Target speed (RPM) */
            int32_t actual_speed_rpm; /**< Actual speed (RPM) */
            int32_t speed_error_rpm;  /**< Speed tracking error (RPM) */
            uint16_t speed_stability; /**< Speed stability metric (0-100%) */
        } speed;

        struct
        {
            int16_t ia_q15;       /**< Phase A current (Q15) */
            int16_t ib_q15;       /**< Phase B current (Q15) */
            int16_t ic_q15;       /**< Phase C current (Q15) */
            uint16_t rms_current; /**< RMS current */
            uint16_t current_thd; /**< Current Total Harmonic Distortion */
        } current;

        struct
        {
            uint16_t dc_voltage;  /**< DC bus voltage */
            uint16_t temperature; /**< Motor temperature */
            uint8_t hall_state;   /**< Hall sensor state */
            int16_t rotor_angle;  /**< Estimated rotor angle */
        } sensors;

        struct
        {
            uint32_t runtime_hours;   /**< Total runtime hours */
            uint32_t start_count;     /**< Motor start count */
            uint32_t fault_count;     /**< Total fault count */
            uint32_t last_fault_code; /**< Last occurred fault code */
        } statistics;

    } MotorDiagnostics_t;

    /**
     * @brief System diagnostic data structure
     */
    typedef struct
    {
        uint32_t uptime_ms;  /**< System uptime (ms) */
        uint32_t boot_count; /**< System boot count */

        PerformanceMetrics_t performance; /**< Performance metrics */
        MotorDiagnostics_t motors[2];     /**< Motor diagnostic data */

        struct
        {
            uint16_t active_faults[8];   /**< Currently active fault codes */
            uint8_t fault_count;         /**< Number of active faults */
            uint32_t last_fault_time_ms; /**< Last fault occurrence time */
            uint32_t total_fault_count;  /**< Total fault occurrence count */
        } faults;

        struct
        {
            uint8_t firmware_version[3]; /**< Firmware version [major.minor.patch] */
            uint32_t build_timestamp;    /**< Firmware build timestamp */
            uint8_t hardware_revision;   /**< Hardware revision */
        } version_info;

    } SystemDiagnostics_t;

/* =============================================================================
 * CONFIGURATION CONSTANTS
 * ============================================================================= */

/** Maximum number of stored fault codes */
#define DIAG_MAX_STORED_FAULTS (32)

/** Performance sampling period (ms) */
#define DIAG_PERFORMANCE_SAMPLE_MS (100)

/** Fault history buffer size */
#define DIAG_FAULT_HISTORY_SIZE (16)

/** Maximum diagnostic message length */
#define DIAG_MAX_MESSAGE_LENGTH (128)

    /* =============================================================================
     * PUBLIC FUNCTION DECLARATIONS
     * ============================================================================= */

    /**
     * @brief Initialize system diagnostics
     * @details Sets up diagnostic monitoring, fault detection, and performance tracking
     * @return bool True if initialization successful
     */
    bool SystemDiagnostics_Initialize(void);

    /**
     * @brief Update diagnostic data and perform monitoring
     * @details Should be called periodically from main loop
     */
    void SystemDiagnostics_Update(void);

    /**
     * @brief Record a system fault
     * @param fault_code Fault code to record
     * @param motor_id Motor that generated fault (0xFF for system faults)
     * @param severity Fault severity level
     * @return bool True if fault recorded successfully
     */
    bool SystemDiagnostics_RecordFault(FaultCode_t fault_code, uint8_t motor_id, DiagSeverity_t severity);

    /**
     * @brief Clear specific fault code
     * @param fault_code Fault code to clear
     * @return bool True if fault cleared successfully
     */
    bool SystemDiagnostics_ClearFault(FaultCode_t fault_code);

    /**
     * @brief Clear all faults
     * @details Clears all active fault conditions
     */
    void SystemDiagnostics_ClearAllFaults(void);

    /**
     * @brief Get current system diagnostic data
     * @param diagnostics Pointer to structure to populate with diagnostic data
     * @return bool True if data retrieved successfully
     */
    bool SystemDiagnostics_GetData(SystemDiagnostics_t *diagnostics);

    /**
     * @brief Get current performance metrics
     * @param metrics Pointer to structure to populate with performance data
     * @return bool True if metrics retrieved successfully
     */
    bool SystemDiagnostics_GetPerformanceMetrics(PerformanceMetrics_t *metrics);

    /**
     * @brief Get motor diagnostic data
     * @param motor_id Motor identifier (0 or 1)
     * @param diagnostics Pointer to structure to populate
     * @return bool True if data retrieved successfully
     */
    bool SystemDiagnostics_GetMotorData(uint8_t motor_id, MotorDiagnostics_t *diagnostics);

    /**
     * @brief Check if specific fault is active
     * @param fault_code Fault code to check
     * @return bool True if fault is currently active
     */
    bool SystemDiagnostics_IsFaultActive(FaultCode_t fault_code);

    /**
     * @brief Get number of active faults
     * @return uint8_t Number of currently active faults
     */
    uint8_t SystemDiagnostics_GetActiveFaultCount(void);

    /**
     * @brief Start performance timing measurement
     * @details Call at the beginning of a timed section
     */
    void SystemDiagnostics_StartTiming(void);

    /**
     * @brief End performance timing measurement
     * @details Call at the end of a timed section to record execution time
     */
    void SystemDiagnostics_EndTiming(void);

    /**
     * @brief Log diagnostic message
     * @param severity Message severity level
     * @param format Printf-style format string
     * @param ... Variable arguments for format string
     */
    void SystemDiagnostics_LogMessage(DiagSeverity_t severity, const char *format, ...);

    /**
     * @brief Run system self-test
     * @details Performs comprehensive system health check
     * @return bool True if all tests passed
     */
    bool SystemDiagnostics_RunSelfTest(void);

    /* =============================================================================
     * CALLBACK FUNCTION TYPES
     * ============================================================================= */

    /**
     * @brief Fault detection callback function type
     * @param fault_code Detected fault code
     * @param motor_id Motor that generated the fault
     * @param severity Fault severity level
     */
    typedef void (*FaultDetectionCallback_t)(FaultCode_t fault_code, uint8_t motor_id, DiagSeverity_t severity);

    /**
     * @brief Performance threshold callback function type
     * @param metric_id Performance metric identifier
     * @param current_value Current metric value
     * @param threshold_value Threshold that was exceeded
     */
    typedef void (*PerformanceThresholdCallback_t)(uint8_t metric_id, uint32_t current_value, uint32_t threshold_value);

    /**
     * @brief Register fault detection callback
     * @param callback Callback function to register
     * @return bool True if callback registered successfully
     */
    bool SystemDiagnostics_RegisterFaultCallback(FaultDetectionCallback_t callback);

    /**
     * @brief Register performance threshold callback
     * @param callback Callback function to register
     * @return bool True if callback registered successfully
     */
    bool SystemDiagnostics_RegisterPerformanceCallback(PerformanceThresholdCallback_t callback);

/* =============================================================================
 * UTILITY MACROS
 * ============================================================================= */

/** Convert fault code to category */
#define FAULT_GET_CATEGORY(code) (((code) >> 12) & 0xF)

/** Check if fault is motor-related */
#define FAULT_IS_MOTOR(code) (FAULT_GET_CATEGORY(code) == 1)

/** Check if fault is system-related */
#define FAULT_IS_SYSTEM(code) (FAULT_GET_CATEGORY(code) == 2)

/** Check if fault is communication-related */
#define FAULT_IS_COMM(code) (FAULT_GET_CATEGORY(code) == 3)

/** Check if fault is hardware-related */
#define FAULT_IS_HARDWARE(code) (FAULT_GET_CATEGORY(code) == 4)

/** Performance timing macros for easy use */
#define DIAG_TIME_START() SystemDiagnostics_StartTiming()
#define DIAG_TIME_END() SystemDiagnostics_EndTiming()

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_DIAGNOSTICS_H */