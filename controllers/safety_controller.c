/**
 * @file safety_controller.c
 * @brief Safety Controller Implementation
 * @author Enhanced by AI Assistant
 * @version v2.0.0
 * @date 2025
 *
 * Safety-critical subsystem that monitors system conditions,
 * manages fault detection, and executes emergency procedures.
 * Ensures safe operation of the motor control system.
 */

#include "controllers/safety_controller.h"
#include "SystemDefine.h"
#include "MotorDrive.h"
#include "ErrDeal.h"
#include <string.h>

/* =============================================================================
 * PRIVATE CONSTANTS AND DEFINITIONS
 * ============================================================================= */

#define SAFETY_WATCHDOG_TIMEOUT_MS 1000 /**< Watchdog timeout in milliseconds */
#define SAFETY_FAULT_HISTORY_SIZE 10    /**< Number of fault events to store */
#define SAFETY_TEMPERATURE_LIMIT 85     /**< Temperature limit in Celsius */
#define SAFETY_VOLTAGE_MIN_MV 10000     /**< Minimum voltage in millivolts */
#define SAFETY_VOLTAGE_MAX_MV 30000     /**< Maximum voltage in millivolts */
#define SAFETY_CURRENT_MAX_MA 25000     /**< Maximum current in milliamperes */

/* =============================================================================
 * PRIVATE TYPES AND STRUCTURES
 * ============================================================================= */

/**
 * @brief Fault event record for history tracking
 */
typedef struct
{
    SafetyFaultType_t fault_type;
    uint8_t motor_id;
    uint32_t timestamp_ms;
    uint32_t fault_data;
    bool resolved;
} FaultEvent_t;

/**
 * @brief Safety controller instance data
 */
typedef struct
{
    bool initialized;
    bool system_enabled;
    bool emergency_stop_active;
    SafetyState_t current_state;

    struct
    {
        uint32_t last_watchdog_feed_ms;
        uint32_t watchdog_timeout_count;
        bool watchdog_enabled;
    } watchdog;

    struct
    {
        SafetyLimits_t limits;
        SafetyThresholds_t thresholds;
        bool limits_configured;
    } configuration;

    struct
    {
        FaultEvent_t events[SAFETY_FAULT_HISTORY_SIZE];
        uint8_t write_index;
        uint8_t event_count;
        uint32_t total_fault_count;
    } fault_history;

    struct
    {
        SafetyFaultCallback_t fault_callback;
        SafetyStateChangeCallback_t state_callback;
        SafetyEmergencyCallback_t emergency_callback;
    } callbacks;

    struct
    {
        uint32_t monitor_cycle_count;
        uint32_t fault_check_count;
        uint32_t emergency_stop_count;
        uint32_t last_update_ms;
    } statistics;

} SafetyControllerState_t;

/** Global safety controller state */
static SafetyControllerState_t g_safety_controller = {0};

/* =============================================================================
 * PRIVATE FUNCTION DECLARATIONS
 * ============================================================================= */

static bool SafetyController_InitializeWatchdog(void);
static void SafetyController_UpdateState(SafetyState_t new_state);
static void SafetyController_RecordFaultEvent(SafetyFaultType_t fault_type,
                                              uint8_t motor_id, uint32_t fault_data);
static bool SafetyController_CheckVoltageRange(uint32_t voltage_mv);
static bool SafetyController_CheckCurrentRange(uint32_t current_ma);
static bool SafetyController_CheckTemperatureRange(int32_t temperature_c);
static bool SafetyController_ValidateLimits(const SafetyLimits_t *limits);
static bool SafetyController_ValidateThresholds(const SafetyThresholds_t *thresholds);
static void SafetyController_ExecuteEmergencyStop(const char *reason);
static void SafetyController_CheckWatchdogTimeout(void);

/* =============================================================================
 * PUBLIC FUNCTION IMPLEMENTATIONS
 * ============================================================================= */

/**
 * @brief Initialize safety controller subsystem
 * @return bool True if initialization successful
 */
bool SafetyController_Initialize(void)
{
    // Initialize controller state
    memset(&g_safety_controller, 0, sizeof(SafetyControllerState_t));

    // Set initial state
    g_safety_controller.current_state = SAFETY_STATE_INITIALIZING;
    g_safety_controller.system_enabled = false;
    g_safety_controller.emergency_stop_active = false;

    // Initialize watchdog
    if (!SafetyController_InitializeWatchdog())
    {
        return false;
    }

    // Set default safety limits
    SafetyLimits_t default_limits = {
        .max_voltage_mv = SAFETY_VOLTAGE_MAX_MV,
        .min_voltage_mv = SAFETY_VOLTAGE_MIN_MV,
        .max_current_ma = SAFETY_CURRENT_MAX_MA,
        .max_temperature_c = SAFETY_TEMPERATURE_LIMIT,
        .max_speed_rpm = MOTOR_MAX_SPEED_RPM};

    memcpy(&g_safety_controller.configuration.limits, &default_limits,
           sizeof(SafetyLimits_t));

    // Set default thresholds
    SafetyThresholds_t default_thresholds = {
        .overcurrent_threshold_ma = (SAFETY_CURRENT_MAX_MA * 90) / 100,   // 90% of max
        .overvoltage_threshold_mv = (SAFETY_VOLTAGE_MAX_MV * 95) / 100,   // 95% of max
        .undervoltage_threshold_mv = (SAFETY_VOLTAGE_MIN_MV * 105) / 100, // 105% of min
        .overtemperature_threshold_c = SAFETY_TEMPERATURE_LIMIT - 10,     // 10°C margin
        .stall_detection_time_ms = 2000,
        .fault_recovery_time_ms = 5000};

    memcpy(&g_safety_controller.configuration.thresholds, &default_thresholds,
           sizeof(SafetyThresholds_t));

    g_safety_controller.configuration.limits_configured = true;
    g_safety_controller.initialized = true;

    // Transition to safe state
    SafetyController_UpdateState(SAFETY_STATE_SAFE);

    return true;
}

/**
 * @brief Configure safety limits and thresholds
 * @param limits Safety limits structure
 * @param thresholds Safety thresholds structure
 * @return bool True if configuration successful
 */
bool SafetyController_Configure(const SafetyLimits_t *limits,
                                const SafetyThresholds_t *thresholds)
{
    if (!limits || !thresholds || !g_safety_controller.initialized)
    {
        return false;
    }

    // Validate configuration
    if (!SafetyController_ValidateLimits(limits) ||
        !SafetyController_ValidateThresholds(thresholds))
    {
        return false;
    }

    // Store configuration
    memcpy(&g_safety_controller.configuration.limits, limits, sizeof(SafetyLimits_t));
    memcpy(&g_safety_controller.configuration.thresholds, thresholds, sizeof(SafetyThresholds_t));

    g_safety_controller.configuration.limits_configured = true;

    return true;
}

/**
 * @brief Check system safety conditions
 * @param conditions Current system conditions to check
 * @return SafetyStatus_t Safety check result
 */
SafetyStatus_t SafetyController_CheckConditions(const SafetyConditions_t *conditions)
{
    if (!conditions || !g_safety_controller.initialized)
    {
        return SAFETY_STATUS_ERROR;
    }

    g_safety_controller.statistics.monitor_cycle_count++;
    g_safety_controller.statistics.last_update_ms = HAL_GetTick();

    SafetyStatus_t status = SAFETY_STATUS_OK;

    // Check voltage range
    for (uint8_t i = 0; i < conditions->voltage_count; i++)
    {
        if (!SafetyController_CheckVoltageRange(conditions->voltages_mv[i]))
        {
            if (conditions->voltages_mv[i] > g_safety_controller.configuration.limits.max_voltage_mv)
            {
                SafetyController_RecordFaultEvent(SAFETY_FAULT_OVERVOLTAGE, 0, conditions->voltages_mv[i]);
                status = SAFETY_STATUS_CRITICAL_FAULT;
            }
            else
            {
                SafetyController_RecordFaultEvent(SAFETY_FAULT_UNDERVOLTAGE, 0, conditions->voltages_mv[i]);
                status = SAFETY_STATUS_WARNING;
            }
        }
    }

    // Check current range for each motor
    for (uint8_t motor = 0; motor < 2; motor++)
    {
        for (uint8_t phase = 0; phase < 3; phase++)
        {
            uint32_t current_ma = conditions->motor_currents_ma[motor][phase];
            if (!SafetyController_CheckCurrentRange(current_ma))
            {
                SafetyController_RecordFaultEvent(SAFETY_FAULT_OVERCURRENT, motor, current_ma);
                status = SAFETY_STATUS_CRITICAL_FAULT;
            }
        }
    }

    // Check temperature range
    for (uint8_t i = 0; i < conditions->temperature_count; i++)
    {
        if (!SafetyController_CheckTemperatureRange(conditions->temperatures_c[i]))
        {
            SafetyController_RecordFaultEvent(SAFETY_FAULT_OVERTEMPERATURE, 0, conditions->temperatures_c[i]);
            status = SAFETY_STATUS_WARNING;
        }
    }

    // Check motor speeds
    for (uint8_t motor = 0; motor < 2; motor++)
    {
        if (abs(conditions->motor_speeds_rpm[motor]) > g_safety_controller.configuration.limits.max_speed_rpm)
        {
            SafetyController_RecordFaultEvent(SAFETY_FAULT_OVERSPEED, motor, abs(conditions->motor_speeds_rpm[motor]));
            status = SAFETY_STATUS_WARNING;
        }
    }

    // Check emergency stop
    if (conditions->emergency_stop_pressed)
    {
        SafetyController_ExecuteEmergencyStop("Emergency stop button pressed");
        status = SAFETY_STATUS_EMERGENCY_STOP;
    }

    // Check watchdog timeout
    SafetyController_CheckWatchdogTimeout();

    // Update safety state based on status
    if (status == SAFETY_STATUS_CRITICAL_FAULT)
    {
        SafetyController_UpdateState(SAFETY_STATE_FAULT);
    }
    else if (status == SAFETY_STATUS_EMERGENCY_STOP)
    {
        SafetyController_UpdateState(SAFETY_STATE_EMERGENCY_STOP);
    }
    else if (status == SAFETY_STATUS_WARNING)
    {
        SafetyController_UpdateState(SAFETY_STATE_WARNING);
    }
    else if (g_safety_controller.current_state != SAFETY_STATE_SAFE)
    {
        SafetyController_UpdateState(SAFETY_STATE_SAFE);
    }

    return status;
}

/**
 * @brief Report safety fault to the system
 * @param fault_info Fault information structure
 * @return bool True if fault reported successfully
 */
bool SafetyController_ReportFault(const SafetyFaultInfo_t *fault_info)
{
    if (!fault_info || !g_safety_controller.initialized)
    {
        return false;
    }

    g_safety_controller.statistics.fault_check_count++;

    // Record fault event
    SafetyController_RecordFaultEvent(fault_info->fault_type,
                                      fault_info->motor_id,
                                      fault_info->fault_data);

    // Determine fault severity and response
    bool critical_fault = false;

    switch (fault_info->fault_type)
    {
    case SAFETY_FAULT_OVERCURRENT:
    case SAFETY_FAULT_OVERVOLTAGE:
    case SAFETY_FAULT_HARDWARE_FAILURE:
        critical_fault = true;
        break;

    case SAFETY_FAULT_UNDERVOLTAGE:
    case SAFETY_FAULT_OVERTEMPERATURE:
    case SAFETY_FAULT_OVERSPEED:
    case SAFETY_FAULT_STALL_DETECTED:
    case SAFETY_FAULT_COMMUNICATION_LOSS:
        critical_fault = false;
        break;

    default:
        break;
    }

    // Execute appropriate response
    if (critical_fault)
    {
        SafetyController_ExecuteEmergencyStop("Critical fault detected");
        SafetyController_UpdateState(SAFETY_STATE_FAULT);
    }
    else
    {
        SafetyController_UpdateState(SAFETY_STATE_WARNING);
    }

    // Notify callback
    if (g_safety_controller.callbacks.fault_callback)
    {
        g_safety_controller.callbacks.fault_callback(fault_info);
    }

    return true;
}

/**
 * @brief Execute emergency stop procedure
 */
void SafetyController_EmergencyStop(void)
{
    SafetyController_ExecuteEmergencyStop("Manual emergency stop");
}

/**
 * @brief Clear safety faults and attempt recovery
 * @return bool True if faults cleared and recovery initiated
 */
bool SafetyController_ClearFaults(void)
{
    if (!g_safety_controller.initialized)
    {
        return false;
    }

    // Can only clear faults in warning or fault states
    if (g_safety_controller.current_state == SAFETY_STATE_EMERGENCY_STOP)
    {
        return false;
    }

    // Mark all fault events as resolved
    for (uint8_t i = 0; i < g_safety_controller.fault_history.event_count; i++)
    {
        g_safety_controller.fault_history.events[i].resolved = true;
    }

    // Clear emergency stop if it was active
    g_safety_controller.emergency_stop_active = false;

    // Transition to safe state
    SafetyController_UpdateState(SAFETY_STATE_SAFE);

    return true;
}

/**
 * @brief Enable or disable system operation
 * @param enable System enable flag
 * @return bool True if system state changed successfully
 */
bool SafetyController_EnableSystem(bool enable)
{
    if (!g_safety_controller.initialized)
    {
        return false;
    }

    // Cannot enable system in fault or emergency stop states
    if (enable && (g_safety_controller.current_state == SAFETY_STATE_FAULT ||
                   g_safety_controller.current_state == SAFETY_STATE_EMERGENCY_STOP))
    {
        return false;
    }

    g_safety_controller.system_enabled = enable;

    if (enable)
    {
        SafetyController_UpdateState(SAFETY_STATE_SAFE);
    }
    else
    {
        SafetyController_UpdateState(SAFETY_STATE_DISABLED);
    }

    return true;
}

/**
 * @brief Feed the safety watchdog
 */
void SafetyController_FeedWatchdog(void)
{
    if (!g_safety_controller.initialized || !g_safety_controller.watchdog.watchdog_enabled)
    {
        return;
    }

    g_safety_controller.watchdog.last_watchdog_feed_ms = HAL_GetTick();
}

/**
 * @brief Get current safety state
 * @return SafetyState_t Current safety state
 */
SafetyState_t SafetyController_GetState(void)
{
    return g_safety_controller.current_state;
}

/**
 * @brief Get safety statistics
 * @param stats Pointer to structure to store statistics
 * @return bool True if statistics retrieved successfully
 */
bool SafetyController_GetStatistics(SafetyStatistics_t *stats)
{
    if (!stats || !g_safety_controller.initialized)
    {
        return false;
    }

    stats->monitor_cycle_count = g_safety_controller.statistics.monitor_cycle_count;
    stats->fault_count = g_safety_controller.fault_history.total_fault_count;
    stats->emergency_stop_count = g_safety_controller.statistics.emergency_stop_count;
    stats->watchdog_timeout_count = g_safety_controller.watchdog.watchdog_timeout_count;
    stats->uptime_ms = g_safety_controller.statistics.last_update_ms;
    stats->current_state = g_safety_controller.current_state;

    return true;
}

/**
 * @brief Get fault history
 * @param history Array to store fault history
 * @param max_events Maximum number of events to retrieve
 * @return uint8_t Number of events retrieved
 */
uint8_t SafetyController_GetFaultHistory(SafetyFaultHistory_t *history, uint8_t max_events)
{
    if (!history || !g_safety_controller.initialized)
    {
        return 0;
    }

    uint8_t events_to_copy = (g_safety_controller.fault_history.event_count < max_events) ? g_safety_controller.fault_history.event_count : max_events;

    // Copy most recent events
    for (uint8_t i = 0; i < events_to_copy; i++)
    {
        FaultEvent_t *src_event = &g_safety_controller.fault_history.events[i];
        SafetyFaultHistory_t *dst_event = &history[i];

        dst_event->fault_type = src_event->fault_type;
        dst_event->motor_id = src_event->motor_id;
        dst_event->timestamp_ms = src_event->timestamp_ms;
        dst_event->fault_data = src_event->fault_data;
        dst_event->resolved = src_event->resolved;
    }

    return events_to_copy;
}

/* =============================================================================
 * CALLBACK REGISTRATION FUNCTIONS
 * ============================================================================= */

/**
 * @brief Register safety fault callback
 */
bool SafetyController_RegisterFaultCallback(SafetyFaultCallback_t callback)
{
    if (!g_safety_controller.initialized)
    {
        return false;
    }

    g_safety_controller.callbacks.fault_callback = callback;
    return true;
}

/**
 * @brief Register safety state change callback
 */
bool SafetyController_RegisterStateCallback(SafetyStateChangeCallback_t callback)
{
    if (!g_safety_controller.initialized)
    {
        return false;
    }

    g_safety_controller.callbacks.state_callback = callback;
    return true;
}

/**
 * @brief Register emergency stop callback
 */
bool SafetyController_RegisterEmergencyCallback(SafetyEmergencyCallback_t callback)
{
    if (!g_safety_controller.initialized)
    {
        return false;
    }

    g_safety_controller.callbacks.emergency_callback = callback;
    return true;
}

/* =============================================================================
 * PRIVATE FUNCTION IMPLEMENTATIONS
 * ============================================================================= */

/**
 * @brief Initialize watchdog functionality
 */
static bool SafetyController_InitializeWatchdog(void)
{
    g_safety_controller.watchdog.watchdog_enabled = true;
    g_safety_controller.watchdog.last_watchdog_feed_ms = HAL_GetTick();
    g_safety_controller.watchdog.watchdog_timeout_count = 0;

    return true;
}

/**
 * @brief Update safety state and notify callbacks
 */
static void SafetyController_UpdateState(SafetyState_t new_state)
{
    SafetyState_t old_state = g_safety_controller.current_state;

    if (new_state != old_state)
    {
        g_safety_controller.current_state = new_state;

        // Notify state change callback
        if (g_safety_controller.callbacks.state_callback)
        {
            g_safety_controller.callbacks.state_callback(old_state, new_state);
        }
    }
}

/**
 * @brief Record fault event in history
 */
static void SafetyController_RecordFaultEvent(SafetyFaultType_t fault_type,
                                              uint8_t motor_id, uint32_t fault_data)
{
    FaultEvent_t *event = &g_safety_controller.fault_history.events[g_safety_controller.fault_history.write_index];

    event->fault_type = fault_type;
    event->motor_id = motor_id;
    event->timestamp_ms = HAL_GetTick();
    event->fault_data = fault_data;
    event->resolved = false;

    // Update indices
    g_safety_controller.fault_history.write_index =
        (g_safety_controller.fault_history.write_index + 1) % SAFETY_FAULT_HISTORY_SIZE;

    if (g_safety_controller.fault_history.event_count < SAFETY_FAULT_HISTORY_SIZE)
    {
        g_safety_controller.fault_history.event_count++;
    }

    g_safety_controller.fault_history.total_fault_count++;
}

/**
 * @brief Check if voltage is within safe range
 */
static bool SafetyController_CheckVoltageRange(uint32_t voltage_mv)
{
    return (voltage_mv >= g_safety_controller.configuration.limits.min_voltage_mv &&
            voltage_mv <= g_safety_controller.configuration.limits.max_voltage_mv);
}

/**
 * @brief Check if current is within safe range
 */
static bool SafetyController_CheckCurrentRange(uint32_t current_ma)
{
    return (current_ma <= g_safety_controller.configuration.limits.max_current_ma);
}

/**
 * @brief Check if temperature is within safe range
 */
static bool SafetyController_CheckTemperatureRange(int32_t temperature_c)
{
    return (temperature_c <= g_safety_controller.configuration.limits.max_temperature_c);
}

/**
 * @brief Validate safety limits configuration
 */
static bool SafetyController_ValidateLimits(const SafetyLimits_t *limits)
{
    if (!limits)
        return false;

    return (limits->min_voltage_mv > 0 &&
            limits->max_voltage_mv > limits->min_voltage_mv &&
            limits->max_current_ma > 0 &&
            limits->max_temperature_c > 0 &&
            limits->max_speed_rpm > 0);
}

/**
 * @brief Validate safety thresholds configuration
 */
static bool SafetyController_ValidateThresholds(const SafetyThresholds_t *thresholds)
{
    if (!thresholds)
        return false;

    return (thresholds->overcurrent_threshold_ma > 0 &&
            thresholds->overvoltage_threshold_mv > 0 &&
            thresholds->undervoltage_threshold_mv > 0 &&
            thresholds->overtemperature_threshold_c > 0 &&
            thresholds->stall_detection_time_ms > 0 &&
            thresholds->fault_recovery_time_ms > 0);
}

/**
 * @brief Execute emergency stop procedure
 */
static void SafetyController_ExecuteEmergencyStop(const char *reason)
{
    g_safety_controller.emergency_stop_active = true;
    g_safety_controller.statistics.emergency_stop_count++;

    // Update state
    SafetyController_UpdateState(SAFETY_STATE_EMERGENCY_STOP);

    // Notify emergency callback
    if (g_safety_controller.callbacks.emergency_callback)
    {
        g_safety_controller.callbacks.emergency_callback(reason);
    }

    // Record emergency stop as a fault event
    SafetyController_RecordFaultEvent(SAFETY_FAULT_EMERGENCY_STOP, 0,
                                      g_safety_controller.statistics.emergency_stop_count);
}

/**
 * @brief Check for watchdog timeout
 */
static void SafetyController_CheckWatchdogTimeout(void)
{
    if (!g_safety_controller.watchdog.watchdog_enabled)
    {
        return;
    }

    uint32_t current_time = HAL_GetTick();
    uint32_t time_since_feed = current_time - g_safety_controller.watchdog.last_watchdog_feed_ms;

    if (time_since_feed > SAFETY_WATCHDOG_TIMEOUT_MS)
    {
        g_safety_controller.watchdog.watchdog_timeout_count++;
        SafetyController_RecordFaultEvent(SAFETY_FAULT_WATCHDOG_TIMEOUT, 0, time_since_feed);
        SafetyController_ExecuteEmergencyStop("Watchdog timeout");
    }
}