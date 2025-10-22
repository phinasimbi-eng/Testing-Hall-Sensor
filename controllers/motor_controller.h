/**
 * @file motor_controller.h
 * @brief Motor Control Subsystem Interface
 * @author Enhanced by AI Assistant
 * @version v2.0.0
 * @date 2025
 *
 * Dedicated motor control subsystem that handles all motor-related operations
 * including FOC control, Hall sensor processing, and motor state management.
 * This controller abstracts motor complexities from the main system controller.
 */

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

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
     * @brief Motor operating states
     */
    typedef enum
    {
        MOTOR_STATE_IDLE = 0, /**< Motor stopped and idle */
        MOTOR_STATE_STARTING, /**< Motor in startup sequence */
        MOTOR_STATE_RUNNING,  /**< Motor running normally */
        MOTOR_STATE_STOPPING, /**< Motor in shutdown sequence */
        MOTOR_STATE_FAULT,    /**< Motor in fault condition */
        MOTOR_STATE_LOCKED    /**< Motor locked/blocked */
    } MotorState_t;

    /**
     * @brief Motor control modes
     */
    typedef enum
    {
        MOTOR_CONTROL_SPEED = 0, /**< Speed control mode */
        MOTOR_CONTROL_TORQUE,    /**< Torque control mode */
        MOTOR_CONTROL_POSITION,  /**< Position control mode */
        MOTOR_CONTROL_VOLTAGE    /**< Voltage control mode (open loop) */
    } MotorControlMode_t;

    /**
     * @brief Motor direction
     */
    typedef enum
    {
        MOTOR_DIRECTION_FORWARD = 0, /**< Forward rotation */
        MOTOR_DIRECTION_REVERSE      /**< Reverse rotation */
    } MotorDirection_t;

    /**
     * @brief Motor parameters structure
     */
    typedef struct
    {
        uint8_t motor_id;            /**< Motor identifier (0 or 1) */
        uint8_t pole_pairs;          /**< Number of pole pairs */
        float resistance_ohms;       /**< Phase resistance (Ohms) */
        float inductance_henry;      /**< Phase inductance (Henry) */
        float flux_linkage;          /**< Flux linkage constant */
        uint16_t max_speed_rpm;      /**< Maximum speed (RPM) */
        uint16_t max_current_ma;     /**< Maximum current (mA) */
        uint16_t encoder_resolution; /**< Encoder resolution (pulses/rev) */
    } MotorParameters_t;

    /**
     * @brief Motor control configuration
     */
    typedef struct
    {
        MotorControlMode_t control_mode; /**< Control mode */
        MotorDirection_t direction;      /**< Rotation direction */

        struct
        {
            uint16_t kp;          /**< Proportional gain */
            uint16_t ki;          /**< Integral gain */
            uint16_t kd;          /**< Derivative gain */
            int32_t output_limit; /**< Output limit */
        } speed_controller;

        struct
        {
            uint16_t kp;          /**< Proportional gain */
            uint16_t ki;          /**< Integral gain */
            int32_t output_limit; /**< Output limit */
        } current_controller;

        struct
        {
            uint16_t pwm_frequency_hz; /**< PWM switching frequency */
            uint16_t dead_time_ns;     /**< Dead time in nanoseconds */
            bool brake_on_stop;        /**< Enable braking when stopped */
        } pwm_config;

    } MotorControlConfig_t;

    /**
     * @brief Motor feedback data
     */
    typedef struct
    {
        int32_t actual_speed_rpm;   /**< Actual motor speed (RPM) */
        int32_t target_speed_rpm;   /**< Target speed (RPM) */
        int16_t phase_currents[3];  /**< Phase currents A, B, C (Q15 format) */
        uint16_t dc_bus_voltage;    /**< DC bus voltage */
        int16_t rotor_angle;        /**< Rotor electrical angle */
        uint8_t hall_state;         /**< Hall sensor state */
        uint16_t motor_temperature; /**< Motor temperature */
        MotorState_t current_state; /**< Current motor state */
    } MotorFeedback_t;

    /**
     * @brief Motor command structure
     */
    typedef struct
    {
        bool enable;                /**< Enable/disable motor */
        int32_t target_value;       /**< Target value (speed/torque/position) */
        MotorDirection_t direction; /**< Rotation direction */
        bool emergency_stop;        /**< Emergency stop command */
        bool brake_engage;          /**< Engage brake */
    } MotorCommand_t;

    /**
     * @brief Motor fault flags
     */
    typedef union
    {
        struct
        {
            uint16_t overcurrent : 1;     /**< Overcurrent fault */
            uint16_t overvoltage : 1;     /**< Overvoltage fault */
            uint16_t undervoltage : 1;    /**< Undervoltage fault */
            uint16_t overtemperature : 1; /**< Overtemperature fault */
            uint16_t hall_fault : 1;      /**< Hall sensor fault */
            uint16_t encoder_fault : 1;   /**< Encoder fault */
            uint16_t stall_detected : 1;  /**< Motor stall detected */
            uint16_t phase_loss : 1;      /**< Phase loss detected */
            uint16_t communication : 1;   /**< Communication fault */
            uint16_t configuration : 1;   /**< Configuration error */
            uint16_t reserved : 6;        /**< Reserved bits */
        } bits;
        uint16_t all; /**< All fault flags as uint16 */
    } MotorFaultFlags_t;

/* =============================================================================
 * CONFIGURATION CONSTANTS
 * ============================================================================= */

/** Maximum number of motors supported */
#define MOTOR_MAX_COUNT (2)

/** Default PWM frequency (Hz) */
#define MOTOR_DEFAULT_PWM_FREQUENCY (20000)

/** Maximum speed in RPM */
#define MOTOR_MAX_SPEED_RPM (5000)

/** Minimum speed in RPM */
#define MOTOR_MIN_SPEED_RPM (50)

/** Control loop frequency (Hz) */
#define MOTOR_CONTROL_FREQUENCY_HZ (10000)

/** Fault monitoring interval (ms) */
#define MOTOR_FAULT_CHECK_INTERVAL_MS (10)

    /* =============================================================================
     * PUBLIC FUNCTION DECLARATIONS
     * ============================================================================= */

    /**
     * @brief Initialize motor controller subsystem
     * @details Initializes all motor control hardware and algorithms
     * @return bool True if initialization successful
     */
    bool MotorController_Initialize(void);

    /**
     * @brief Configure motor parameters and control settings
     * @param motor_id Motor identifier (0 or 1)
     * @param parameters Motor parameters structure
     * @param config Control configuration structure
     * @return bool True if configuration successful
     */
    bool MotorController_Configure(uint8_t motor_id,
                                   const MotorParameters_t *parameters,
                                   const MotorControlConfig_t *config);

    /**
     * @brief Execute motor control command
     * @param motor_id Motor identifier (0 or 1)
     * @param command Motor command structure
     * @return bool True if command executed successfully
     */
    bool MotorController_ExecuteCommand(uint8_t motor_id, const MotorCommand_t *command);

    /**
     * @brief Get motor feedback data
     * @param motor_id Motor identifier (0 or 1)
     * @param feedback Pointer to feedback structure to populate
     * @return bool True if feedback data retrieved successfully
     */
    bool MotorController_GetFeedback(uint8_t motor_id, MotorFeedback_t *feedback);

    /**
     * @brief Get motor fault status
     * @param motor_id Motor identifier (0 or 1)
     * @param faults Pointer to fault flags structure to populate
     * @return bool True if fault data retrieved successfully
     */
    bool MotorController_GetFaultStatus(uint8_t motor_id, MotorFaultFlags_t *faults);

    /**
     * @brief Clear motor faults
     * @param motor_id Motor identifier (0 or 1)
     * @return bool True if faults cleared successfully
     */
    bool MotorController_ClearFaults(uint8_t motor_id);

    /**
     * @brief Emergency stop all motors
     * @details Immediately stops all motors and disables PWM outputs
     */
    void MotorController_EmergencyStopAll(void);

    /**
     * @brief Update motor control algorithms
     * @details Should be called from high-frequency interrupt (10kHz)
     * This function executes FOC control loops for all motors
     */
    void MotorController_UpdateControlLoops(void);

    /**
     * @brief Monitor motor conditions and detect faults
     * @details Should be called periodically from main loop
     */
    void MotorController_MonitorConditions(void);

    /**
     * @brief Calibrate motor sensors
     * @param motor_id Motor identifier (0 or 1)
     * @return bool True if calibration successful
     */
    bool MotorController_CalibrateMotor(uint8_t motor_id);

    /**
     * @brief Set motor control mode
     * @param motor_id Motor identifier (0 or 1)
     * @param mode New control mode
     * @return bool True if mode set successfully
     */
    bool MotorController_SetControlMode(uint8_t motor_id, MotorControlMode_t mode);

    /**
     * @brief Get motor controller status
     * @param motor_id Motor identifier (0 or 1)
     * @return MotorState_t Current motor state
     */
    MotorState_t MotorController_GetState(uint8_t motor_id);

    /**
     * @brief Enable/disable motor
     * @param motor_id Motor identifier (0 or 1)
     * @param enable Enable flag
     * @return bool True if command successful
     */
    bool MotorController_Enable(uint8_t motor_id, bool enable);

    /**
     * @brief Set motor speed target
     * @param motor_id Motor identifier (0 or 1)
     * @param speed_rpm Target speed in RPM
     * @return bool True if command successful
     */
    bool MotorController_SetSpeed(uint8_t motor_id, int32_t speed_rpm);

    /**
     * @brief Set motor torque target
     * @param motor_id Motor identifier (0 or 1)
     * @param torque_percent Target torque in percentage (-100 to +100)
     * @return bool True if command successful
     */
    bool MotorController_SetTorque(uint8_t motor_id, int32_t torque_percent);

    /* =============================================================================
     * CALLBACK FUNCTION TYPES
     * ============================================================================= */

    /**
     * @brief Motor state change callback function type
     * @param motor_id Motor identifier
     * @param old_state Previous motor state
     * @param new_state New motor state
     */
    typedef void (*MotorStateChangeCallback_t)(uint8_t motor_id, MotorState_t old_state, MotorState_t new_state);

    /**
     * @brief Motor fault callback function type
     * @param motor_id Motor identifier
     * @param fault_flags Active fault flags
     */
    typedef void (*MotorFaultCallback_t)(uint8_t motor_id, MotorFaultFlags_t fault_flags);

    /**
     * @brief Register motor state change callback
     * @param callback Callback function to register
     * @return bool True if callback registered successfully
     */
    bool MotorController_RegisterStateCallback(MotorStateChangeCallback_t callback);

    /**
     * @brief Register motor fault callback
     * @param callback Callback function to register
     * @return bool True if callback registered successfully
     */
    bool MotorController_RegisterFaultCallback(MotorFaultCallback_t callback);

/* =============================================================================
 * UTILITY MACROS
 * ============================================================================= */

/** Check if motor ID is valid */
#define MOTOR_IS_VALID_ID(id) ((id) < MOTOR_MAX_COUNT)

/** Convert RPM to internal units */
#define MOTOR_RPM_TO_INTERNAL(rpm) ((int32_t)((rpm) * 65536 / 60))

/** Convert internal units to RPM */
#define MOTOR_INTERNAL_TO_RPM(units) ((int32_t)((units) * 60 / 65536))

/** Check if speed is within valid range */
#define MOTOR_IS_VALID_SPEED(speed) (((speed) >= -MOTOR_MAX_SPEED_RPM) && \
                                     ((speed) <= MOTOR_MAX_SPEED_RPM))

/** Check if motor has any faults */
#define MOTOR_HAS_FAULTS(faults) ((faults).all != 0)

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROLLER_H */