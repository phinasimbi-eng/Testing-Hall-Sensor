/**
 * @file communication_manager.h
 * @brief Communication Interface Manager
 * @author Enhanced by AI Assistant
 * @version v2.0.0
 * @date 2025
 *
 * Unified interface for managing UART, SPI, and OLED communication
 * channels with the motor control system.
 */

#ifndef COMMUNICATION_MANAGER_H
#define COMMUNICATION_MANAGER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /* =============================================================================
     * TYPE DEFINITIONS
     * ============================================================================= */

    /**
     * @brief Communication interface types
     */
    typedef enum
    {
        COMM_INTERFACE_UART = 0, /**< UART serial interface */
        COMM_INTERFACE_SPI,      /**< SPI interface */
        COMM_INTERFACE_OLED,     /**< OLED display interface */
        COMM_INTERFACE_COUNT     /**< Total number of interfaces */
    } CommInterface_t;

    /**
     * @brief Communication status
     */
    typedef enum
    {
        COMM_STATUS_IDLE = 0,     /**< Interface idle */
        COMM_STATUS_TRANSMITTING, /**< Currently transmitting */
        COMM_STATUS_RECEIVING,    /**< Currently receiving */
        COMM_STATUS_ERROR         /**< Error condition */
    } CommStatus_t;

    /**
     * @brief Message types for communication
     */
    typedef enum
    {
        MSG_TYPE_STATUS_REQUEST = 0x01,  /**< Status information request */
        MSG_TYPE_STATUS_RESPONSE = 0x02, /**< Status information response */
        MSG_TYPE_MOTOR_COMMAND = 0x10,   /**< Motor control command */
        MSG_TYPE_PARAMETER_SET = 0x20,   /**< Parameter configuration */
        MSG_TYPE_PARAMETER_GET = 0x21,   /**< Parameter read request */
        MSG_TYPE_DIAGNOSTIC = 0x30,      /**< Diagnostic information */
        MSG_TYPE_FAULT_ALERT = 0x40,     /**< Fault notification */
        MSG_TYPE_HEARTBEAT = 0x50        /**< Heartbeat/keepalive */
    } MessageType_t;

    /**
     * @brief Communication message structure
     */
    typedef struct
    {
        uint8_t start_marker; /**< Message start marker (0x7E) */
        uint8_t length;       /**< Message payload length */
        uint8_t type;         /**< Message type */
        uint8_t sequence;     /**< Sequence number */
        uint8_t payload[64];  /**< Message payload data */
        uint16_t checksum;    /**< Message checksum */
        uint8_t end_marker;   /**< Message end marker (0x7E) */
    } CommMessage_t;

    /**
     * @brief Communication interface configuration
     */
    typedef struct
    {
        bool enabled;        /**< Interface enabled flag */
        uint32_t baud_rate;  /**< Baud rate (for UART) */
        uint16_t timeout_ms; /**< Communication timeout */
        bool auto_response;  /**< Automatic response enabled */
    } CommConfig_t;

/* =============================================================================
 * CONFIGURATION CONSTANTS
 * ============================================================================= */

/** Maximum message payload size */
#define COMM_MAX_PAYLOAD_SIZE (64)

/** Message start/end marker */
#define COMM_MESSAGE_MARKER (0x7E)

/** Default UART baud rate */
#define COMM_DEFAULT_UART_BAUD (115200)

/** Communication timeout (ms) */
#define COMM_DEFAULT_TIMEOUT_MS (1000)

/** Maximum retry attempts */
#define COMM_MAX_RETRY_COUNT (3)

    /* =============================================================================
     * PUBLIC FUNCTION DECLARATIONS
     * ============================================================================= */

    /**
     * @brief Initialize communication manager
     * @details Sets up all configured communication interfaces
     * @return bool True if initialization successful
     */
    bool CommunicationManager_Initialize(void);

    /**
     * @brief Configure communication interface
     * @param interface Interface type to configure
     * @param config Configuration parameters
     * @return bool True if configuration successful
     */
    bool CommunicationManager_Configure(CommInterface_t interface, const CommConfig_t *config);

    /**
     * @brief Send message through specified interface
     * @param interface Interface to use for transmission
     * @param message Message to send
     * @return bool True if message sent successfully
     */
    bool CommunicationManager_SendMessage(CommInterface_t interface, const CommMessage_t *message);

    /**
     * @brief Check for incoming messages
     * @param interface Interface to check
     * @param message Buffer to store received message
     * @return bool True if message received
     */
    bool CommunicationManager_ReceiveMessage(CommInterface_t interface, CommMessage_t *message);

    /**
     * @brief Process pending communication tasks
     * @details Should be called periodically from main loop
     */
    void CommunicationManager_ProcessCommands(void);

    /**
     * @brief Get communication interface status
     * @param interface Interface to query
     * @return CommStatus_t Current status of the interface
     */
    CommStatus_t CommunicationManager_GetStatus(CommInterface_t interface);

    /**
     * @brief Send motor status information
     * @param motor_id Motor identifier (0 or 1)
     * @return bool True if status sent successfully
     */
    bool CommunicationManager_SendMotorStatus(uint8_t motor_id);

    /**
     * @brief Send system diagnostic information
     * @return bool True if diagnostics sent successfully
     */
    bool CommunicationManager_SendDiagnostics(void);

    /**
     * @brief Send fault notification
     * @param fault_code Fault code to report
     * @param motor_id Motor that generated fault
     * @return bool True if fault notification sent
     */
    bool CommunicationManager_SendFaultNotification(uint16_t fault_code, uint8_t motor_id);

    /**
     * @brief Update OLED display with system information
     * @details Refreshes OLED display with current system status
     */
    void CommunicationManager_UpdateDisplay(void);

    /* =============================================================================
     * CALLBACK FUNCTION TYPES
     * ============================================================================= */

    /**
     * @brief Message received callback function type
     * @param interface Interface that received the message
     * @param message Received message
     */
    typedef void (*MessageReceivedCallback_t)(CommInterface_t interface, const CommMessage_t *message);

    /**
     * @brief Communication error callback function type
     * @param interface Interface with error
     * @param error_code Error code
     */
    typedef void (*CommErrorCallback_t)(CommInterface_t interface, uint16_t error_code);

    /**
     * @brief Register message received callback
     * @param callback Callback function to register
     * @return bool True if callback registered successfully
     */
    bool CommunicationManager_RegisterMessageCallback(MessageReceivedCallback_t callback);

    /**
     * @brief Register communication error callback
     * @param callback Callback function to register
     * @return bool True if callback registered successfully
     */
    bool CommunicationManager_RegisterErrorCallback(CommErrorCallback_t callback);

/* =============================================================================
 * UTILITY MACROS
 * ============================================================================= */

/** Calculate message checksum */
#define COMM_CALC_CHECKSUM(msg)                 \
    ((uint16_t)((msg)->type + (msg)->sequence + \
                /* sum payload bytes */ 0)) // Simplified for brevity

/** Validate message structure */
#define COMM_IS_VALID_MESSAGE(msg)                   \
    (((msg)->start_marker == COMM_MESSAGE_MARKER) && \
     ((msg)->end_marker == COMM_MESSAGE_MARKER) &&   \
     ((msg)->length <= COMM_MAX_PAYLOAD_SIZE))

#ifdef __cplusplus
}
#endif

#endif /* COMMUNICATION_MANAGER_H */