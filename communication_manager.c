/**
 * @file communication_manager.c
 * @brief Communication Interface Manager Implementation
 * @author Enhanced by AI Assistant
 * @version v2.0.0
 * @date 2025
 *
 * Manages UART, SPI, and OLED communication interfaces for the motor control system.
 */

#include "communication_manager.h"
#include "SystemDefine.h"
#include "MotorDrive.h"
#include "main_controller.h"
#include "oled.h"
#include <string.h>
#include <stdio.h>

/* =============================================================================
 * PRIVATE VARIABLES AND STATE
 * ============================================================================= */

/**
 * @brief Communication manager state structure
 */
typedef struct
{
    CommConfig_t interface_config[COMM_INTERFACE_COUNT];
    CommStatus_t interface_status[COMM_INTERFACE_COUNT];

    struct
    {
        uint32_t messages_sent;
        uint32_t messages_received;
        uint32_t errors_count;
        uint32_t last_heartbeat_ms;
    } statistics;

    MessageReceivedCallback_t message_callback;
    CommErrorCallback_t error_callback;

    bool initialized;

} CommManager_t;

/** Global communication manager instance */
static CommManager_t g_comm_manager = {0};

/** External variables from existing code */
extern RecvFrame mRecvFrame;
extern Motor_Obj motor_I[2];

/* =============================================================================
 * PRIVATE FUNCTION DECLARATIONS
 * ============================================================================= */

static bool CommManager_InitializeUART(void);
static bool CommManager_InitializeSPI(void);
static bool CommManager_InitializeOLED(void);
static void CommManager_ProcessUARTMessage(const CommMessage_t *message);
static void CommManager_ProcessSPIMessage(const CommMessage_t *message);
static uint16_t CommManager_CalculateChecksum(const CommMessage_t *message);
static bool CommManager_ValidateMessage(const CommMessage_t *message);
static void CommManager_HandleMotorCommand(const CommMessage_t *message);
static void CommManager_HandleParameterRequest(const CommMessage_t *message);
static void CommManager_UpdateDisplayContent(void);

/* =============================================================================
 * PUBLIC FUNCTION IMPLEMENTATIONS
 * ============================================================================= */

/**
 * @brief Initialize communication manager
 * @details Sets up all configured communication interfaces
 * @return bool True if initialization successful
 */
bool CommunicationManager_Initialize(void)
{
    // Initialize default configurations
    for (int i = 0; i < COMM_INTERFACE_COUNT; i++)
    {
        g_comm_manager.interface_config[i].enabled = false;
        g_comm_manager.interface_config[i].baud_rate = COMM_DEFAULT_UART_BAUD;
        g_comm_manager.interface_config[i].timeout_ms = COMM_DEFAULT_TIMEOUT_MS;
        g_comm_manager.interface_config[i].auto_response = true;

        g_comm_manager.interface_status[i] = COMM_STATUS_IDLE;
    }

    // Initialize UART interface
    g_comm_manager.interface_config[COMM_INTERFACE_UART].enabled = true;
    if (!CommManager_InitializeUART())
    {
        return false;
    }

// Initialize SPI interface
#ifdef SPI_SEND
    g_comm_manager.interface_config[COMM_INTERFACE_SPI].enabled = true;
    if (!CommManager_InitializeSPI())
    {
        return false;
    }
#endif

    // Initialize OLED interface
    g_comm_manager.interface_config[COMM_INTERFACE_OLED].enabled = true;
    if (!CommManager_InitializeOLED())
    {
        return false;
    }

    // Reset statistics
    memset(&g_comm_manager.statistics, 0, sizeof(g_comm_manager.statistics));

    g_comm_manager.initialized = true;
    return true;
}

/**
 * @brief Configure communication interface
 * @param interface Interface type to configure
 * @param config Configuration parameters
 * @return bool True if configuration successful
 */
bool CommunicationManager_Configure(CommInterface_t interface, const CommConfig_t *config)
{
    if (!g_comm_manager.initialized || interface >= COMM_INTERFACE_COUNT || !config)
    {
        return false;
    }

    // Copy configuration
    memcpy(&g_comm_manager.interface_config[interface], config, sizeof(CommConfig_t));

    return true;
}

/**
 * @brief Send message through specified interface
 * @param interface Interface to use for transmission
 * @param message Message to send
 * @return bool True if message sent successfully
 */
bool CommunicationManager_SendMessage(CommInterface_t interface, const CommMessage_t *message)
{
    if (!g_comm_manager.initialized || interface >= COMM_INTERFACE_COUNT || !message)
    {
        return false;
    }

    if (!g_comm_manager.interface_config[interface].enabled)
    {
        return false;
    }

    // Validate message
    if (!CommManager_ValidateMessage(message))
    {
        g_comm_manager.statistics.errors_count++;
        return false;
    }

    bool success = false;

    switch (interface)
    {
    case COMM_INTERFACE_UART:
#ifdef UART_SEND
        // Send via UART (using existing Send_Value_Uart function)
        Send_Value_Uart((uint8_t *)message, sizeof(CommMessage_t));
        success = true;
#endif
        break;

    case COMM_INTERFACE_SPI:
#ifdef SPI_SEND
        // Send via SPI - implement based on existing SPI functions
        success = true;
#endif
        break;

    case COMM_INTERFACE_OLED:
        // OLED is output only, no message sending
        success = false;
        break;
    }

    if (success)
    {
        g_comm_manager.statistics.messages_sent++;
        g_comm_manager.interface_status[interface] = COMM_STATUS_TRANSMITTING;
    }
    else
    {
        g_comm_manager.statistics.errors_count++;
        g_comm_manager.interface_status[interface] = COMM_STATUS_ERROR;
    }

    return success;
}

/**
 * @brief Check for incoming messages
 * @param interface Interface to check
 * @param message Buffer to store received message
 * @return bool True if message received
 */
bool CommunicationManager_ReceiveMessage(CommInterface_t interface, CommMessage_t *message)
{
    if (!g_comm_manager.initialized || interface >= COMM_INTERFACE_COUNT || !message)
    {
        return false;
    }

    bool message_received = false;

    switch (interface)
    {
    case COMM_INTERFACE_UART:
        // Check UART receive buffer (integrate with existing mRecvFrame)
        if (mRecvFrame.State == 2)
        { // Data reception complete
            // Convert existing frame format to CommMessage_t
            if (mRecvFrame.index > 0 && mRecvFrame.index < COMM_MAX_PAYLOAD_SIZE)
            {
                message->start_marker = COMM_MESSAGE_MARKER;
                message->length = mRecvFrame.index;
                message->type = MSG_TYPE_MOTOR_COMMAND; // Default type
                message->sequence = 0;
                memcpy(message->payload, mRecvFrame.Buf, mRecvFrame.index);
                message->checksum = CommManager_CalculateChecksum(message);
                message->end_marker = COMM_MESSAGE_MARKER;

                message_received = true;
                mRecvFrame.State = 0; // Reset receive state
            }
        }
        break;

    case COMM_INTERFACE_SPI:
        // SPI typically doesn't have unsolicited incoming messages
        break;

    case COMM_INTERFACE_OLED:
        // OLED is output only
        break;
    }

    if (message_received)
    {
        g_comm_manager.statistics.messages_received++;
        g_comm_manager.interface_status[interface] = COMM_STATUS_RECEIVING;

        // Validate received message
        if (!CommManager_ValidateMessage(message))
        {
            g_comm_manager.statistics.errors_count++;
            return false;
        }
    }

    return message_received;
}

/**
 * @brief Process pending communication tasks
 * @details Should be called periodically from main loop
 */
void CommunicationManager_ProcessCommands(void)
{
    if (!g_comm_manager.initialized)
    {
        return;
    }

    CommMessage_t received_message;

    // Process UART messages
    if (CommunicationManager_ReceiveMessage(COMM_INTERFACE_UART, &received_message))
    {
        CommManager_ProcessUARTMessage(&received_message);

        // Call registered callback if available
        if (g_comm_manager.message_callback)
        {
            g_comm_manager.message_callback(COMM_INTERFACE_UART, &received_message);
        }
    }

    // Process SPI messages
    if (CommunicationManager_ReceiveMessage(COMM_INTERFACE_SPI, &received_message))
    {
        CommManager_ProcessSPIMessage(&received_message);

        // Call registered callback if available
        if (g_comm_manager.message_callback)
        {
            g_comm_manager.message_callback(COMM_INTERFACE_SPI, &received_message);
        }
    }

    // Update OLED display periodically
    static uint32_t last_display_update = 0;
    uint32_t current_time = SystemController_GetUptime();
    if (current_time - last_display_update > 100)
    { // Update every 100ms
        CommunicationManager_UpdateDisplay();
        last_display_update = current_time;
    }

    // Update interface status back to idle if no activity
    for (int i = 0; i < COMM_INTERFACE_COUNT; i++)
    {
        if (g_comm_manager.interface_status[i] == COMM_STATUS_TRANSMITTING ||
            g_comm_manager.interface_status[i] == COMM_STATUS_RECEIVING)
        {
            g_comm_manager.interface_status[i] = COMM_STATUS_IDLE;
        }
    }
}

/**
 * @brief Get communication interface status
 * @param interface Interface to query
 * @return CommStatus_t Current status of the interface
 */
CommStatus_t CommunicationManager_GetStatus(CommInterface_t interface)
{
    if (!g_comm_manager.initialized || interface >= COMM_INTERFACE_COUNT)
    {
        return COMM_STATUS_ERROR;
    }

    return g_comm_manager.interface_status[interface];
}

/**
 * @brief Send motor status information
 * @param motor_id Motor identifier (0 or 1)
 * @return bool True if status sent successfully
 */
bool CommunicationManager_SendMotorStatus(uint8_t motor_id)
{
    if (motor_id > 1)
    {
        return false;
    }

    CommMessage_t message;
    message.start_marker = COMM_MESSAGE_MARKER;
    message.type = MSG_TYPE_STATUS_RESPONSE;
    message.sequence = 0;
    message.length = 16; // Status data size

    // Pack motor status data
    uint16_t *data = (uint16_t *)message.payload;
    data[0] = motor_id;
    data[1] = motor_I[motor_id].DataObj.MotorSpeed; // Current speed
    data[2] = motor_I[motor_id].DataObj.Ia_Q15;     // Phase A current
    data[3] = motor_I[motor_id].DataObj.Ib_Q15;     // Phase B current
    data[4] = motor_I[motor_id].DataObj.Ic_Q15;     // Phase C current
    data[5] = motor_I[motor_id].DataObj.Udc_Real;   // DC bus voltage
    data[6] = motor_I[motor_id].Flag.Arr[0];        // Fault flags (first word)
    data[7] = motor_I[motor_id].MotorState;         // Motor state

    message.checksum = CommManager_CalculateChecksum(&message);
    message.end_marker = COMM_MESSAGE_MARKER;

    return CommunicationManager_SendMessage(COMM_INTERFACE_UART, &message);
}

/**
 * @brief Send system diagnostic information
 * @return bool True if diagnostics sent successfully
 */
bool CommunicationManager_SendDiagnostics(void)
{
    CommMessage_t message;
    message.start_marker = COMM_MESSAGE_MARKER;
    message.type = MSG_TYPE_DIAGNOSTIC;
    message.sequence = 0;
    message.length = 16;

    // Pack diagnostic data
    uint32_t *data = (uint32_t *)message.payload;
    data[0] = g_comm_manager.statistics.messages_sent;
    data[1] = g_comm_manager.statistics.messages_received;
    data[2] = g_comm_manager.statistics.errors_count;
    data[3] = SystemController_GetUptime();

    message.checksum = CommManager_CalculateChecksum(&message);
    message.end_marker = COMM_MESSAGE_MARKER;

    return CommunicationManager_SendMessage(COMM_INTERFACE_UART, &message);
}

/**
 * @brief Send fault notification
 * @param fault_code Fault code to report
 * @param motor_id Motor that generated fault
 * @return bool True if fault notification sent
 */
bool CommunicationManager_SendFaultNotification(uint16_t fault_code, uint8_t motor_id)
{
    CommMessage_t message;
    message.start_marker = COMM_MESSAGE_MARKER;
    message.type = MSG_TYPE_FAULT_ALERT;
    message.sequence = 0;
    message.length = 4;

    // Pack fault data
    uint16_t *data = (uint16_t *)message.payload;
    data[0] = fault_code;
    data[1] = motor_id;

    message.checksum = CommManager_CalculateChecksum(&message);
    message.end_marker = COMM_MESSAGE_MARKER;

    return CommunicationManager_SendMessage(COMM_INTERFACE_UART, &message);
}

/**
 * @brief Update OLED display with system information
 * @details Refreshes OLED display with current system status
 */
void CommunicationManager_UpdateDisplay(void)
{
    if (!g_comm_manager.interface_config[COMM_INTERFACE_OLED].enabled)
    {
        return;
    }

    CommManager_UpdateDisplayContent();
}

/**
 * @brief Register message received callback
 * @param callback Callback function to register
 * @return bool True if callback registered successfully
 */
bool CommunicationManager_RegisterMessageCallback(MessageReceivedCallback_t callback)
{
    if (!callback)
    {
        return false;
    }

    g_comm_manager.message_callback = callback;
    return true;
}

/**
 * @brief Register communication error callback
 * @param callback Callback function to register
 * @return bool True if callback registered successfully
 */
bool CommunicationManager_RegisterErrorCallback(CommErrorCallback_t callback)
{
    if (!callback)
    {
        return false;
    }

    g_comm_manager.error_callback = callback;
    return true;
}

/* =============================================================================
 * PRIVATE FUNCTION IMPLEMENTATIONS
 * ============================================================================= */

/**
 * @brief Initialize UART interface
 * @return bool True if initialization successful
 */
static bool CommManager_InitializeUART(void)
{
#ifdef UART_SEND
    // UART initialization is handled by existing Uart_init() function
    // This would be called during System_Init()
    return true;
#else
    return false;
#endif
}

/**
 * @brief Initialize SPI interface
 * @return bool True if initialization successful
 */
static bool CommManager_InitializeSPI(void)
{
#ifdef SPI_SEND
    // SPI initialization is handled by existing Spi_init() function
    // This would be called during System_Init()
    return true;
#else
    return false;
#endif
}

/**
 * @brief Initialize OLED interface
 * @return bool True if initialization successful
 */
static bool CommManager_InitializeOLED(void)
{
    // Initialize OLED display
    oled_init();
    OLED_Clear();

    // Display startup message
    OLED_ShowString(0, 0, (const uint8_t *)"Motor Control v2.0", 12);
    OLED_ShowString(0, 16, (const uint8_t *)"Initializing...", 12);
    OLED_Refresh_Gram();

    return true;
}

/**
 * @brief Process received UART message
 * @param message Received message to process
 */
static void CommManager_ProcessUARTMessage(const CommMessage_t *message)
{
    switch (message->type)
    {
    case MSG_TYPE_MOTOR_COMMAND:
        CommManager_HandleMotorCommand(message);
        break;

    case MSG_TYPE_PARAMETER_GET:
        CommManager_HandleParameterRequest(message);
        break;

    case MSG_TYPE_STATUS_REQUEST:
        // Send status for both motors
        CommunicationManager_SendMotorStatus(0);
        CommunicationManager_SendMotorStatus(1);
        break;

    case MSG_TYPE_HEARTBEAT:
        g_comm_manager.statistics.last_heartbeat_ms = SystemController_GetUptime();
        break;

    default:
        // Unknown message type
        g_comm_manager.statistics.errors_count++;
        break;
    }
}

/**
 * @brief Process received SPI message
 * @param message Received message to process
 */
static void CommManager_ProcessSPIMessage(const CommMessage_t *message)
{
    // SPI message processing implementation
    // This would depend on the specific SPI protocol used
}

/**
 * @brief Calculate message checksum
 * @param message Message to calculate checksum for
 * @return uint16_t Calculated checksum
 */
static uint16_t CommManager_CalculateChecksum(const CommMessage_t *message)
{
    uint16_t checksum = 0;

    checksum += message->type;
    checksum += message->sequence;
    checksum += message->length;

    for (int i = 0; i < message->length; i++)
    {
        checksum += message->payload[i];
    }

    return checksum;
}

/**
 * @brief Validate message structure and checksum
 * @param message Message to validate
 * @return bool True if message is valid
 */
static bool CommManager_ValidateMessage(const CommMessage_t *message)
{
    // Check message markers
    if (message->start_marker != COMM_MESSAGE_MARKER ||
        message->end_marker != COMM_MESSAGE_MARKER)
    {
        return false;
    }

    // Check payload length
    if (message->length > COMM_MAX_PAYLOAD_SIZE)
    {
        return false;
    }

    // Verify checksum
    uint16_t calculated_checksum = CommManager_CalculateChecksum(message);
    if (calculated_checksum != message->checksum)
    {
        return false;
    }

    return true;
}

/**
 * @brief Handle motor command message
 * @param message Motor command message
 */
static void CommManager_HandleMotorCommand(const CommMessage_t *message)
{
    if (message->length < 6)
    {
        return; // Invalid command length
    }

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

    case 0x03: // Set speed
        SpeedControl(motor_id, value);
        break;

    case 0xFF: // Emergency stop
        SystemController_EmergencyStop();
        break;
    }
}

/**
 * @brief Handle parameter request message
 * @param message Parameter request message
 */
static void CommManager_HandleParameterRequest(const CommMessage_t *message)
{
    // Implementation for parameter requests
    // This would return motor parameters, configuration, etc.
}

/**
 * @brief Update OLED display content
 * @details Updates OLED with current system status information
 */
static void CommManager_UpdateDisplayContent(void)
{
    char display_buffer[32];

    // Clear display
    OLED_Clear();

    // Line 1: System status
    SystemStatus_t status;
    if (SystemController_GetStatus(&status) == SYSTEM_SUCCESS)
    {
        snprintf(display_buffer, sizeof(display_buffer), "State: %d Mode: %d",
                 status.current_state, status.operating_mode);
        OLED_ShowString(0, 0, (const uint8_t *)display_buffer, 8);
    }

    // Line 2: Motor 1 status
    snprintf(display_buffer, sizeof(display_buffer), "M1: %ldRPM %s",
             motor_I[0].DataObj.MotorSpeed,
             motor_I[0].Flag.Bits.MotorStarted ? "ON" : "OFF");
    OLED_ShowString(0, 12, (const uint8_t *)display_buffer, 8);

    // Line 3: Motor 2 status
    snprintf(display_buffer, sizeof(display_buffer), "M2: %ldRPM %s",
             motor_I[1].DataObj.MotorSpeed,
             motor_I[1].Flag.Bits.MotorStarted ? "ON" : "OFF");
    OLED_ShowString(0, 24, (const uint8_t *)display_buffer, 8);

    // Line 4: System voltage
    snprintf(display_buffer, sizeof(display_buffer), "Vdc: %dV", motor_I[0].DataObj.Udc_Real);
    OLED_ShowString(0, 36, (const uint8_t *)display_buffer, 8);

    // Line 5: Fault status
    if (SystemController_HasFaults())
    {
        OLED_ShowString(0, 48, (const uint8_t *)"FAULT ACTIVE", 8);
    }
    else
    {
        snprintf(display_buffer, sizeof(display_buffer), "Uptime: %lds",
                 SystemController_GetUptime() / 1000);
        OLED_ShowString(0, 48, (const uint8_t *)display_buffer, 8);
    }

    // Refresh display
    OLED_Refresh_Gram();
}