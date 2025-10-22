/**
 * @file communication_controller.c
 * @brief Communication Controller Implementation
 * @author Enhanced by AI Assistant
 * @version v2.0.0
 * @date 2025
 *
 * Communication controller that manages all communication interfaces
 * including UART, SPI, I2C, and OLED display. Provides abstracted messaging
 * layer for system communication and data logging.
 */

#include "controllers/communication_controller.h"
#include "communication_manager.h"
#include "SystemDefine.h"
#include "Uart.h"
#include "Spi.h"
#include "Data_Uart.h"
#include "oled.h"
#include "myiic.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* =============================================================================
 * PRIVATE CONSTANTS AND DEFINITIONS
 * ============================================================================= */

#define COMM_CONTROLLER_BUFFER_SIZE 512  /**< Internal buffer size */
#define COMM_CONTROLLER_MESSAGE_QUEUE 32 /**< Message queue size */
#define COMM_CONTROLLER_LOG_BUFFER 256   /**< Log buffer size */

/* =============================================================================
 * PRIVATE TYPES AND STRUCTURES
 * ============================================================================= */

/**
 * @brief Communication controller global state
 */
typedef struct
{
    bool initialized;
    bool uart_enabled;
    bool spi_enabled;
    bool i2c_enabled;
    bool display_enabled;

    struct
    {
        CommDataCallback_t data_callback;
        CommErrorCallback_t error_callback;
        CommStatusCallback_t status_callback;
    } callbacks;

    struct
    {
        uint32_t uart_tx_count;
        uint32_t uart_rx_count;
        uint32_t spi_transfers;
        uint32_t i2c_transfers;
        uint32_t display_updates;
        uint32_t error_count;
    } statistics;

    char log_buffer[COMM_CONTROLLER_LOG_BUFFER];

} CommunicationControllerState_t;

/** Global communication controller state */
static CommunicationControllerState_t g_comm_controller = {0};

/* =============================================================================
 * PRIVATE FUNCTION DECLARATIONS
 * ============================================================================= */

static bool CommController_InitializeUart(void);
static bool CommController_InitializeSpi(void);
static bool CommController_InitializeI2c(void);
static bool CommController_InitializeDisplay(void);

/* =============================================================================
 * PUBLIC FUNCTION IMPLEMENTATIONS
 * ============================================================================= */

/**
 * @brief Initialize communication controller
 * @return bool True if initialization successful
 */
bool CommunicationController_Initialize(void)
{
    // Initialize controller state
    memset(&g_comm_controller, 0, sizeof(CommunicationControllerState_t));

    // Initialize UART interface
    if (!CommController_InitializeUart())
    {
        return false;
    }

    // Initialize SPI interface
    if (!CommController_InitializeSpi())
    {
        return false;
    }

    // Initialize I2C interface
    if (!CommController_InitializeI2c())
    {
        return false;
    }

    // Initialize display
    if (!CommController_InitializeDisplay())
    {
        return false;
    }

    g_comm_controller.initialized = true;

    // Send initialization message
    CommunicationController_LogMessage(COMM_LOG_INFO, "Communication controller initialized");

    return true;
}

/**
 * @brief Send data via UART
 * @param data Pointer to data buffer
 * @param length Number of bytes to send
 * @return bool True if data sent successfully
 */
bool CommunicationController_SendUart(const uint8_t *data, uint16_t length)
{
    if (!data || length == 0 || !g_comm_controller.initialized ||
        !g_comm_controller.uart_enabled)
    {
        return false;
    }

    g_comm_controller.statistics.uart_tx_count += length;

    // Use existing UART functions
    return Usart_SendData((char *)data, length) == 0;
}

/**
 * @brief Send formatted string via UART
 * @param format Printf-style format string
 * @param ... Variable arguments
 * @return bool True if string sent successfully
 */
bool CommunicationController_SendUartString(const char *format, ...)
{
    if (!format || !g_comm_controller.initialized || !g_comm_controller.uart_enabled)
    {
        return false;
    }

    va_list args;
    va_start(args, format);
    int length = vsnprintf(g_comm_controller.log_buffer,
                           COMM_CONTROLLER_LOG_BUFFER, format, args);
    va_end(args);

    if (length > 0)
    {
        return CommunicationController_SendUart((uint8_t *)g_comm_controller.log_buffer, length);
    }

    return false;
}

/**
 * @brief Send data via SPI
 * @param data Pointer to data to send
 * @param length Number of bytes to send
 * @param response Buffer for response data (can be NULL)
 * @return bool True if transfer successful
 */
bool CommunicationController_SendSpi(const uint8_t *data, uint16_t length, uint8_t *response)
{
    if (!data || length == 0 || !g_comm_controller.initialized ||
        !g_comm_controller.spi_enabled)
    {
        return false;
    }

    g_comm_controller.statistics.spi_transfers++;

    // Use existing SPI functions
    for (uint16_t i = 0; i < length; i++)
    {
        uint8_t rx_byte = SPI_ReadWriteByte(data[i]);
        if (response)
        {
            response[i] = rx_byte;
        }
    }

    return true;
}

/**
 * @brief Send data via I2C
 * @param device_addr I2C device address
 * @param data Pointer to data buffer
 * @param length Number of bytes to send
 * @return bool True if transfer successful
 */
bool CommunicationController_SendI2c(uint8_t device_addr, const uint8_t *data, uint16_t length)
{
    if (!data || length == 0 || !g_comm_controller.initialized ||
        !g_comm_controller.i2c_enabled)
    {
        return false;
    }

    g_comm_controller.statistics.i2c_transfers++;

    // Use existing I2C functions
    IIC_Start();

    // Send device address for write
    if (!IIC_Send_Byte((device_addr << 1) | 0))
    {
        IIC_Stop();
        g_comm_controller.statistics.error_count++;
        return false;
    }

    // Send data bytes
    for (uint16_t i = 0; i < length; i++)
    {
        if (!IIC_Send_Byte(data[i]))
        {
            IIC_Stop();
            g_comm_controller.statistics.error_count++;
            return false;
        }
    }

    IIC_Stop();
    return true;
}

/**
 * @brief Read data via I2C
 * @param device_addr I2C device address
 * @param buffer Buffer to store received data
 * @param length Number of bytes to read
 * @return bool True if transfer successful
 */
bool CommunicationController_ReadI2c(uint8_t device_addr, uint8_t *buffer, uint16_t length)
{
    if (!buffer || length == 0 || !g_comm_controller.initialized ||
        !g_comm_controller.i2c_enabled)
    {
        return false;
    }

    g_comm_controller.statistics.i2c_transfers++;

    // Use existing I2C functions
    IIC_Start();

    // Send device address for read
    if (!IIC_Send_Byte((device_addr << 1) | 1))
    {
        IIC_Stop();
        g_comm_controller.statistics.error_count++;
        return false;
    }

    // Read data bytes
    for (uint16_t i = 0; i < length; i++)
    {
        buffer[i] = IIC_Read_Byte(i < (length - 1)); // ACK for all but last byte
    }

    IIC_Stop();
    return true;
}

/**
 * @brief Display message on OLED
 * @param line Line number (0-7)
 * @param message Message string to display
 * @return bool True if message displayed successfully
 */
bool CommunicationController_DisplayMessage(uint8_t line, const char *message)
{
    if (!message || line >= 8 || !g_comm_controller.initialized ||
        !g_comm_controller.display_enabled)
    {
        return false;
    }

    g_comm_controller.statistics.display_updates++;

    // Use existing OLED functions
    OLED_ShowString(0, line * 8, (uint8_t *)message, 8);
    OLED_Refresh_Gram();

    return true;
}

/**
 * @brief Clear OLED display
 * @return bool True if display cleared successfully
 */
bool CommunicationController_ClearDisplay(void)
{
    if (!g_comm_controller.initialized || !g_comm_controller.display_enabled)
    {
        return false;
    }

    OLED_Clear();
    OLED_Refresh_Gram();

    return true;
}

/**
 * @brief Display system status on OLED
 * @param status System status information
 * @return bool True if status displayed successfully
 */
bool CommunicationController_DisplayStatus(const CommSystemStatus_t *status)
{
    if (!status || !g_comm_controller.initialized || !g_comm_controller.display_enabled)
    {
        return false;
    }

    char line_buffer[32];

    // Clear display first
    CommunicationController_ClearDisplay();

    // Line 0: Motor 1 speed
    snprintf(line_buffer, sizeof(line_buffer), "M1: %4d rpm", status->motor1_speed_rpm);
    CommunicationController_DisplayMessage(0, line_buffer);

    // Line 1: Motor 2 speed
    snprintf(line_buffer, sizeof(line_buffer), "M2: %4d rpm", status->motor2_speed_rpm);
    CommunicationController_DisplayMessage(1, line_buffer);

    // Line 2: System voltage
    snprintf(line_buffer, sizeof(line_buffer), "V: %2d.%dV",
             status->voltage_mv / 1000, (status->voltage_mv % 1000) / 100);
    CommunicationController_DisplayMessage(2, line_buffer);

    // Line 3: Temperature
    snprintf(line_buffer, sizeof(line_buffer), "T: %2dC", status->temperature_c);
    CommunicationController_DisplayMessage(3, line_buffer);

    // Line 4: System state
    const char *state_str = status->system_enabled ? (status->fault_active ? "FAULT" : "RUN") : "STOP";
    CommunicationController_DisplayMessage(4, state_str);

    return true;
}

/**
 * @brief Send formatted log message
 * @param level Log level
 * @param format Printf-style format string
 * @param ... Variable arguments
 * @return bool True if message logged successfully
 */
bool CommunicationController_LogMessage(CommLogLevel_t level, const char *format, ...)
{
    if (!format || !g_comm_controller.initialized || !g_comm_controller.uart_enabled)
    {
        return false;
    }

    // Format log message with timestamp and level
    uint32_t timestamp = HAL_GetTick();
    int prefix_len = snprintf(g_comm_controller.log_buffer, COMM_CONTROLLER_LOG_BUFFER,
                              "[%08lu] ", timestamp);

    // Add log level prefix
    const char *level_str;
    switch (level)
    {
    case COMM_LOG_ERROR:
        level_str = "ERROR: ";
        break;
    case COMM_LOG_WARNING:
        level_str = "WARN:  ";
        break;
    case COMM_LOG_INFO:
        level_str = "INFO:  ";
        break;
    case COMM_LOG_DEBUG:
        level_str = "DEBUG: ";
        break;
    default:
        level_str = "LOG:   ";
        break;
    }

    prefix_len += snprintf(g_comm_controller.log_buffer + prefix_len,
                           COMM_CONTROLLER_LOG_BUFFER - prefix_len, "%s", level_str);

    // Format user message
    va_list args;
    va_start(args, format);
    vsnprintf(g_comm_controller.log_buffer + prefix_len,
              COMM_CONTROLLER_LOG_BUFFER - prefix_len, format, args);
    va_end(args);

    // Add newline
    size_t len = strlen(g_comm_controller.log_buffer);
    if (len < COMM_CONTROLLER_LOG_BUFFER - 2)
    {
        g_comm_controller.log_buffer[len] = '\r';
        g_comm_controller.log_buffer[len + 1] = '\n';
        g_comm_controller.log_buffer[len + 2] = '\0';
    }

    // Send log message
    return CommunicationController_SendUart((uint8_t *)g_comm_controller.log_buffer,
                                            strlen(g_comm_controller.log_buffer));
}

/**
 * @brief Process incoming communication data
 */
void CommunicationController_ProcessData(void)
{
    if (!g_comm_controller.initialized)
    {
        return;
    }

    // Process incoming UART data if available
    // This integrates with existing UART interrupt handling

    // Process other communication interfaces as needed
}

/**
 * @brief Get communication statistics
 * @param stats Pointer to statistics structure
 * @return bool True if statistics retrieved successfully
 */
bool CommunicationController_GetStatistics(CommStatistics_t *stats)
{
    if (!stats || !g_comm_controller.initialized)
    {
        return false;
    }

    stats->uart_tx_bytes = g_comm_controller.statistics.uart_tx_count;
    stats->uart_rx_bytes = g_comm_controller.statistics.uart_rx_count;
    stats->spi_transfers = g_comm_controller.statistics.spi_transfers;
    stats->i2c_transfers = g_comm_controller.statistics.i2c_transfers;
    stats->display_updates = g_comm_controller.statistics.display_updates;
    stats->error_count = g_comm_controller.statistics.error_count;

    return true;
}

/**
 * @brief Enable or disable communication interface
 * @param interface Interface type
 * @param enable Enable flag
 * @return bool True if interface state changed successfully
 */
bool CommunicationController_EnableInterface(CommInterface_t interface, bool enable)
{
    if (!g_comm_controller.initialized)
    {
        return false;
    }

    switch (interface)
    {
    case COMM_INTERFACE_UART:
        g_comm_controller.uart_enabled = enable;
        break;

    case COMM_INTERFACE_SPI:
        g_comm_controller.spi_enabled = enable;
        break;

    case COMM_INTERFACE_I2C:
        g_comm_controller.i2c_enabled = enable;
        break;

    case COMM_INTERFACE_OLED:
        g_comm_controller.display_enabled = enable;
        if (!enable)
        {
            CommunicationController_ClearDisplay();
        }
        break;

    default:
        return false;
    }

    return true;
}

/* =============================================================================
 * CALLBACK REGISTRATION FUNCTIONS
 * ============================================================================= */

/**
 * @brief Register data received callback
 */
bool CommunicationController_RegisterDataCallback(CommDataCallback_t callback)
{
    if (!g_comm_controller.initialized)
    {
        return false;
    }

    g_comm_controller.callbacks.data_callback = callback;
    return true;
}

/**
 * @brief Register error callback
 */
bool CommunicationController_RegisterErrorCallback(CommErrorCallback_t callback)
{
    if (!g_comm_controller.initialized)
    {
        return false;
    }

    g_comm_controller.callbacks.error_callback = callback;
    return true;
}

/**
 * @brief Register status change callback
 */
bool CommunicationController_RegisterStatusCallback(CommStatusCallback_t callback)
{
    if (!g_comm_controller.initialized)
    {
        return false;
    }

    g_comm_controller.callbacks.status_callback = callback;
    return true;
}

/* =============================================================================
 * INTERRUPT SERVICE ROUTINES (Called by existing ISRs)
 * ============================================================================= */

/**
 * @brief UART receive interrupt handler
 * @details Called from existing UART ISR
 */
void CommunicationController_UartRxIsr(uint8_t received_byte)
{
    if (!g_comm_controller.initialized)
    {
        return;
    }

    g_comm_controller.statistics.uart_rx_count++;

    // Notify callback if registered
    if (g_comm_controller.callbacks.data_callback)
    {
        g_comm_controller.callbacks.data_callback(COMM_INTERFACE_UART,
                                                  &received_byte, 1);
    }
}

/* =============================================================================
 * PRIVATE FUNCTION IMPLEMENTATIONS
 * ============================================================================= */

/**
 * @brief Initialize UART interface
 */
static bool CommController_InitializeUart(void)
{
    // UART initialization is handled by existing Uart_Init()
    g_comm_controller.uart_enabled = true;
    return true;
}

/**
 * @brief Initialize SPI interface
 */
static bool CommController_InitializeSpi(void)
{
    // SPI initialization is handled by existing SPI_Init()
    g_comm_controller.spi_enabled = true;
    return true;
}

/**
 * @brief Initialize I2C interface
 */
static bool CommController_InitializeI2c(void)
{
    // I2C initialization is handled by existing IIC_Init()
    g_comm_controller.i2c_enabled = true;
    return true;
}

/**
 * @brief Initialize display interface
 */
static bool CommController_InitializeDisplay(void)
{
    // OLED initialization is handled by existing OLED_Init()
    g_comm_controller.display_enabled = true;

    // Display startup message
    CommunicationController_ClearDisplay();
    CommunicationController_DisplayMessage(0, "System Starting");
    CommunicationController_DisplayMessage(1, "Please wait...");

    return true;
}