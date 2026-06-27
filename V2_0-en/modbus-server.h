/**
 * @file    modbus-server.h
 * @brief   Modbus RTU server protocol header
 * @version 2.1.0
 * @date    2026
 *
 * @details Object-oriented design in C provides:
 *          - Instantiation of independent Modbus RTU servers.
 *          - Per-server data mapping.
 *          - Per-server callbacks for extending the Modbus protocol.
 *          - Circular receive buffer
 *          - Dynamic timeout calculation
 *          - Hardware CRC calculation
 *          - Choice of DMA or blocking send
 * @note    Serial ports are assumed to share baud rate and parity settings.
 *
 */

#ifndef __MODBUS_SERVER_H
#define __MODBUS_SERVER_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =============================================================
 *                      Data type definitions
 * ============================================================= */

/* =============================================================
 *                              Macros
 * ============================================================= */

#define MB_RTU_ADU_MAX 256 /* Modbus RTU ADU buffer size */
#define MB_MAX_INSTANCES 4 /* Maximum number of Modbus instances */

#if HAVE_REV
# define HTONL(_val) __REV(_val)    /* __REV(0x12345678) => 0x78563412 */
# define NTOHL(_val) __REV(_val)    /* __REV(0x12345678) => 0x78563412 */
# define HTONS(_val) __REVSH(_val)  /* __REVSH(0x1234) => 0x3412 */
# define NTOHS(_val) __REVSH(_val)  /* __REVSH(0x1234) => 0x3412 */
#else
# define HTONL(_val) \
  (uint32_t)(((_val) & 0xFF) << 24 | ((_val) & 0xFF00) << 8 | ((_val) & 0xFF0000) >> 8 | ((_val) >> 24 & 0xFF))
# define NTOHL(_val) \
  (uint32_t)(((_val) & 0xFF) << 24 | ((_val) & 0xFF00) << 8 | ((_val) & 0xFF0000) >> 8 | ((_val) >> 24 & 0xFF))
# define HTONS(_val) \
  (uint16_t)(((_val) & 0xFF) << 8 | (_val) >> 8 & 0xFF)
# define NTOHS(_val) \
  (uint16_t)(((_val) & 0xFF) << 8 | (_val) >> 8 & 0xFF)
#endif  /* !HAVE_REV */

#define IS_BIT_SET(_ba, _bit)                                                  \
  ((((uint8_t *)(_ba))[(_bit) / 8] >> ((_bit) % 8)) & 0x01)
#define CLEAR_BIT(_ba, _bit)                                                   \
  (((uint8_t *)(_ba))[(_bit) / 8] &= ~(uint8_t)(1 << ((_bit) % 8)))
#define SET_BIT(_ba, _bit)                                                     \
  (((uint8_t *)(_ba))[(_bit) / 8] |= (1 << ((_bit) % 8)))

/* =============================================================
 *               Modbus function code type definition
 * ============================================================= */

typedef enum {
  MB_IDLE = 0x00,                           /**< Reserved */
  MB_READ_COILS = 0x01,                     /**< Read coil status */
  MB_READ_DISCRETE_INPUTS = 0x02,           /**< Read discrete input */
  MB_READ_HOLDING_REGISTERS = 0x03,         /**< Read holding register */
  MB_READ_INPUT_REGISTERS = 0x04,           /**< Read input register */
  MB_WRITE_SINGLE_COIL = 0x05,              /**< Write a single coil */
  MB_WRITE_SINGLE_REGISTER = 0x06,          /**< Write a single register */
  MB_READ_EXCEPTION_STATUS = 0x07,          /**< Read exception status */
  MB_DIAGNOSTICS = 0x08,                    /**< Diagnostics */
  MB_GET_COMM_EVENT_COUNTER = 0x0B,         /**< Get communication event counter */
  MB_GET_COMM_EVENT_LOG = 0x0C,             /**< Get communication event log */
  MB_WRITE_MULTIPLE_COILS = 0x0F,           /**< Write multiple coils */
  MB_WRITE_MULTIPLE_REGISTERS = 0x10,       /**< Write multiple registers */
  MB_REPORT_SERVER_ID = 0x11,               /**< Report server ID */
  MB_READ_FILE_RECORD = 0x14,               /**< Read file record */
  MB_WRITE_FILE_RECORD = 0x15,              /**< Write file record */
  MB_MASK_WRITE_REGISTER = 0x16,            /**< Mask write register */
  MB_READ_WRITE_MULTIPLE_REGISTERS = 0x17,  /**< Read/Write multiple registers */
  MB_READ_FIFO_QUEUE = 0x18,                /**< Read FIFO queue */
  MB_CONFIGURE_UART = 0x65                       /**< User-defined device configuration */
} mb_function_code_t;

/* =============================================================
 *              Modbus exception code type definition
 * ============================================================= */

typedef enum {
  MB_ILLEGAL_FUNCTION = 0x01,       /**< Illegal function code */
  MB_ILLEGAL_DATA_ADDRESS = 0x02,   /**< Illegal data address */
  MB_ILLEGAL_DATA_VALUE = 0x03,     /**< Illegal data value */
  MB_SERVER_DEVICE_FAILURE = 0x04,  /**< Server device failure */
  MB_ACKNOWLEDGE = 0x05,            /**< Acknowledge */
  MB_SERVER_DEVICE_BUSY = 0x06,     /**< Server busy */
  MB_MEMORY_PARITY_ERROR = 0x08     /**< Memory parity error */
} mb_exception_code_t;

typedef enum {
  MB_SERVER_ADDRESS = 0x01,         /**< Parameter value is server address */
  MB_RS485_BAUDRATE = 0x02,         /**< Parameter value is RS485 baud rate */
  MB_RS485_PARITY = 0x03            /**< Parameter value is RS485 parity */
} mb_param_t;


/* =============================================================
 *            Type definitions and forward declarations
 * ============================================================= */

typedef struct MB_ServerHandle MB_ServerHandle_t;
typedef uint16_t (*mb_func_t) (const uint8_t *pdu, uint16_t pdu_len, MB_ServerHandle_t *hmb);

/**
 * @brief   Modbus server configuration structure
 * @details Used to initialize Modbus server instance. Saved to flash memory.
 */
typedef struct MB_RTUConfig {
  uint8_t  address;                 /**< server address (1 - 247) */
  uint16_t baud_rate;               /**< baud rate (used for dynamic timeout calculation) */
  uint8_t  parity;                  /**< Current parity setting (0: none, 1: odd, 2: even) */
  bool     use_dma_tx;              /**< Enable UART DMA */
} MB_RTUConfig_t;

/**
 * @brief   Data mapping configuration structure
 * @details Used to bind external data sources to a Modbus instance.
 *          Supports multiple Modbus instances sharing the same data source
 *
 * @warning Data shared across Modbus instances should be protected with a mutex.
 *
 */
typedef struct MB_DataMap {
  /* Coils - Modbus address 0xxxx */
  uint8_t *coils_p;                 /**< Pointer to coil data (bit-wise storage). */
  uint16_t coil_count;              /**< Number of coils (0 - 65535) */

  /* Discrete Inputs - Modbus address 1xxxx */
  uint8_t *discrete_inputs_p;       /**< Pointer to discrete input data (bit-wise storage). */
  uint16_t discrete_count;          /**< Number of discrete inputs (0 - 65535). */

  /* Holding Registers - Modbus address 4xxxx */
  uint16_t *holding_registers_p;    /**< Pointer to holding registers (16-bit storage). */
  uint16_t holding_register_count;  /**< Number of holding registers (0 - 125). */

  /* Input Registers - Modbus address 3xxxx */
  uint16_t *input_registers_p;      /**< Pointer to input registers (16-bit storage). */
  uint16_t input_register_count;    /**< Number of input registers (0 - 125). */
} MB_DataMap_t;

/**
 * @brief RS485 hardware configuration structure
 */
typedef struct MB_RS485Config {
  UART_HandleTypeDef *huart_p;      /**< Pointer to HAL UART handle */
  GPIO_TypeDef *de_port_p;          /**< DE control pin GPIO port (e.g., GPIOA) */
  uint16_t      de_pin;             /**< DE control pin number (e.g., GPIO_PIN_8) */
} MB_RS485Config_t;

/**
 * @brief   Modbus RTU server initialization structure
 * @details Used to initialize a Modbus server instance.
 */
typedef struct MB_ServerConfig {
  MB_RTUConfig_t rtu;               /**< RTU server address and serial configuration */
  MB_DataMap_t data_map;            /**< Coil and register maps */
  MB_RS485Config_t rs485;           /**< RS485 interface */
} MB_ServerConfig_t;

/* =============================================================
 *                Callback function type definitions
 * ============================================================= */

/**
 * @brief   Callback configuration function type definition
 * @param   hmb          Modbus server handle
 * @param   param_type   Parameter type (see definition of mb_param_t).
 * @param   param_value  Parameter value
 * @retval  true if sucessful, otherwise false.
 * @retval  false fails, the protocol stack will send an exception response.
 *
 * @note    This callback is used to handle device configuration commands (such
 *          as modifying server address, baud rate, etc.).  Operations such as Flash
 *          saving and system reset should be completed within the callback.
 *
 * @warning The callback execution will block Modbus processing. Please minimize
 *          the execution time. For operations requiring a delay (such as resetting after
 *          writing to Flash), it is recommended to set a flag and handle it in the main
 *          loop.
 */
typedef bool (*MB_ConfigCallback_t)(MB_ServerHandle_t *hmb, mb_param_t param_type,
                                    uint16_t param_value);

/**
 * @brief  Callback write-validation function type definition (optional))
 * @param  hmb        Modbus server handle
 * @param  func_code  Function code
 * @param  start_addr Starting address
 * @param  quantity   Quantity
 * @retval true       Allow writes
 * @retval false      Reject write operation, and return an exception response
 *
 * @note   Is used to implement write access control or data validation.
 */
typedef bool (*MB_WriteCallback_t)(MB_ServerHandle_t *hmb, uint8_t func_code,
                                   uint16_t start_addr, uint16_t quantity);

/**
 * @brief   Modbus server handle type definition
 * @details The attribute contains independent runtime states and supports
 *          parallel execution of multiple instances. Similar to class member variables
 *          in C++
 *
 * @note    Users should initialize this structure using MB_Init()
 *          Direct access to internal members is not recommended; please use the
 *          provided API.
 */
typedef struct MB_ServerHandle {
  uint32_t *device_uid_p;           /**< Pointer to 96-bit STM32H7 device unique ID */
  MB_RTUConfig_t rtu;               /**< RTU server address and serial configuration */
  MB_DataMap_t data_map;            /**< Coil and register maps */
  MB_RS485Config_t rs485;           /**< RS485 interface */

  /* =========  Modbus RTU request and response buffers ======== */
  uint8_t  req[2][MB_RTU_ADU_MAX];  /**< Modbus ping-pong request buffers */
  uint8_t  req_idx;                 /**< Modbus request buffer index (0 or 1) */
  uint16_t req_len;                 /**< Modbus request buffer data length */
  uint8_t  req_ready;               /**< Modbus request state (1 = data pending) */

  uint8_t  res[MB_RTU_ADU_MAX];     /**< Modbus response buffer */
  uint16_t res_len;                 /**< Modbus response buffer length/index */

  /* ==================== Callback Functions =================== */
  MB_ConfigCallback_t config_cb;    /**< User-defined function code 0x65 callback */
  MB_WriteCallback_t  write_cb;     /**< User-defined write validation callback */

  /* ================= Diagnostics and Counters ================ */
  uint8_t  processing[MB_RTU_ADU_MAX];  /**< Previous Modbus request (0 = idle) */
  bool     listen_only_mode;        /**< Listen-only mode flag. If true, inhibit server responses */
  uint32_t bus_message_count;       /**< Count of Modbus messages received since power-up/reset */
  uint32_t bus_event_count;         /**< Count of valid Modbus messages processed since power-up/reset */
  uint32_t crc_error_count;         /**< Count of CRC errors since power-up/reset */
  uint32_t bus_exception_count;     /**< Count of PDU process exceptions since power-up/reset */
  uint8_t  exception_status;        /**< Bit-wise exception register */
} MB_ServerHandle_t;

/* =============================================================
 *                    API function declarations
 * ============================================================= */

/**
 * @brief  Initializes a Modbus server instance
 * @param  hmb     Modbus server handle
 * @param  config  Initialization structure pointer
 * @retval Initialization status: 0: success,  >0: failure
 *
 * @note   After calling this function, the Modbus server station begins
 * listening for UART data. Ensure that the UART has been correctly configured
 * and initialized in CubeMX.
 *
 * @code
 *         // Usage example
 *         MB_ServerHandle_t hmb1;
 *         MB_ServerConfig_t config = { ... };
 *         if (!MB_Init(&hmb1, &config)) {
 *             Error_Handler();
 *         }
 * @endcode
 */
int MB_Init(MB_ServerHandle_t *hmb, const MB_ServerConfig_t *config);


/**
 * @brief   Modbus server ADU processing function
 * @param   hmb  Modbus server handle
 *
 * @note    This should be called periodically in the main loop.
 *          This function parses the received Modbus frame and generates a
 *          response. A ping-pong buffering mechanism is used to prevent the reception of
 *          new data during processing.
 *
 * @warning This function is not thread-safe. In an RTOS environment, ensure
 * that it is called by a single-task application. Or use a mutex lock for
 * protection
 */
uint16_t MB_ServerProcessADU(MB_ServerHandle_t *hmb);


/**
 * @brief UART receive complete/idle interrupt callback
 * @param hmb   Modbus server handle
 * @param size  Length of received data
 *
 * @note  This function needs to be called within HAL_UARTEx_RxEventCallback().
 *
 * @code
 *        void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
 *          if (huart->Instance == hmb1.huart->Instance) {
 *            MB_RxCallback(&hmb1, Size);
 *          } else if (huart->Instance == hmb2.huart->Instance) {
 *            MB_RxCallback(&hmb2, Size);
 *          }
 *        }
 * @endcode
 */
void MB_RxCallback(MB_ServerHandle_t *hmb, uint16_t size);


/**
 * @brief  UART send complete callback (for DMA mode only)
 * @param  hmb  Modbus server handle
 *
 * @note   This only needs to be called when use_dma_tx = true.
 *         This function needs to be called within HAL_UART_TxCpltCallback().
 *         Used to switch RS485 direction after DMA send is complete.
 */
void MB_TxCallback(MB_ServerHandle_t *hmb);


/**
 * @brief Start UART reception
 * @param hmb  Modbus server handle
 *
 * @note  This is usually called automatically by MB_Init()
 *        In certain error recovery scenarios, it can be manually invoked to
 *        restart the receiver.
 */
void MB_StartReceive(MB_ServerHandle_t *hmb);

/* =============================================================
 *                       Extern declarations
 * ============================================================= */

extern CRC_HandleTypeDef hcrc;
extern uint16_t mb_crc_table[256];
extern MB_ServerHandle_t *hmb_instances[MB_MAX_INSTANCES];  /**< Array of Modbus server instances */

#ifdef __cplusplus
}
#endif

#endif /* __MODBUS_SERVER_H */
