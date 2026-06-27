/**
 * @file    modbus-server.c
 * @brief   Modbus RTU server implementation
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
 */

#include "modbus-server.h"
#include <cstdint>
#include <string.h>


/* ============================================================================
 *                   Static function declarations
 * ============================================================================ */

static uint16_t MB_CRC16(const MB_ServerHandle_t *hmb, const uint8_t *buffer, uint16_t length);
static uint16_t MB_Exception(mb_exception_code_t exception, MB_ServerHandle_t *hmb);
static void MB_SendResponse(const MB_ServerHandle_t *hmb, uint16_t len);
static void MB_RS485_SetTxMode(const MB_ServerHandle_t *hmb);
static void MB_RS485_SetRxMode(const MB_ServerHandle_t *hmb);
static uint16_t MB_ReadCoils(const uint8_t *pdu, uint16_t pdu_len, MB_ServerHandle_t *hmb);
static uint16_t MB_ReadDiscreteInputs(const uint8_t *pdu, uint16_t pdu_len, MB_ServerHandle_t *hmb);
static uint16_t MB_ReadHoldingRegisters(const uint8_t *pdu, uint16_t pdu_len, MB_ServerHandle_t *hmb);
static uint16_t MB_ReadInputRegisters(const uint8_t *pdu, uint16_t pdu_len, MB_ServerHandle_t *hmb);
static uint16_t MB_WriteSingleCoil(const uint8_t *pdu, uint16_t pdu_len, MB_ServerHandle_t *hmb);
static uint16_t MB_WriteSingleRegister(const uint8_t *pdu, uint16_t pdu_len, MB_ServerHandle_t *hmb);
static uint16_t MB_ReadExceptionStatus(const uint8_t *pdu, uint16_t pdu_len, MB_ServerHandle_t *hmb);
static uint16_t MB_Diagnostics(const uint8_t *pdu, uint16_t pdu_len, MB_ServerHandle_t *hmb);
static uint16_t MB_GetCommEventCounter(const uint8_t *pdu, uint16_t pdu_len, MB_ServerHandle_t *hmb);
static uint16_t MB_GetCommEventLog(const uint8_t *pdu, uint16_t pdu_len, MB_ServerHandle_t *hmb);
static uint16_t MB_WriteMultipleCoils(const uint8_t *pdu, uint16_t pdu_len, MB_ServerHandle_t *hmb);
static uint16_t MB_WriteMultipleRegisters(const uint8_t *pdu, uint16_t pdu_len, MB_ServerHandle_t *hmb);
static uint16_t MB_ReportServerId(const uint8_t *pdu, uint16_t pdu_len, MB_ServerHandle_t *hmb);
static uint16_t MB_ReadFileRecord(const uint8_t *pdu, uint16_t pdu_len, MB_ServerHandle_t *hmb);
static uint16_t MB_WriteFileRecord(const uint8_t *pdu, uint16_t pdu_len, MB_ServerHandle_t *hmb);
static uint16_t MB_MaskWriteRegister(const uint8_t *pdu, uint16_t pdu_len, MB_ServerHandle_t *hmb);
static uint16_t MB_ReadWriteMultipleRegisters(const uint8_t *pdu, uint16_t pdu_len, MB_ServerHandle_t *hmb);
static uint16_t MB_ReadFifoQueue(const uint8_t *pdu, uint16_t pdu_len, MB_ServerHandle_t *hmb);
static uint16_t MB_ConfigurationUpdate(const uint8_t *pdu, uint16_t pdu_len, MB_ServerHandle_t *hmb);
static uint16_t MB_IllegalFunction(const uint8_t *pdu, uint16_t pdu_len, MB_ServerHandle_t *hmb);

static const mb_func_t MB_Func[] = {
    MB_IllegalFunction,            /* Function code 0x00 */
    MB_ReadCoils,                  /* Function code 0x01 */
    MB_ReadDiscreteInputs,         /* Function code 0x02 */
    MB_ReadHoldingRegisters,       /* Function code 0x03 */
    MB_ReadInputRegisters,         /* Function code 0x04 */
    MB_WriteSingleCoil,            /* Function code 0x05 */
    MB_WriteSingleRegister,        /* Function code 0x06 */
    MB_ReadExceptionStatus,        /* Function code 0x07 */
    MB_Diagnostics,                /* Function code 0x08 */
    MB_IllegalFunction,            /* Function code 0x09 */
    MB_IllegalFunction,            /* Function code 0x0A */
    MB_GetCommEventCounter,        /* Function code 0x0B */
    MB_GetCommEventLog,            /* Function code 0x0C */
    MB_IllegalFunction,            /* Function code 0x0D */
    MB_IllegalFunction,            /* Function code 0x0E */
    MB_WriteMultipleCoils,         /* Function code 0x0F */
    MB_WriteMultipleRegisters,     /* Function code 0x10 */
    MB_ReportServerId,             /* Function code 0x11 */
    MB_IllegalFunction,            /* Function code 0x12 */
    MB_IllegalFunction,            /* Function code 0x13 */
    MB_ReadFileRecord,             /* Function code 0x14 */
    MB_WriteFileRecord,            /* Function code 0x15 */
    MB_MaskWriteRegister,          /* Function code 0x16 */
    MB_ReadWriteMultipleRegisters, /* Function code 0x17 */
    MB_ReadFifoQueue,              /* Function code 0x18 */
};

/* ============================================================================
 *               Modbus port initialization function
 * ============================================================================ */

/**
 * @brief   Initialize a Modbus server instance
 */
int MB_Init(MB_ServerHandle_t *hmb, const MB_Config_t *config) {
  /* Parameter validity check */
  if (!hmb || !config || !config->huart) {
    return 1;
  }

  /*
   * Check the validity of the server address
   * Per "Modbus over serial line V1.02", Section 2.1.
   */
  if (!(1 <= config->server_addr && config->server_addr <= 247)) {
    return 2;
  }

  /* Copy UART configuration */
  hmb->huart = config->huart;
  hmb->server_addr = config->server_addr;
  hmb->baud_rate = config->baud_rate;

  /* Initialize the ping-pong buffer state */
  hmb->req_idx = 0;
  hmb->req_len = 0;
  hmb->req_ready = 0;

  /* Copy data mapping */
  if (config->data_map.holding_registers
      && config->data_map.holding_register_count > 125) {
    return 3; /* Holding registers exceed maximum allowed */
  } else if (config->data_map.input_registers
             && config->data_map.input_register_count > 125) {
    return 4; /* Input registers exceed maximum allowed */
  }

  hmb->data_map = config->data_map;

  /* Copy RS485 configuration */
  hmb->rs485 = config->rs485;

  /* Copy runtime options */
  hmb->use_dma_tx = config->use_dma_tx;
  hmb->use_hw_crc = config->use_hw_crc;
  hmb->use_crc_table = config->use_crc_table;

  if (hmb->use_hw_crc && hcrc.Instance) {
    hmb->hcrc = &hcrc;
  }

  // if (hmb->use_crc_table) {
  //   /* If using CRC table, initialize it. */
  //   MB_CRC_Init();
  // }
  //
  // /* RS485 initialization: Defaults to receive mode */
  // if (hmb->rs485.enabled) {
  //   MB_RS485_SetRxMode(hmb);
  // }
  //
  // /* Enable UART idle interrupt reception */
  // MB_StartReceive(hmb);

  return 0;
}

/* ============================================================================
 *                 Interrupt callback interface
 * ============================================================================ */

/**
 * @brief   UART receive complete/idle interrupt callback
 */
void MB_RxCallback(MB_ServerHandle_t *hmb, uint16_t size) {
  static uint8_t buf_idx = 0;

  if (!hmb) {
    return;
  }

  /*
   * Ping-pong buffer switching:
   * 1. Set the current receive buffer as the processing buffer.
   * 2. Record the length of received data
   * 3. Switch the interrupt receive target to another buffer.
   * 4. Immediately restart reception to reduce the data loss window.
   */

  /* Swap active and process buffers */
  hmb->req_idx ^= 1;

  /* Set the processing buffer data length */
  hmb->req_len = size;
  hmb->req_ready = 1;

  /* Restart receive immediately, pointing to the new buffer */
  HAL_UARTEx_ReceiveToIdle_IT(hmb->huart, hmb->req[hmb->req_idx], MB_RTU_ADU_MAX);
}

/**
 * @brief   UART send complete callback (DMA mode)
 */
void MB_TxCallback(MB_ServerHandle_t *hmb) {
  if (!hmb) {
    return;
  }

  if (hmb->use_dma_tx && hmb->rs485.enabled) {
    /*
     * DMA transmission complete, wait for TC flag, then switch RS485 to receive
     * mode. Note: When the DMA completes the interrupt, all data has already
     * been moved into transmit buffer. But we need to wait for the TC flag to
     * ensure the last byte is completely sent.
     */
    while (__HAL_UART_GET_FLAG(hmb->huart, UART_FLAG_TC) == RESET) {
      HAL_Delay(1); /* Wait for transmission complete */
    }
    MB_RS485_SetRxMode(hmb);
  }
}

/* ============================================================================
 *           Modbus RTU ADU parser and response generator
 * ============================================================================ */

/**
 * @brief   Modbus frame parsing and response processing
 * @return  ADU response length or 0 if no response (e.g., error or broadcast)
 */
uint16_t MB_ServerProcessADU(MB_ServerHandle_t *hmb) {
  static uint32_t errs = 0;

  if (!hmb || !hmb->req_ready || hmb->req_len < 4) {
    return 0;
  }

  /* Toggle request index for ADU process buffer */
  uint8_t *req = hmb->req[hmb->req_idx ^ 1];
  uint16_t req_len = hmb->req_len;

  /* Clear request ready flag */
  hmb->req_ready = 0;
  hmb->req_len = 0;
  hmb->bus_message_count++;

  uint8_t req_addr = req[0];

  /* Verify address or broadcast */
  if (req_addr != hmb->server_addr && req_addr != 0) {
    return 0;
  }

  /* Verify request CRC */
  uint16_t received_crc = NTOHS(*(uint16_t *)(req + req_len - 2));
  uint16_t calculated_crc = MB_CRC16(hmb, req, req_len - 2);

  /* If CRC mismatch, increment CRC error counter, but don't respond */
  if (received_crc != calculated_crc) {
    hmb->crc_error_count++;
    return 0;
  }

  /*
   * Initialize request PDU pointer and response frame.
   * The response address is set to the current server address.
   */
  mb_function_code_t func_code = req[1];
  const uint8_t *pdu = &req[2];
  uint16_t pdu_len = req_len - 4;  /* PDU: Remove server address, function code and CRC */

  /* Check for "Poll Program Complete" */
  if (*hmb->processing) {
    if (memcmp(hmb->processing, pdu, pdu_len) == 0) {
      /* Still working on previous request. */
      MB_Exception(MB_ACKNOWLEDGE, hmb);
      goto post_process;
    } else if (!(func_code == 0x08 || func_code == 0x0B || func_code == 0x0C)) {
        MB_Exception(MB_SERVER_DEVICE_BUSY, hmb);
        goto post_process;
    }
  } else if (!(func_code == 0x08 || func_code == 0x0B || func_code == 0x0C)) {
    /* Mark the current request as being processed. */
    memcpy(hmb->processing, pdu, pdu_len);
  }

  hmb->res[0] = hmb->server_addr;
  hmb->res[1] = func_code;
  hmb->res_len = 2;

  if (0x00 <= func_code && func_code <= 0x18) {
    MB_Func[func_code](pdu, pdu_len, hmb);
  } else if (func_code == 0x65) {
    MB_ConfigurationUpdate(pdu, pdu_len, hmb);
  } else {
    MB_Exception(MB_ILLEGAL_FUNCTION, hmb);
  }

  if (!(func_code == 0x08 || func_code == 0x0B || func_code == 0x0C))
    *hmb->processing = 0;

 post_process:
  /* Don't respond to broadcast or with empty message */
  if (req_addr == 0 || hmb->res_len == 0)
    return 0;

  /* Send CRC in network byte order. */
  uint16_t crc = MB_CRC16(hmb, hmb->res, hmb->res_len);
  *(uint16_t *)(hmb->res + hmb->res_len) = HTONS(crc);
  return (hmb->res_len += 2);
}

static uint16_t MB_ReadCoils(const uint8_t *pdu, uint16_t pdu_len, MB_ServerHandle_t *hmb) {
  if (pdu_len != 4) {
    return MB_Exception(MB_ILLEGAL_DATA_VALUE, hmb);
  } else if (!hmb->data_map.coils || hmb->data_map.coil_count == 0) {
    return MB_Exception(MB_ILLEGAL_FUNCTION, hmb);
  }

  uint16_t start_addr = NTOHS(*(uint16_t *)pdu);
  uint16_t quantity = NTOHS(*(uint16_t *)(pdu + 2));

  /*
   * 65535 coils can be addressed. Modbus RTU messages can hold 252
   * bytes of data, so there space to read 2008 coils (including
   * 1-byte byte_count), but the Modbus Application Protocol 1.1b,
   * Section 6.1 limits coil-read maximum to 2000.
   */
  if (!(1 <= quantity && quantity <= 2000)) {
    return MB_Exception(MB_ILLEGAL_DATA_VALUE, hmb);
  } else if (start_addr + quantity > hmb->data_map.coil_count) {
    return MB_Exception(MB_ILLEGAL_DATA_ADDRESS, hmb);
  }

  uint8_t byte_count = (quantity + 7) / 8;

  hmb->res[hmb->res_len++] = byte_count;
  memset(hmb->res + hmb->res_len, 0, byte_count);

  /*
   * Read coil status and pack response
   * Note: Shared data access should be protected with critical sections.
   */
  for (uint16_t i = 0; i < quantity; ++i) {
    uint16_t offset = start_addr + i;

    if (IS_BIT_SET(hmb->data_map.coils, offset)) {
      SET_BIT(hmb->res + hmb->res_len, i);
    }
  }

  hmb->res_len += byte_count;
  hmb->bus_event_count++;
  return 0;
}

static uint16_t MB_ReadDiscreteInputs(const uint8_t *pdu, uint16_t pdu_len, MB_ServerHandle_t *hmb) {
  if (pdu_len != 4) {
    return MB_Exception(MB_ILLEGAL_DATA_VALUE, hmb);
  } else if (!hmb->data_map.discrete_inputs || hmb->data_map.discrete_count == 0) {
    return MB_Exception(MB_ILLEGAL_FUNCTION, hmb);
  }

  uint16_t start_addr = NTOHS(*(uint16_t *)pdu);
  uint16_t quantity = NTOHS(*(uint16_t *)(pdu + 2));

  /*
   * 65535 bits can be addressed. Modbus RTU messages can hold 252
   * bytes of data, so there space to read 2008 coils (including
   * 1-byte byte_count), but the Modbus Application Protocol 1.1b,
   * Section 6.1 limits coil-read maximum to 2000.
   */
  if (!(1 <= quantity && quantity <= 2000)) {
    return MB_Exception(MB_ILLEGAL_DATA_VALUE, hmb);
  } else if (start_addr + quantity > hmb->data_map.discrete_count) {
    return MB_Exception(MB_ILLEGAL_DATA_ADDRESS, hmb);
  }

  uint8_t byte_count = (quantity + 7) / 8;

  hmb->res[hmb->res_len++] = byte_count;
  memset(hmb->res + hmb->res_len, 0, byte_count);

  /*
   * Read discrete input status and pack response
   * Note: Shared data access should be protected with critical sections.
   */
  for (uint16_t i = 0; i < quantity; ++i) {
    uint16_t offset = start_addr + i;

    if (IS_BIT_SET(hmb->data_map.discrete_inputs, offset)) {
      SET_BIT(hmb->res + hmb->res_len, i);
    }
  }

  hmb->res_len += byte_count;
  hmb->bus_event_count++;
  return 0;
}

static int16_t MB_ReadHoldingRegisters(const uint8_t *pdu, uint16_t pdu_len, MB_ServerHandle_t *hmb) {
  if (pdu_len != 4) {
    return MB_Exception(MB_ILLEGAL_DATA_VALUE, hmb);
  } else if (!hmb->data_map.holding_registers || hmb->data_map.holding_register_count == 0) {
    return MB_Exception(MB_ILLEGAL_FUNCTION, hmb);
  }

  uint16_t start_addr = NTOHS(*(uint16_t *)pdu);
  uint16_t quantity = NTOHS(*(uint16_t *)(pdu + 2));

  /* Parameter validation */
  if (!(1 <= quantity && quantity <= 125)) {
    return MB_Exception(MB_ILLEGAL_DATA_VALUE, hmb);
  } else if (start_addr + quantity > hmb->data_map.holding_register_count) {
    return MB_Exception(MB_ILLEGAL_DATA_ADDRESS, hmb);
  }

  uint8_t byte_count = 2 * quantity;

  hmb->res[hmb->res_len++] = byte_count;
  memset(hmb->res + hmb->res_len, 0, byte_count);

    /*
     * Read data from holding registers
     * Note: For 16-bit registers, the Cortex-M core guarantees the
     *       atomicity of a single read.
     */
  for (uint16_t i = 0; i < quantity; ++i) {
    *(uint16_t *)(hmb->res + hmb->res_len + 2 * i) =
        HTONS(hmb->data_map.holding_registers[start_addr + i]);
  }

  hmb->res_len += byte_count;
  hmb->bus_event_count++;
  return 0;
}

static uint16_t MB_ReadInputRegisters(const uint8_t *pdu, uint16_t pdu_len, MB_ServerHandle_t *hmb) {
  if (pdu_len != 4) {
    return MB_Exception(MB_ILLEGAL_DATA_VALUE, hmb);
  } else if (!hmb->data_map.input_registers || hmb->data_map.input_register_count == 0) {
    return MB_Exception(MB_ILLEGAL_FUNCTION, hmb);
  }

  uint16_t start_addr = NTOHS(*(uint16_t *)pdu);
  uint16_t quantity = NTOHS(*(uint16_t *)(pdu + 2));

  /* Parameter validation */
  if (!(1 <= quantity && quantity <= 125)) {
    return MB_Exception(MB_ILLEGAL_DATA_VALUE, hmb);
  } else if (start_addr + quantity > hmb->data_map.input_register_count) {
    return MB_Exception(MB_ILLEGAL_DATA_ADDRESS, hmb);
  }

  uint8_t byte_count = 2 * quantity;

  hmb->res[hmb->res_len++] = byte_count;
  memset(hmb->res + hmb->res_len, 0, byte_count);

  /* Read data from the input register */
  for (uint16_t i = 0; i < quantity; ++i) {
    *(uint16_t *)(hmb->res + hmb->res_len + 2 * i) =
      HTONS(hmb->data_map.input_registers[start_addr + i]);
  }

  hmb->res_len += byte_count;
  hmb->bus_event_count++;
  return 0;
}

static uint16_t MB_WriteSingleCoil(const uint8_t *pdu, uint16_t pdu_len, MB_ServerHandle_t *hmb) {
  if (pdu_len != 4) {
    return MB_Exception(MB_ILLEGAL_DATA_VALUE, hmb);
  } else if (!hmb->data_map.coils || hmb->data_map.coil_count == 0) {
    return MB_Exception(MB_ILLEGAL_FUNCTION, hmb);
  }

  uint16_t start_addr = NTOHS(*(uint16_t *)pdu);
  uint16_t val = NTOHS(*(uint16_t *)(pdu + 2));

  if (start_addr >= hmb->data_map.coil_count) {
    return MB_Exception(MB_ILLEGAL_DATA_ADDRESS, hmb);
  }

  /* Callback check before writing */
  if (hmb->write_cb) {
    if (!hmb->write_cb(hmb, MB_WRITE_SINGLE_COIL, start_addr, 1)) {
      return MB_Exception(MB_SERVER_DEVICE_FAILURE, hmb);
    }
  }

  /* Write to coil */
  if (val == 0xFF00) {
    SET_BIT(hmb->data_map.coils, start_addr);
  } else if (val == 0x0000) {
    CLEAR_BIT(hmb->data_map.coils, start_addr);
  } else {
    return MB_Exception(MB_ILLEGAL_DATA_VALUE, hmb);
  }

  /* Echo request */
  memcpy(hmb->res, pdu, pdu_len);
  hmb->res_len += pdu_len;
  hmb->bus_event_count++;
 return 0;
}

static uint16_t MB_WriteSingleRegister(const uint8_t *pdu, uint16_t pdu_len, MB_ServerHandle_t *hmb) {
 if (pdu_len != 4) {
    return MB_Exception(MB_ILLEGAL_DATA_VALUE, hmb);
  } else if (!hmb->data_map.holding_registers || hmb->data_map.holding_register_count == 0) {
    return MB_Exception(MB_ILLEGAL_FUNCTION, hmb);
  }

  uint16_t start_addr = NTOHS(*(uint16_t *)pdu);
  uint16_t value = NTOHS(*(uint16_t *)(pdu + 2));

  if (start_addr >= hmb->data_map.holding_register_count) {
    return MB_Exception(MB_ILLEGAL_DATA_ADDRESS, hmb);
  }

  /* Callback check before writing */
  if (hmb->write_cb
      && !hmb->write_cb(hmb, MB_WRITE_SINGLE_REGISTER, start_addr, 1)) {
    return MB_Exception(MB_SERVER_DEVICE_FAILURE, hmb);
  }

  /* Write to register */
  hmb->data_map.holding_registers[start_addr] = value;

  /* Echo request */
  memcpy(hmb->res, pdu, pdu_len);
  hmb->res_len += pdu_len;
  hmb->bus_event_count++;
  return 0;
}


static uint16_t MB_ReadExceptionStatus(const uint8_t *pdu, uint16_t pdu_len, MB_ServerHandle_t *hmb) {
  if (pdu_len != 0) {
    return MB_Exception(MB_ILLEGAL_DATA_VALUE, hmb);
  }

  /* Return the exception status byte */
  hmb->res[hmb->res_len++] = hmb->exception_status;
  hmb->bus_event_count++;
  return 0;
}

static uint16_t MB_Diagnostics(const uint8_t *pdu, uint16_t pdu_len, MB_ServerHandle_t *hmb) {
  if (pdu_len < 4) {
    return MB_Exception(MB_ILLEGAL_DATA_VALUE, hmb);
  }

  uint16_t sub_func = NTOHS(*(uint16_t *)pdu);
  uint16_t *data = (uint16_t *)(pdu + 2);

  switch (sub_func) {
  case 0x0000:                  /* Return query data */
    /* Echo request */
    memcpy(hmb->res + hmb->res_len, pdu, pdu_len);
    hmb->res_len += pdu_len;
    break;

  case 0x0001:                  /* Restart communications option */
    if (hmb->listen_only_mode) {
      /* Suppress response */
      hmb->res_len = 0;
    } else {
      /* Echo request */
      memcpy(hmb->res + hmb->res_len, pdu, pdu_len);
      hmb->res_len += pdu_len;
    }

    /* Reset the exception status and counters */
    hmb->exception_status = 0;
    hmb->bus_exception_count = 0;
    hmb->crc_error_count = 0;

    if (NTOHS(*data) == 0xFF00) {
      /* Clear Communications Event Log */
      hmb->bus_message_count = 0;
    }

    /* Re-initialize UART */
    if (HAL_UART_Init(hmb->huart) != HAL_OK) {
      return MB_Exception(MB_SERVER_DEVICE_FAILURE, hmb);
    }
    break;

  case 0x0004:                  /* Force listen-only mode */
    hmb->listen_only_mode = true;
    hmb->res_len = 0;           /* Don't respond */
    break;

  case 0x000A:                  /* Clear counters */
    /* Echo request */
    memcpy(hmb->res + hmb->res_len, pdu, pdu_len);
    hmb->res_len += pdu_len;

    /* Reset the exception status and counters */
    hmb->bus_message_count = 0;
    hmb->crc_error_count = 0;
    hmb->exception_status = 0;
    hmb->bus_exception_count = 0;
    break;

  case 0x000B:
    /* Echo request */
    memcpy(hmb->res + hmb->res_len, pdu, pdu_len);
    hmb->res_len += pdu_len;

    /* Reset the exception status and counters */
    hmb->bus_message_count = 0;
    hmb->crc_error_count = 0;
    hmb->exception_status = 0;
    hmb->bus_exception_count = 0;
    break;

  default:
    return MB_Exception(MB_ILLEGAL_DATA_VALUE, hmb);
  }

  hmb->bus_event_count++;
 return 0;
}

static uint16_t MB_GetCommEventCounter(const uint8_t *pdu, uint16_t pdu_len,
                                       MB_ServerHandle_t *hmb) {
  if (pdu_len != 0) {
    return MB_Exception(MB_ILLEGAL_DATA_VALUE, hmb);
  }

  /* Return status word count and bus event count */
  *(uint16_t *)(hmb->res + hmb->res_len) = *hmb->processing ? 0xFF : 0x00;
  *(uint16_t *)(hmb->res + hmb->res_len + 2) = HTONS(hmb->bus_event_count);
  hmb->res_len += 4;
  return 0;
}

static uint16_t MB_GetCommEventLog(const uint8_t *pdu, uint16_t pdu_len,
                                 MB_ServerHandle_t *hmb) {
  if (pdu_len != 4) {
    return MB_Exception(MB_ILLEGAL_DATA_VALUE, hmb);
  }

  /* Return status word count and bus event count */
  *(uint16_t *)(hmb->res + hmb->res_len) = *hmb->processing ? 0xFF : 0x00;
  *(uint16_t *)(hmb->res + hmb->res_len + 2) = HTONS(hmb->bus_event_count);
  hmb->res_len += 4;
  return 0;
}


static uint16_t MB_WriteMultipleCoils(const uint8_t *pdu, uint16_t pdu_len, MB_ServerHandle_t *hmb) {
  if (pdu_len < 5) {
    return MB_Exception(MB_ILLEGAL_DATA_VALUE, hmb);
  } else if (!hmb->data_map.coils || hmb->data_map.coil_count == 0) {
    return MB_Exception(MB_ILLEGAL_FUNCTION, hmb);
  }

  uint16_t start_addr = NTOHS(*(uint16_t *)pdu);
  uint16_t quantity = NTOHS(*(uint16_t *)(pdu + 2));
  uint8_t byte_count = pdu[4];

  if (!(1 <= quantity && quantity <= 1968 && byte_count == (quantity + 7) / 8 && pdu_len == byte_count + 5)) {
    return MB_Exception(MB_ILLEGAL_DATA_VALUE, hmb);
  } else if (start_addr + quantity > hmb->data_map.coil_count) {
    return MB_Exception(MB_ILLEGAL_DATA_ADDRESS, hmb);
  }

  /* Callback check before writing */
  if (hmb->write_cb
      && !hmb->write_cb(hmb, MB_WRITE_MULTIPLE_COILS, start_addr, quantity)) {
    return MB_Exception(MB_SERVER_DEVICE_FAILURE, hmb);
  }

  /* Write to coils */
  for (uint16_t i = 0; i < quantity; ++i) {
    uint16_t offset = start_addr + i;

    if (IS_BIT_SET(pdu + 5, i)) {
      SET_BIT(hmb->data_map.coils, offset);
    } else {
      CLEAR_BIT(hmb->data_map.coils, offset);
    }
  }

  /* Echo starting address and quantity */
  memcpy(hmb->res + hmb->res_len, pdu, 4);
  hmb->res_len += 4;
  hmb->bus_event_count++;
  return 0;
}


static uint16_t MB_WriteMultipleRegisters(const uint8_t *pdu, uint16_t pdu_len, MB_ServerHandle_t *hmb) {
  if (pdu_len < 5) {
    return MB_Exception(MB_ILLEGAL_DATA_VALUE, hmb);
  } else if (!hmb->data_map.holding_registers || hmb->data_map.holding_register_count == 0) {
    return MB_Exception(MB_ILLEGAL_FUNCTION, hmb);
  }

  uint16_t start_addr = NTOHS(*(uint16_t *)pdu);
  uint16_t quantity = NTOHS(*(uint16_t *)(pdu + 2));
  uint8_t byte_count = pdu[4];

  if (!(1 <= quantity && quantity <= 123 && byte_count == 2 * quantity && pdu_len == byte_count + 5)) {
    return MB_Exception(MB_ILLEGAL_DATA_VALUE, hmb);
  } else if (start_addr + quantity > hmb->data_map.holding_register_count) {
    return MB_Exception(MB_ILLEGAL_DATA_ADDRESS, hmb);
  }

  /* Callback check before writing */
  if (hmb->write_cb
      && !hmb->write_cb(hmb, MB_WRITE_MULTIPLE_REGISTERS, start_addr, quantity)) {
    return MB_Exception(MB_SERVER_DEVICE_FAILURE, hmb);
  }

  /* Write to registers */
  for (uint16_t i = 0; i < quantity; ++i) {
    hmb->data_map.holding_registers[start_addr + i] = NTOHS(*(uint16_t *)(pdu + 5 + 2 * i));
  }

  /* Echo starting address and quantity */
  memcpy(hmb->res + hmb->res_len, pdu, 4);
  hmb->res_len += 4;
  hmb->bus_event_count++;
  return 0;
}

static uint16_t MB_ReportServerId(const uint8_t *pdu, uint16_t pdu_len, MB_ServerHandle_t *hmb) {
  if (pdu_len != 4) {
    return MB_Exception(MB_ILLEGAL_DATA_VALUE, hmb);
  }

  hmb->bus_event_count++;
  return 0;
}


static uint16_t MB_ReadFileRecord(const uint8_t *pdu, uint16_t pdu_len, MB_ServerHandle_t *hmb) {
  if (pdu_len != 4) {
    return MB_Exception(MB_ILLEGAL_DATA_VALUE, hmb);
  }

  hmb->bus_event_count++;
  return 0;
}


static uint16_t MB_WriteFileRecord(const uint8_t *pdu, uint16_t pdu_len, MB_ServerHandle_t *hmb) {
  if (pdu_len != 4) {
    return MB_Exception(MB_ILLEGAL_DATA_VALUE, hmb);
  }

  hmb->bus_event_count++;
  return 0;
}


static uint16_t MB_MaskWriteRegister(const uint8_t *pdu, uint16_t pdu_len, MB_ServerHandle_t *hmb) {
  if (pdu_len != 4) {
    return MB_Exception(MB_ILLEGAL_DATA_VALUE, hmb);
  }

  hmb->bus_event_count++;
  return 0;
}


static uint16_t MB_ReadWriteMultipleRegisters(const uint8_t *pdu, uint16_t pdu_len, MB_ServerHandle_t *hmb) {
  if (pdu_len != 4) {
    return MB_Exception(MB_ILLEGAL_DATA_VALUE, hmb);
  }

  hmb->bus_event_count++;
  return 0;
}


static uint16_t MB_ReadFifoQueue(const uint8_t *pdu, uint16_t pdu_len, MB_ServerHandle_t *hmb) {
  if (pdu_len != 4) {
    return MB_Exception(MB_ILLEGAL_DATA_VALUE, hmb);
  }

  hmb->bus_event_count++;
  return 0;
}


static uint16_t MB_ConfigurationUpdate(const uint8_t *pdu, uint16_t pdu_len, MB_ServerHandle_t *hmb) {
  /*
   * PDU format: [Param_Hi][Param_Lo][Value_Hi][Value_Lo]
   * The length is fixed at 8 bytes.
   */
  if (pdu_len != 4) {
    return MB_Exception(MB_ILLEGAL_DATA_VALUE, hmb);
  } else if (!hmb->config_cb) {
    return MB_Exception(MB_ILLEGAL_FUNCTION, hmb);
  }

  mb_param_t param = NTOHS(*(uint16_t *)pdu);
  uint16_t value = NTOHS(*(uint16_t *)(pdu + 2));

  if (!hmb->config_cb(hmb, param, value)) {
    return MB_Exception(MB_ILLEGAL_DATA_VALUE, hmb);
  }

  memcpy(hmb->res + hmb->res_len, pdu, pdu_len);
  hmb->res_len += pdu_len;
  hmb->bus_event_count++;
  return 0;
}


static uint16_t MB_IllegalFunction(const uint8_t *pdu, uint16_t pdu_len, MB_ServerHandle_t *hmb) {
  return MB_Exception(MB_ILLEGAL_FUNCTION, hmb);
}


/**
 * @brief   Initialize  Modbus ADU exception
 * @param   hmb         Modbus server handle
 * @param   exception  Exception code
 * @return  Return exception code
 */
static uint16_t MB_Exception(mb_exception_code_t exception, MB_ServerHandle_t *hmb) {
  hmb->res[1] |= 0x80;          /* Exception indicated by setting MSB */
  hmb->res[2] = exception;
  hmb->res_len = 3;
  hmb->exception_status |= 1 << (exception - 1 & 0x07);  /* Set exception code bit */
  hmb->bus_exception_count++;
  return exception;
}


/* ============================================================================
 *                  Configure interface functions
 * ============================================================================ */

/**
 * @brief   Start UART reception
 */
void MB_StartReceive(MB_ServerHandle_t *hmb) {
  if (hmb && hmb->huart) {
    /* RS485 initialization: Defaults to receive mode */
    if (hmb->rs485.enabled) {
      MB_RS485_SetRxMode(hmb);
    }

    HAL_UARTEx_ReceiveToIdle_IT(hmb->huart, hmb->req[hmb->req_idx], MB_RTU_ADU_MAX);
  }
}


/* ============================================================================
 *                    Low-level helper functions
 * ============================================================================
 */

/**
 * @brief   Calculate CRC16 checksum
 * @param   mb      Modbus port pointer
 * @param   buffer  Pointer to the data buffer
 * @param   length  Data length
 * @return  16-bit CRC checksum - byte order must be swapped before sending.
 */
static uint16_t MB_CRC16(const MB_ServerHandle_t *hmb, const uint8_t *buffer,
                         uint16_t length) {
  uint16_t crc = 0xFFFF;
  uint8_t idx;

  if (hmb->hcrc) {
    /* Use STM32 CRC peripheral */
    crc = HAL_CRC_Calculate(hmb->hcrc, (uint32_t *)buffer, (int32_t)length);
  } else if (hmb->use_crc_table) {
    /* Table lookup method: approximately 10 times faster */
    for (uint16_t i = 0; i < length; i++) {
      idx = (uint8_t)((crc ^ buffer[i]) & 0xFF);
      crc = (crc >> 8) ^ CRC16_TABLE[idx];
    }
  } else {
    /* Bit shifting method: Small code size */
    for (uint16_t i = 0; i < length; i++) {
      crc ^= buffer[i];
      for (uint8_t j = 0; j < 8; j++) {
        crc = (crc & 0x0001) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
      }
    }
  }

  return crc;
}


/**
 * @brief   RS485 switched to send mode
 */
static void MB_RS485_SetTxMode(const MB_ServerHandle_t *hmb) {
  if (hmb->rs485.enabled && hmb->rs485.de_port) {
    if (hmb->rs485.de_polarity) {
      HAL_GPIO_WritePin(hmb->rs485.de_port, hmb->rs485.de_pin, GPIO_PIN_SET);
    } else {
      HAL_GPIO_WritePin(hmb->rs485.de_port, hmb->rs485.de_pin, GPIO_PIN_RESET);
    }
  }
}


/**
 * @brief   Switch RS485 to receive mode
 */
static void MB_RS485_SetRxMode(const MB_ServerHandle_t *hmb) {
  if (hmb->rs485.enabled && hmb->rs485.de_port != NULL) {
    if (hmb->rs485.de_polarity) {
      HAL_GPIO_WritePin(hmb->rs485.de_port, hmb->rs485.de_pin, GPIO_PIN_RESET);
    } else {
      HAL_GPIO_WritePin(hmb->rs485.de_port, hmb->rs485.de_pin, GPIO_PIN_SET);
    }
  }
}

/**
 * @brief Send Modbus response frame
 * @param hmb  Modbus handle pointer
 * @param len  Length of the data portion (excluding CRC)
 */
static void MB_SendResponse(const MB_ServerHandle_t *hmb, uint16_t len) {
  /* RS485 Direction Control: Switch to transmit mode */
  MB_RS485_SetTxMode(hmb);

  if (hmb->use_dma_tx) {
    /*
     * DMA mode: Non-blocking send, freeing up the CPU
     * RS485 direction switching is handled in Modbus_TxCallback
     */
    HAL_UART_Transmit_DMA(hmb->huart, hmb->res, total_len);
  } else {
    /*
     * TODO: When protocal is NO PARITY, support setting parity bit to MARK (1).
     * See, e.g.,
     * https://control.com/forums/threads/modbus-over-serial-line-parity-question.42281/
     */

    /*
     * Blocking mode: Dynamically calculates the timeout (ms)
     * Formula: timeout = (number of bytes * 11 bits * 1000ms) / baud rate + safety margin
     * 11 bits = 1 start bit + 8 data bits + 1 parity bit + 1 stop bit Safety
     * margin: +50ms or +10% (whichever is greater)
     */
    uint32_t tx_time_ms = ((uint32_t)total_len * 11 * 1000) / hmb->baud_rate;
    uint32_t margin = (tx_time_ms / 11) > 50 ? (tx_time_ms / 11) : 50;
    uint32_t timeout = tx_time_ms + margin;

    /* Ensure the minimum timeout is 100ms */
    if (timeout < 100) {
      timeout = 100;
    }

    /* Blocking send */
    HAL_UART_Transmit(hmb->huart, hmb->res, total_len, timeout);

    /* RS485 Direction Control: Switch back to Receive Mode after waiting for
     * the TC flag */
    if (hmb->rs485.enabled) {
      while (__HAL_UART_GET_FLAG(hmb->huart, UART_FLAG_TC) == RESET) {
        HAL_Delay(10); /* Wait for transmission to complete */
      }
      MB_RS485_SetRxMode(hmb);
    }
  }
}
