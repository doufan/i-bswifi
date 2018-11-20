/*
 *  BLE-specific cipher implementations optimized for Silicon Labs devices
 *  with a CRYPTO peripheral.
 *
 *  Copyright (C) 2017, Silicon Labs, http://www.silabs.com
 *  SPDX-License-Identifier: Apache-2.0
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may
 *  not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#include "crypto_ble.h"
#include "em_device.h"

#if defined(CRYPTO_PRESENT)

#include "crypto_management.h"
#include "em_crypto.h"
#include <string.h>

/***************************************************************************//**
 * @brief
 *   Write a 128 bit value (optionally unaligned) into a crypto register.
 *
 * @note
 *   This function provide a low-level api for writing to the multi-word
 *   registers in the crypto peripheral. Applications should prefer to use
 *   @ref CRYPTO_DataWrite, @ref CRYPTO_DDataWrite or @ref CRYPTO_QDataWrite
 *   for writing to the DATA, DDATA and QDATA registers.
 *
 * @param[in]  reg
 *   Pointer to the crypto register.
 *
 * @param[in]  val
 *   This is a pointer to 4 32 bit integers that contains the 128 bit value
 *   which will be written to the crypto register.
 ******************************************************************************/
__STATIC_INLINE void CRYPTO_DataWriteUnaligned(volatile uint32_t * reg,
                                               const uint8_t * val)
{
  /* Check data is 32bit aligned, if not move to temporary buffer before
     writing.*/
  if ((uint32_t)val & 0x3)
  {
    uint32_t temp[4];
    memcpy(temp, val, 16);
    CRYPTO_DataWrite(reg, temp);
  }
  else
  {
    CRYPTO_DataWrite(reg, (uint32_t*)val);
  }
}

/***************************************************************************//**
 * @brief
 *   Read a 128 bit value from a crypto register into optionally unaligned
 *   buffer.
 *
 * @note
 *   This function provide a low-level api for reading one of the multi-word
 *   registers in the crypto peripheral. Applications should prefer to use
 *   @ref CRYPTO_DataRead, @ref CRYPTO_DDataRead or @ref CRYPTO_QDataRead
 *   for reading the value of the DATA, DDATA and QDATA registers.
 *
 * @param[in]  reg
 *   Pointer to the crypto register.
 *
 * @param[out]  val
 *   This is a pointer to an array that is capable of holding 4 32 bit integers
 *   that will be filled with the 128 bit value from the crypto register.
 ******************************************************************************/
__STATIC_INLINE void CRYPTO_DataReadUnaligned(volatile uint32_t * reg,
                                              uint8_t * val)
{
  /* Check data is 32bit aligned, if not, read into temporary buffer and
     then move to user buffer. */
  if ((uint32_t)val & 0x3)
  {
    uint32_t temp[4];
    CRYPTO_DataRead(reg, temp);
    memcpy(val, temp, 16);
  }
  else
  {
    CRYPTO_DataRead(reg, (uint32_t*)val);
  }
}

/*
 * CCM buffer encryption optimized for BLE
 */
int mbedtls_ccm_encrypt_and_tag_ble( unsigned char       *data,
                                     size_t               length,
                                     const unsigned char *key,
                                     const unsigned char *iv,
                                     unsigned char        header,
                                     unsigned char       *tag )
{
    /* Local variables used to optimize load/store sequences from memory to
     crypto. We want to load all 4 32bit data words to local register
     variables in the first sequence, then store them all in the second
     sequence.*/
    register uint32_t iv0;
    register uint32_t iv1;
    register uint32_t iv2;
    register uint32_t iv3;
    /* Mangling DDATA1 (KEY), DDATA2 (= DATA0/DATA1), DDATA3 (=DATA2/DATA3),
       DDATA4 (KEYBUF). Max execution length = 16 */
    CRYPTO_TypeDef   *device = crypto_management_acquire_preemption(
                                 CRYPTO_MANAGEMENT_SAVE_DDATA1
                                 | CRYPTO_MANAGEMENT_SAVE_DDATA2
                                 | CRYPTO_MANAGEMENT_SAVE_DDATA3
                                 | CRYPTO_MANAGEMENT_SAVE_DDATA4
                                 | CRYPTO_MANAGEMENT_SAVE_UPTO_SEQ4 );

    /* Setup CRYPTO for AES-128 mode (256 not supported) */
    device->CTRL      = CRYPTO_CTRL_AES_AES128;
    device->WAC       = 0UL;

    if (key)
    {
        CRYPTO_KeyBuf128Write(device, (uint32_t *)key);
    }

    /* Calculate Counter IV for encryption. */
    iv0 = 0x01 | (*(uint32_t *)(&iv[0]) << 8);
    iv1 = *(uint32_t *)(&iv[3]);
    iv2 = *(uint32_t *)(&iv[7]);
    iv3 = *(uint16_t *)(&iv[11]);

    /* Store Counter IV in crypto->DATA1 */
    device->DATA1 = iv0;
    device->DATA1 = iv1;
    device->DATA1 = iv2;
    device->DATA1 = iv3;

    /* Calculate CBC IV for authentication. */
    iv0 |= 0x49;
    iv3 |= __REV(length);

    /* Store CBC IV in device->DATA0 */
    device->DATA0 = iv0;
    device->DATA0 = iv1;
    device->DATA0 = iv2;
    device->DATA0 = iv3;

    /* Store header in device->DATA3 */
    device->DATA3 = 0x0100 | (header << 16);
    device->DATA3 = 0;
    device->DATA3 = 0;
    device->DATA3 = 0;

    device->SEQCTRL  = length;
    device->SEQCTRLB = 0;

    /* The following code is tested to run faster than using instruction
     sequences. */
    device->CMD = CRYPTO_CMD_INSTR_AESENC;
    device->CMD = CRYPTO_CMD_INSTR_DATA3TODATA0XOR;
    device->CMD = CRYPTO_CMD_INSTR_AESENC;
    device->CMD = CRYPTO_CMD_INSTR_DATA0TODATA3;

    CRYPTO_EXECUTE_16(device,
                      CRYPTO_CMD_INSTR_EXECIFA,

                      // CRYPTO_CMD_INSTR_BUFTODATA0,
                      CRYPTO_CMD_INSTR_DMA0TODATA,
                      CRYPTO_CMD_INSTR_DATA0TODATA2, // save DMA value

                      CRYPTO_CMD_INSTR_DATA3TODATA0XOR,
                      CRYPTO_CMD_INSTR_AESENC,
                      CRYPTO_CMD_INSTR_DATA0TODATA3,
                      CRYPTO_CMD_INSTR_DATA1INC,
                      CRYPTO_CMD_INSTR_DATA1TODATA0,
                      CRYPTO_CMD_INSTR_AESENC,
                      //CRYPTO_CMD_INSTR_DATA0TOBUFXOR,
                      CRYPTO_CMD_INSTR_DATA2TODATA0XOR,//data0 = data0 xor dma
                      CRYPTO_CMD_INSTR_DATATODMA0,

                      CRYPTO_CMD_INSTR_EXECIFLAST,
                      CRYPTO_CMD_INSTR_DATA1INCCLR,
                      CRYPTO_CMD_INSTR_DATA1TODATA0,
                      CRYPTO_CMD_INSTR_AESENC,
                      CRYPTO_CMD_INSTR_DATA3TODATA0XOR
                      );

    uint32_t tempBuf[4];

    while (length)
    {
        if (length < 16) {
            /* Use temporary buffer for zero padding */
            memset( tempBuf, 0, 16 );
            memcpy( tempBuf, data, length );
            CRYPTO_DataWrite( &device->DATA0, tempBuf );
            CRYPTO_DataRead( &device->DATA0, tempBuf );
            memcpy( data, tempBuf, length );
            length = 0;
        } else {
            CRYPTO_DataWriteUnaligned( &device->DATA0, data );
            CRYPTO_DataReadUnaligned( &device->DATA0, data );
            length  -= 16;
            data    += 16;
        }
    }

    /* Read authentication tag from DATA0 register. */
    CRYPTO_DataRead( &device->DATA0, tempBuf );
    *((uint32_t*)tag) = tempBuf[0];

    crypto_management_release_preemption( device );

    return 0;
}

/*
 * CCM buffer authenticated decryption optimized for BLE
 */
int mbedtls_ccm_auth_decrypt_ble( unsigned char       *data,
                                  size_t               length,
                                  const unsigned char *key,
                                  const unsigned char *iv,
                                  unsigned char        header,
                                  unsigned char       *tag )
{
    /* Local variables used to optimize load/store sequences from memory to
     crypto. We want to load all 4 32bit data words to local register
     variables in the first sequence, then store them all in the second
     sequence.*/
    register uint32_t iv0;
    register uint32_t iv1;
    register uint32_t iv2;
    register uint32_t iv3;
    /* Mangling DDATA1 (KEY), DDATA2 (= DATA0/DATA1), DDATA3 (=DATA2/DATA3),
       DDATA4 (KEYBUF). Max execution length = 18 */
    CRYPTO_TypeDef   *device = crypto_management_acquire_preemption(
                                 CRYPTO_MANAGEMENT_SAVE_DDATA1
                                 | CRYPTO_MANAGEMENT_SAVE_DDATA2
                                 | CRYPTO_MANAGEMENT_SAVE_DDATA3
                                 | CRYPTO_MANAGEMENT_SAVE_DDATA4
                                 | CRYPTO_MANAGEMENT_SAVE_UPTO_SEQ4 );

    /* Setup CRYPTO for AES-128 mode (256 not supported) */
    device->CTRL      = CRYPTO_CTRL_AES_AES128;
    device->WAC       = 0UL;

    if (key)
    {
        CRYPTO_KeyBuf128Write(device, (uint32_t *)key);
    }

    /* Calculate Counter IV for encryption. */
    iv0 = 0x01 | (*(uint32_t *)(&iv[0]) << 8);
    iv1 = *(uint32_t *)(&iv[3]);
    iv2 = *(uint32_t *)(&iv[7]);
    iv3 = *(uint16_t *)(&iv[11]);

    /* Store Counter IV in crypto->DATA1 */
    device->DATA1 = iv0;
    device->DATA1 = iv1;
    device->DATA1 = iv2;
    device->DATA1 = iv3;

    /* Calculate CBC IV for authentication. */
    iv0 |= 0x49;
    iv3 |= __REV(length);

    /* Store CBC IV in device->DATA0 */
    device->DATA0 = iv0;
    device->DATA0 = iv1;
    device->DATA0 = iv2;
    device->DATA0 = iv3;

    /* Store header in device->DATA3 */
    device->DATA3 = 0x0100 | (header << 16);
    device->DATA3 = 0;
    device->DATA3 = 0;
    device->DATA3 = 0;

    device->SEQCTRL  = length;
    device->SEQCTRLB = 0;

    /* The following code is tested to run faster than using instruction
     sequences. */
    device->CMD = CRYPTO_CMD_INSTR_AESENC;
    device->CMD = CRYPTO_CMD_INSTR_DATA3TODATA0XOR;
    device->CMD = CRYPTO_CMD_INSTR_AESENC;
    device->CMD = CRYPTO_CMD_INSTR_DATA0TODATA3;

    CRYPTO_EXECUTE_18(device,
                      CRYPTO_CMD_INSTR_EXECIFA,
                      /* AESDRV_CTR_PREPARE_PROC */
                      CRYPTO_CMD_INSTR_DATA1INC,
                      CRYPTO_CMD_INSTR_DATA1TODATA0,
                      CRYPTO_CMD_INSTR_AESENC,

                      // CRYPTO_CMD_INSTR_BUFTODATA0XOR,
                      // CRYPTO_CMD_INSTR_DATA0TOBUF,
                      CRYPTO_CMD_INSTR_DATA0TODATA2,
                      CRYPTO_CMD_INSTR_DMA0TODATA,
                      CRYPTO_CMD_INSTR_DATA2TODATA0XORLEN,
                      CRYPTO_CMD_INSTR_DATATODMA0,

                      CRYPTO_CMD_INSTR_DATA0TODATA2,
                      CRYPTO_CMD_INSTR_DATA3TODATA0,
                      CRYPTO_CMD_INSTR_DATA2TODATA0XORLEN,

                      CRYPTO_CMD_INSTR_AESENC,
                      CRYPTO_CMD_INSTR_DATA0TODATA3,

                      CRYPTO_CMD_INSTR_EXECIFLAST,
                      CRYPTO_CMD_INSTR_DATA1INCCLR,
                      CRYPTO_CMD_INSTR_DATA1TODATA0,
                      CRYPTO_CMD_INSTR_AESENC,
                      CRYPTO_CMD_INSTR_DATA3TODATA0XOR
                      );

    uint32_t tempBuf[4];

    while (length)
    {
        if (length < 16) {
            /* Use temporary buffer for zero padding */
            memset( tempBuf, 0, 16 );
            memcpy( tempBuf, data, length );
            CRYPTO_DataWrite( &device->DATA0, tempBuf );
            CRYPTO_DataRead( &device->DATA0, tempBuf );
            memcpy( data, tempBuf, length );
            length = 0;
        } else {
            CRYPTO_DataWriteUnaligned( &device->DATA0, data );
            CRYPTO_DataReadUnaligned( &device->DATA0, data );
            length  -= 16;
            data    += 16;
        }
    }

    /* Read authentication tag from DATA0 register. */
    CRYPTO_DataRead( &device->DATA0, tempBuf );
    crypto_management_release_preemption( device );

    if ( *((uint32_t*)tag) == tempBuf[0] ) {
        return 0;
    } else {
        return MBEDTLS_ERR_CCM_AUTH_FAILED;
    }
}

/*
 * Process a table of BLE RPA device keys and look for a
 * match against the supplied hash
 */
int mbedtls_process_ble_rpa(  const unsigned char   keytable[],
                              uint32_t              keymask,
                              uint32_t              prand,
                              uint32_t              hash )
{
    size_t index;
    uint32_t data_register[4] = {0};
    data_register[3] = __REV(prand);

    /* Mangling DDATA1 (KEY) and DDATA2 (= DATA0/DATA1). Max execution length = 2 */
    CRYPTO_TypeDef *device = crypto_management_acquire_preemption(
                              CRYPTO_MANAGEMENT_SAVE_DDATA1
                              | CRYPTO_MANAGEMENT_SAVE_DDATA2
                              | CRYPTO_MANAGEMENT_SAVE_UPTO_SEQ0 );
    /* Set up CRYPTO to do AES, and load prand */
    device->CTRL     = CRYPTO_CTRL_AES_AES128 | CRYPTO_CTRL_KEYBUFDIS;
    device->WAC      = 0UL;

    CRYPTO_DataWrite(&device->DATA1, (uint32_t*)data_register);

    /* For each key, execute AES encrypt operation and compare w hash */
    /* Read result of previous iteration first to minimize stall while waiting
       for AES to finish */
    int currentindex = -1;
    for ( index = 0; index < 32; index++ ) {
        if ( (keymask & (1U << index)) == 0 ) {
            continue;
        }

        CRYPTO_DataRead(&device->DATA0, data_register);
        CRYPTO_DataWrite(&device->KEY, (uint32_t*)(&keytable[index * 16]));
        CRYPTO_EXECUTE_2( device,
                          CRYPTO_CMD_INSTR_DATA1TODATA0,
                          CRYPTO_CMD_INSTR_AESENC );

        if ( ( currentindex >= 0 )
             && ( (data_register[3] & 0xFFFFFF00) == __REV(hash) ) ) {
            crypto_management_release_preemption(device);
            return currentindex;
        }

        currentindex = index;
    }

    /* Read result of last encryption and check for hash */
    CRYPTO_DataRead(&device->DATA0, data_register);
    crypto_management_release_preemption(device);

    if ( (data_register[3] & 0xFFFFFF00) == __REV(hash) ) {
        return currentindex;
    }

    return -1;
}

#endif /* CRYPTO_PRESENT */
