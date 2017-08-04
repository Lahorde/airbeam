/******************************************************************************
 * @file    cairsens.cpp
 * @author  Rémi Pincent - INRIA
 * @date    28/07/2017
 *
 * @brief Cairsens UART library. Refer : ./doc/CairClipUart-Interface-Protocole-downloadDataOnly.211116.pdf
 *
 * Project : cairsens
 * Contact:  Rémi Pincent - remi.pincent@inria.fr
 *
 * Revision History:
 * https://github.com/OpHaCo/hoverbot.git 
 * 
 * LICENSE :
 * cairsens (c) by Rémi Pincent
 * cairsens is licensed under a
 * Creative Commons Attribution-NonCommercial 3.0 Unported License.
 *
 * You should have received a copy of the license along with this
 * work.  If not, see <http://creativecommons.org/licenses/by-nc/3.0/>.

/**************************************************************************
 * Include Files
 **************************************************************************/
#include "cairsens.h"

#include <Arduino.h>
#include <logger_config.h>
/**************************************************************************
 * Manifest Constants
 **************************************************************************/
const uint8_t CairsensUART::SYNC[] = {0xFF};
/** Start Frame */  
const uint8_t CairsensUART::STX[]  = {0x02};
/** End Frame */  
const uint8_t CairsensUART::ETX[]  = {0x03};
/** Header UART end bytes */  
const uint8_t CairsensUART::HEADER_UART_END_BYTES[] = {0x30, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
/** Generic address allowing to communicate with any product without knowing its references */
const uint8_t CairsensUART::ANY_REF[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 
const uint8_t CairsensUART::HEADER_UART_LENGTH = sizeof(CairsensUART::SYNC) + sizeof(CairsensUART::STX) + CairsensUART::LG_LENGTH + sizeof(CairsensUART::HEADER_UART_END_BYTES); 
const uint8_t CairsensUART::GET_INSTANT_VAL_RSP[] = {0x13}; 
/**************************************************************************
 * Type Definitions
 **************************************************************************/

/**************************************************************************
 * Public Methods Definitions
 **************************************************************************/
CairsensUART::CairsensUART(Stream* arg_p_uartStream):
  _p_uartStream(arg_p_uartStream)
{
   
} 

CairsensUART::EError CairsensUART::getInstantValue(uint8_t& arg_u8_instVal)
{
  /** same buffer used for Request and response, give it response length (> request length) */ 
  uint8_t loc_au8_getInstValFrame[HEADER_UART_LENGTH + sizeof(ANY_REF) + sizeof(GET_INSTANT_VAL_RSP) + MEAS_LENGTH + END_LENGTH + CRC_LENGTH + sizeof(ETX)]; 
  uint8_t loc_u8_id = 0; 
  static const uint8_t GET_INST_VAL_TIMEOUT = 30; 

  memcpy(&loc_au8_getInstValFrame[loc_u8_id], SYNC, sizeof(SYNC));
  loc_u8_id += sizeof(SYNC);
  memcpy(&loc_au8_getInstValFrame[loc_u8_id], STX, sizeof(STX));
  loc_u8_id += sizeof(STX);
  /** Length */ 
  loc_au8_getInstValFrame[loc_u8_id++] = 0x13;
  memcpy(&loc_au8_getInstValFrame[loc_u8_id], HEADER_UART_END_BYTES, sizeof(HEADER_UART_END_BYTES));
  loc_u8_id += sizeof(HEADER_UART_END_BYTES);
  memcpy(&loc_au8_getInstValFrame[loc_u8_id], ANY_REF, sizeof(ANY_REF));
  loc_u8_id += sizeof(ANY_REF);
  /** GetInstantVal cmd */ 
  loc_au8_getInstValFrame[loc_u8_id++] = GET_INSTANT_VAL;
  
  /** TODO CRC */ 
  loc_au8_getInstValFrame[loc_u8_id++] = 0xAF;
  loc_au8_getInstValFrame[loc_u8_id++] = 0x88;
  
  
  memcpy(&loc_au8_getInstValFrame[loc_u8_id], ETX, sizeof(ETX));
  loc_u8_id += sizeof(ETX);
  
  /** Write GetValueFrame buffer to UART */
  LOG_DEBUG("Send frame : ");
  for(loc_u8_id = 0; loc_u8_id < HEADER_UART_LENGTH + sizeof(ANY_REF) + CMD_TYPE_LENGTH + CRC_LENGTH + sizeof(ETX); loc_u8_id++)
  {
    LOG_DEBUG("%x ", loc_au8_getInstValFrame[loc_u8_id]);
  }
  LOG_DEBUG_LN("");
  _p_uartStream->write(loc_au8_getInstValFrame, HEADER_UART_LENGTH + sizeof(ANY_REF) + CMD_TYPE_LENGTH + CRC_LENGTH + sizeof(ETX));
  
  /** get response */ 
  _p_uartStream->setTimeout(GET_INST_VAL_TIMEOUT);
   
  loc_u8_id = _p_uartStream->readBytes(loc_au8_getInstValFrame, sizeof(loc_au8_getInstValFrame)); 

  if(loc_u8_id == 0)
  {
    LOG_ERROR("Timeout when getting GetInstantValue RESP");
    return TIMEOUT;
  }
  LOG_DEBUG("received frame : ");
  for(loc_u8_id = 0; loc_u8_id < sizeof(loc_au8_getInstValFrame); loc_u8_id++)
  {
    LOG_DEBUG("%x ", loc_au8_getInstValFrame[loc_u8_id]);
  }
  LOG_DEBUG_LN("");
  
  /** TODO check RSP */
  /** TODO check CRC */
  /** TODO check Life */
  
  arg_u8_instVal = loc_au8_getInstValFrame[HEADER_UART_LENGTH + sizeof(ANY_REF) + sizeof(GET_INSTANT_VAL_RSP)];
  return NO_ERROR; 
} 
 /**************************************************************************
 * Private Methods Definitions
 **************************************************************************/
