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

CairsensUART::EError CairsensUART::getNO2InstantVal(uint8_t& arg_u8_val)
{
  EError loc_e_err = getInstantVal(arg_u8_val);
  if(loc_e_err == NO_ERROR && arg_u8_val > NO2_MAX)
  {
    LOG_INFO("bad range"); 
    return BAD_RANGE;
  }
  return loc_e_err;
}

/** Refer http://www2.dmu.dk/AtmosphericEnvironment/Expost/database/docs/PPM_conversion.pdf */
float CairsensUART::ppbToPpm(EPollutantType arg_e_polType, float arg_f_ppb) 
{
  float loc_f_ret = 0.0; 
  switch(arg_e_polType)
  { 
    case NO2:
      /** 1ppb = 1.88µg/m3 */
      loc_f_ret = 1.88*arg_f_ppb;
      break;
    default:
      ASSERT(false);
  }
  return loc_f_ret; 
} 
/**************************************************************************
* Private Methods Definitions
**************************************************************************/
CairsensUART::EError CairsensUART::getInstantVal(uint8_t& arg_u8_instVal)
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

  if(loc_u8_id != sizeof(loc_au8_getInstValFrame))
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
  if(memcmp(&loc_au8_getInstValFrame[0], SYNC, sizeof(SYNC)) != 0 
      || memcmp(&loc_au8_getInstValFrame[sizeof(SYNC)], STX, sizeof(STX)) != 0
      || memcmp(&loc_au8_getInstValFrame[sizeof(loc_au8_getInstValFrame) - sizeof(ETX)], ETX, sizeof(ETX)) != 0)
  {
    LOG_ERROR("Bad field in response - try to empty stream");
    _p_uartStream->readBytes(loc_au8_getInstValFrame, sizeof(loc_au8_getInstValFrame)); 
    return BAD_FRAME;
  }

  /** TODO check CRC */
  if(FCRC(&loc_au8_getInstValFrame[START_CRC_POS],  loc_au8_getInstValFrame[START_CRC_POS]) != 0)
  {
    LOG_ERROR("Bad CRC - try to empty stream");
    _p_uartStream->readBytes(loc_au8_getInstValFrame, sizeof(loc_au8_getInstValFrame)); 
    return  BAD_CRC; 
  }
  /** TODO check Life */
 
  arg_u8_instVal = loc_au8_getInstValFrame[HEADER_UART_LENGTH + sizeof(ANY_REF) + sizeof(GET_INSTANT_VAL_RSP)];
  return NO_ERROR; 
} 

uint32_t CairsensUART::FCRC( unsigned char Frame[],unsigned char lg)
{
  uint32_t Poly = 0x8408;
  uint32_t Crc;
  unsigned char j,i_bits,carry;
  Crc=0;
  for (j=0;j<lg; j++) {
    Crc = Crc ^ Frame[j];
    for ( i_bits = 0; i_bits < 8; i_bits++ ) {
      carry = Crc & 1;
      Crc = Crc/2;
      if(carry) {
        Crc = Crc ^ Poly;
      }
    }
  }
  return Crc;
}

