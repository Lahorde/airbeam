/******************************************************************************
 * @file    cairsens_uart.h
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
 *****************************************************************************/
#ifndef CAIRSENS_H
#define CAIRSENS_H
 
/**************************************************************************
 * Include Files
 **************************************************************************/
#include <stdint.h>
#include <Arduino.h>
/**************************************************************************
 * Manifest Constants
 **************************************************************************/

/**************************************************************************
 * Type Definitions
 **************************************************************************/

/**************************************************************************
 * Global variables
 **************************************************************************/

/**************************************************************************
 * Macros
 **************************************************************************/

/**************************************************************************
 *  Class Declarations
 **************************************************************************/
class CairsensUART 
{
  /** Public types */
  public :
    typedef enum{
      NO_ERROR,
      BAD_CRC,
      TIMEOUT,
    }EError;
    
  /** Private members */
  private :
   /** Synchro Word */ 
   static const uint8_t SYNC[];
   /** Start Frame */  
   static const uint8_t STX[];
   /** End Frame */  
   static const uint8_t ETX[];
   /** Header UART end bytes */  
   static const uint8_t HEADER_UART_END_BYTES[];
   
  /** Length of LG field */
  static const uint8_t LG_LENGTH = 0x01; 
   
  /** Length of Measure field */
  static const uint8_t MEAS_LENGTH = 0x01; 
  
  /** Length of END field */ 
  static const uint8_t END_LENGTH = 0x02; 
  
  /** HeaderUART composed of :
   * - SYNC
   * - STX
   * - LG
   * - 0x30 0x01 0x02 0x03 0x04 0x05 0x06
   */ 
  static const uint8_t HEADER_UART_LENGTH; 
  
  /** Generic address allowing to communicate with any product without knowing its references */
  static const uint8_t REF_LENGTH = 0x08; 
  static const uint8_t ANY_REF[]; 
  static const uint8_t CRC_LENGTH = 0x02;
  static const uint8_t CMD_TYPE_LENGTH = 0x01; 
  static const uint8_t GET_INSTANT_VAL_RSP[]; 

  typedef enum
  { 
    GET_INSTANT_VAL = 0x12,
  }ECmd;
  
  Stream* _p_uartStream;

  /** public methods */
	public :
    
    /**
     * @brief 
     *
     * @param arg_p_uartStream an UART stream that must be initialized at 9600bps
     */
    CairsensUART(Stream* arg_p_uartStream);

    /**
     * @brief Get current value of the Cairsens. It reads last 1 minute data stored.
     *
     * @param uint8_t current raw value. In order to get value in ppm refer ./doc/CairClipUart-Interface-Protocole-downloadDataOnly.211116.pdf, p11 
     *
     * @return return code 
     */
    EError getInstantValue(uint8_t&);
      
  /** private methods */  
  private :
    
    /**
     * @brief Get sensor life in percent from Life byte value
     *
     * @param arg_u8_life
     *
     * @return 
     */
    uint8_t lifeToPercent(uint8_t arg_u8_life);
};

#endif /* CAIRSENS_H */
