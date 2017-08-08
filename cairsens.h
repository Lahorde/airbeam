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
      BAD_RANGE,
      BAD_FRAME
    }EError;
    
   typedef enum{
     NO2,
     NO,
   }EPollutantType;
   
   /** Max cairclip NO2 value in ppb */
   static const uint8_t NO2_MAX = 250;
    
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
  
  /** CRC computed from byte at this index to last bytes - 2 (CRC length) */
  static const uint8_t START_CRC_POS = 0x02; 
  static const uint8_t CRC_LEN       = 0x02; 

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
     * @brief Get NO2 concentration in ppb. 
     * Cairsens NO2 concentration range [0 to 250]ppb
     *
     * @param uint8_t
     *
     * @return 
     */
    EError getNO2InstantVal(uint8_t& arg_u8_val);

    /**
     * Converts Parts per billion to µg/m3 http://www2.dmu.dk/AtmosphericEnvironment/Expost/database/docs/PPM_conversion.pdf
     */
    static float ppbToPpm(EPollutantType arg_e_polType, float arg_f_ppb); 
      
  /** private methods */  
  private :
    /**
     * @brief Get current value of the Cairsens. It reads last 1 minute data stored.
     *
     * @param uint8_t current raw value. In order to get value in ppm refer ./doc/CairClipUart-Interface-Protocole-downloadDataOnly.211116.pdf, p11 
     *
     * @return return code 
     */
    EError getInstantVal(uint8_t&);
    
    /**
     * @brief Get sensor life in percent from Life byte value
     *
     * @param arg_u8_life
     *
     * @return 
     */
    uint8_t lifeToPercent(uint8_t arg_u8_life);
    

    /**
     * @brief Compute CRC
     *
     * @param Frame[]
     * @param lg
     *
     * @return 
     */
    uint32_t FCRC(unsigned char Frame[], unsigned char lg);
};

#endif /* CAIRSENS_H */
