/******************************************************************************
 * @file    airbeam.ino
 * @author  Rémi Pincent - INRIA
 * @date    29/07/2017
 *
 * @brief AirBeam firmware. Communicates over BT to provide some sensors value 
 * (default : NO2, Temperature, PM2.5)
 * Original code from : https://github.com/HabitatMap/AirCastingAndroidClient/tree/master/arduino/aircasting
 * Project : AirBeam
 * Contact:  Rémi Pincent - remi.pincent@inria.fr
 *
 * Revision History:
 * 
 * LICENSE :
 * AirBeam (c) by Rémi Pincent
 * AirBeam is licensed under a
 * Creative Commons Attribution-NonCommercial 3.0 Unported License.
 *
 * You should have received a copy of the license along with this
 * work.  If not, see <http://creativecommons.org/licenses/by-nc/3.0/>.
 *****************************************************************************/

/**************************************************************************
 * Include Files
 **************************************************************************/
#include <SoftwareSerial.h>
#include <DHT.h>
#include <stdlib.h>
#include <cairsens.h>
#include <logger_config.h>
#include <TimerOne.h>

/**************************************************************************
 * Macros
 **************************************************************************/
/** Temperature provided in Celcius, if not defined temp in fahrenheit */
#define CELSIUS_TEMP

#define DHTPIN 9
#define DHTTYPE DHT22   // DHT 22  (AM2302)

#define LEDPIN 13
#define BT_BUF_LEN 50
/**************************************************************************
 * Manifest Constants
 **************************************************************************/
static const uint16_t BT_PUBLISH_PERIOD = 1000;
static const uint16_t BT_STATUS_PERIOD  = 100;
static const uint16_t NO2_SAMPLE_TIME   = 100;
static const uint16_t PM_SAMPLE_TIME    = 200;
static const uint16_t LED_BLINK_PERIOD  = 200;
/**************************************************************************
 * Type Definitions
 **************************************************************************/
typedef enum{
  NOT_CONNECTED = -1,           /** never been connected */
  NOT_CONNECTED_AFTER_CONN = 0, /** not connected (but been connected) */
  CONNECTED = 1,
}EBTStatus;
/**************************************************************************
 * Global Variables
 **************************************************************************/

/**************************************************************************
 * Static Variables
 **************************************************************************/
static String airbeam_mac = "";
static unsigned long bt_publish_time = 0;
static unsigned long pm_sum = 0;
static uint16_t pm_nbmeas;
static unsigned long pm_starttime = 0;

static unsigned long no2_sum = 0;
static uint16_t no2_nbmeas = 0;
static unsigned long no2_starttime = 0;

static unsigned long bt_status_time = 0;
static volatile bool led_status = LOW;

static EBTStatus bt_connected = NOT_CONNECTED;
char bt_index = 0;
char bt_buffer[BT_BUF_LEN];

static DHT dht(DHTPIN, DHTTYPE);

/**
 Not all pins on the Leonardo and Micro support change interrupts,
 so only the following can be used for RX:
 8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).
 Moreover, multiple instances can be created and assign several soft serial ports is possible,
 but only 1 can be activated to send/receive at a time.
*/
static SoftwareSerial bt_serial(10, 11); //RX, TX
/** Use HW Uart Serial1  */
static HardwareSerial* cairsens_serial = &Serial1;

/** Communicate with Cairsens N0X sensor over UART */
static CairsensUART cairsens(cairsens_serial);

/**************************************************************************
 * Global Functions Declarations
 **************************************************************************/
 
 /**************************************************************************
 * Static Functions Declarations
 **************************************************************************/
static void initializeBT(void);
static uint8_t put_chr_buf(char c);
static void analyze_buf(void);
static void update_bt_status(void);
static void readBTMAC(void);
/** Called under interrupted context */
static void statusLedUpdate(void);
/**************************************************************************
 * Global Functions Definitions
 **************************************************************************/
void setup() {
  delay(3000);
  
  Serial.begin(500000);
  LOG_INIT(&Serial);

  /** Initialize Cairsens UART comm */
  cairsens_serial->begin(9600);

  initializeBT();

  Timer1.initialize((uint32_t) LED_BLINK_PERIOD*1000);
  Timer1.attachInterrupt(statusLedUpdate); 
  pinMode(LEDPIN, OUTPUT);
  led_status = LOW;
  digitalWrite(LEDPIN, led_status);
  
  pm_starttime = millis();
  no2_starttime = millis();
  bt_publish_time = millis();
  bt_status_time = millis();

  /** Initially BT not connected */
  Timer1.start();
}

void loop() { 
  if(millis() - bt_status_time > BT_STATUS_PERIOD)
  {
      update_bt_status();
      bt_status_time = millis();
  }  

  if ((millis() - bt_publish_time) > BT_PUBLISH_PERIOD)
  {
    bt_publish_time = millis(); // moved to create more regular periods.
    double loc_pm = (double)pm_sum/pm_nbmeas;
    loc_pm = (loc_pm*5.0)/1024;
    double loc_hppcf = (240.0*pow(loc_pm,6) - 2491.3*pow(loc_pm,5) + 9448.7*pow(loc_pm,4) - 14840.0*pow(loc_pm,3) + 10684.0*pow(loc_pm,2) + 2211.8*(loc_pm) + 7.9623);
    double loc_ugm3 = .518 + .00274 * loc_hppcf;
    if(loc_ugm3 < 0)
    {
      loc_ugm3 = 0;
    }

    int loc_humi = dht.readHumidity();
    int loc_cel = dht.readTemperature();
  
    LOG_INFO_LN(F("\n%l - AirBeam MAC: %s - Temp: %f°F %d°C - Hum : %dRH - PM2.5 : hppcf: %f  ugm^3: %f - NO2 Gas: %fppm"), 
      millis(),
      airbeam_mac.c_str(),
      (float)(loc_cel * 9)/5 + 32,
      loc_cel,
      loc_humi,
      loc_hppcf,
      loc_ugm3,
      no2_sum/(10.0*no2_nbmeas));
    
    if(bt_connected == CONNECTED){
      bt_serial.print(loc_ugm3);
      bt_serial.print((";AirBeam:"));
      bt_serial.print(airbeam_mac);
      bt_serial.print((";AirBeam-PM;Particulate Matter;PM;micrograms per cubic meter;µg/m³;0;12;35;55;150"));
      bt_serial.print("\n");

#ifdef CELSIUS_TEMP
      bt_serial.print(loc_cel);
      bt_serial.print((";AirBeam:"));
      bt_serial.print(airbeam_mac);
      bt_serial.print((";AirBeam-C;Temperature;F;degrees Celcius;C;-17;-4;10;24;38"));
#else /** Temperature in fahrenheit */
      bt_serial.print(((loc_cel * 9)/5) + 32);
      bt_serial.print((";AirBeam:"));
      bt_serial.print(airbeam_mac);
      bt_serial.print((";AirBeam-F;Temperature;F;degrees Fahrenheit;F;0;25;50;75;100"));
#endif
      bt_serial.print("\n");
      
      bt_serial.print(loc_humi);
      bt_serial.print((";AirBeam:"));
      bt_serial.print(airbeam_mac);
      bt_serial.print((";AirBeam-RH;Humidity;RH;percent;%;0;25;50;75;100"));
      bt_serial.print("\n");
      
      bt_serial.print(no2_sum/(10.0*no2_nbmeas));
      bt_serial.print((";AirBeam:"));
      bt_serial.print(airbeam_mac);
      bt_serial.print(";Cairsens A40-0020-B;Azote dioxyde;N02;micrograms per cubic meter;µg/m³;0;12;35;55;150");
      bt_serial.print("\n");
     
      no2_sum = 0;
      no2_nbmeas = 0;

      pm_sum = 0;
      pm_nbmeas = 0;
    }
  }
  
  if ((millis() - no2_starttime) > NO2_SAMPLE_TIME)
  {
    uint8_t nox_val = 0;
    if(cairsens.getInstantValue(nox_val) == CairsensUART::NO_ERROR)
    {
      LOG_INFO_LN(F("%l - current NOX value = %fppm"), millis(), nox_val/10.0);
      no2_nbmeas++;
      no2_sum += nox_val;
    }
    else
    {
      LOG_ERROR(F("Could not get Cairsens NOX value"));
    }
    no2_starttime = millis();
  }

  if ((millis() - pm_starttime) > PM_SAMPLE_TIME)
  {
    /** Read PM val */
    pm_sum = pm_sum + analogRead(0);
    pm_nbmeas++;

    pm_starttime = millis();
  }
}

 /**************************************************************************
 * Static Functions Definitions
 **************************************************************************/
void initializeBT(void)
{
  /** Initialize BT UART comm, only on SoftwareSerial instance can receive at a time
  after this call, no data can be received by cairsens_serial instance */
  bt_serial.begin(115200);
  bt_serial.println("BC:BR=08");
  delay(100);
  
  bt_serial.begin(19200);
  bt_serial.println("BC:BR=08");
  delay(100);
  
  bt_serial.begin(9600);
  bt_serial.println("BC:BR=08");
  delay(100);
  
  bt_serial.begin(19200);
  
  bt_serial.println("BC:FT=00,00,00,01,03,0000");  // It will never try to autoconnect and will always be in discoverable mode.
  delay(100);
  bt_serial.println(("BC:AD"));
  delay(100);
  readBTMAC();

  LOG_INFO_LN(F("AirBEAM BT MAC : %s"), airbeam_mac.c_str());
  
  bt_serial.print(("BC:NM=AirBeam-"));
  bt_serial.println(airbeam_mac);
  bt_buffer[0] = 0; 
}
 
uint8_t put_chr_buf(char c){
  // puts a character into the bt_buffer.
  // If the character is '\n' returns 1 (and doesn't put it into the buffer)
  // else returns 0
  if(c == '\n'){
      return 1;
  }
  if(bt_index >= BT_BUF_LEN - 2){
    LOG_ERROR(F("BT buffer full"));
    return 0;
  }
  bt_buffer[bt_index] = c;
  bt_index++;
  bt_buffer[bt_index] = 0;
  return 0;
}

void analyze_buf(void){
  // Determines if the buffer contains the SS notation
  if(bt_connected == NOT_CONNECTED){
    if(strncmp(bt_buffer, "SS=", 3) == 0){
      bt_connected = NOT_CONNECTED_AFTER_CONN;
      LOG_INFO_LN(F("1st Connect to BT"));
    }
  }
  else{
    if(strncmp(bt_buffer, "OK", 2) == 0)
    {
    }
    else{
      LOG_INFO_LN(F("ERROR: Unknown response: %s"), bt_buffer);
    }
    /** When connected : we do not get ANY ok response */
    bt_connected = NOT_CONNECTED_AFTER_CONN;
    Timer1.start();
  }
  bt_buffer[0] = 0;
  bt_index = 0;
}

void update_bt_status(){
  // This should be called in the loop with at least a hundred milliescond delay before it is called again.
  // Determine if the BT module is connected or not.
  uint8_t temp = 0;
  if(bt_serial.available()){
    while(bt_serial.available()){
      temp = put_chr_buf(bt_serial.read());
      LOG_DEBUG_LN("BT : buff length = %d - %s", strlen(bt_buffer), bt_buffer);
      if(temp){
        analyze_buf();
      }
    }
  }
  else{
    if(bt_connected == NOT_CONNECTED_AFTER_CONN || bt_connected == NOT_CONNECTED){
      bt_connected = CONNECTED;
      Timer1.stop();
      led_status = HIGH;
      digitalWrite(LEDPIN, led_status);
    }
  }
  
  clear_bt_serial();
  
  // Send out another signal for the next loop
  bt_serial.println(F("BC:UI=01"));
}

void readBTMAC(void) {
  airbeam_mac = "";
  if (bt_serial.available() > 0) {
    int h = bt_serial.available();
    for (int i = 0; i < h; i++) {
      airbeam_mac += (char)bt_serial.read();
    }
    airbeam_mac.remove(23 ,24);
    airbeam_mac.remove(0, 11);
  }
}

void clear_bt_serial(){
  while(bt_serial.available()){
    Serial.write(bt_serial.read());
  }
}

/** Called under interrupted context */
void statusLedUpdate(void)
{
  // Update connection status via the LED
  led_status =!led_status;
  digitalWrite(LEDPIN, led_status);
}

