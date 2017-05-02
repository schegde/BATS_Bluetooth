/*********************************************************************
  This is an example for our nRF51822 based Bluefruit LE modules

  Pick one up today in the adafruit shop!

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  MIT license, check LICENSE for more information
  All text above, and the splash screen below must be included in
  any redistribution
*********************************************************************/

#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
#include <SoftwareSerial.h>
#endif

#define DEBUG_PRINTF

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

/*=========================================================================
    APPLICATION SETTINGS

  â€‚ â€‚ FACTORYRESET_ENABLEâ€‚ â€‚  Perform a factory reset when running this sketch
  â€‚ â€‚
  â€‚ â€‚                         Enabling this will put your Bluefruit LE module
                            in a 'known good' state and clear any config
                            data set in previous sketches or projects, so
  â€‚ â€‚                         running this at least once is a good idea.
  â€‚ â€‚
  â€‚ â€‚                         When deploying your project, however, you will
                            want to disable factory reset by setting this
                            value to 0.â€‚ If you are making changes to your
  â€‚ â€‚                         Bluefruit LE device via AT commands, and those
                            changes aren't persisting across resets, this
                            is the reason why.â€‚ Factory reset will erase
                            the non-volatile memory where config data is
                            stored, setting it back to factory default
                            values.
  â€‚ â€‚ â€‚ â€‚
  â€‚ â€‚                         Some sketches that require you to bond to a
                            central device (HID mouse, keyboard, etc.)
                            won't work at all with this feature enabled
                            since the factory reset will clear all of the
                            bonding data stored on the chip, meaning the
                            central device won't be able to reconnect.
    -----------------------------------------------------------------------*/
#define FACTORYRESET_ENABLE      1
/*=========================================================================*/


// Create the bluefruit object, either software serial...uncomment these lines
/*
  SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

  Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}


/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  //  while (!Serial);  // required for Flora & Micro
  //  delay(500);


  //*****************CURRENT GAP INTERVALS:20,100,1500,30 ; :
  /* GAP INTERVAL DEFINITIONS:
    Minimum connection interval (in milliseconds)
    Maximum connection interval (in milliseconds)
    Fast Advertising interval (in milliseconds)
    Fast Advertising timeout (in seconds)
  */
  Serial.begin(115200);


  if (!ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }


  //
  //  if ( FACTORYRESET_ENABLE )
  //  {
  //    /* Perform a factory reset to make sure everything is in a known state */
  //    Serial.println(F("Performing a factory reset: "));
  //    if ( ! ble.factoryReset() ){
  //      error(F("Couldn't factory reset"));
  //    }
  //  }




  ble.println("AT+GAPINTERVALS=,,5000,");
  ble.waitForOK();
  ble.println("AT+GAPDISCONNECT");
  ble.waitForOK();
  delay(1000);
  ble.println("AT+GAPSTOPADV");
  ble.waitForOK();



  /* Disable command echo from Bluefruit */
  ble.echo(false);

  #ifdef DEBUG_PRINTF
    Serial.println("*************************************************");
    Serial.println("Bike is idle now!!!! 5 second interval advertisement");
    Serial.println("*************************************************");
  #endif
  // Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */


  //  ble.info();


}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{


  // Display command prompt
  Serial.print(F("AT > "));
  Serial.print("\n");

  // Check for user input and echo it back if anything was found
  char command[BUFSIZE + 1];
  getUserInput(command, BUFSIZE);
  char response[BUFSIZE + 1];
  char BLE_ADDR[BUFSIZE + 1];
  String RPI_BLE_ADDR= String("B8:27:EB:BF:DC:1C");  //Address of the raspberry Pi!!!
 
  // Send command


  if (command[0] == '1') {

    // Vehicle starts moving interrupt!
    response[0] = '0';


    //Start fast advertisement
    BLE_start_adv();

    int counter = 0;
//
    //Poll for connection!!! Trigger alarm ~10 seconds if device not found!!
    while (response[0] == '0') {
      delay(2000);
      ble.println("AT+GAPGETCONN");
      ble.readline(response, BUFSIZE, 2000, false);
      if(response[0] == '1'){  //Got connected 
        ble.println("AT+BLEGETPEERADDR");
        ble.readline(BLE_ADDR, BUFSIZE,2000, false);
        String BLE_PEER_ADDR = String(BLE_ADDR);
        if(!(BLE_PEER_ADDR.equals(RPI_BLE_ADDR))){ //IF wrong device!!! disconnect!
          ble.disconnect();
          response[0]='0'; //Continue search!
        }
        else{
        #ifdef DEBUG_PRINTF  
          Serial.println("*************************************************************");
          Serial.println("Master found!!!!");
          Serial.println("*************************************************************");
        #endif
          
        }
     }      
      counter++;
      if (counter == 5) {  //Counter defines the wait duration!!
        #ifdef DEBUG_PRINTF
          Serial.println("*************************************************************");
          Serial.println("I will trigger the alarm system now!!!!");
          Serial.println("*************************************************************");
        #endif
        BLE_stop_adv();
        break;
      }
    }
  
 

    //**********TODO: Check for connection for a programmed duration(~1-5 minutes?), if successfull connection from
    //                the paired device, change the gap interval to a large value and sleep for another programmed duration(~20-30 min?)
    //                If paired device not found, trigger the alarm system! and then wait for a sender response to reset to normal state!!

  }
  else if (command[0] == '2') {
    BLE_stop_adv();
  }
  else {
    ble.println(command);
    ble.waitForOK();
  }




}

/**************************************************************************/
/*!
    @brief  Checks for user input (via the Serial Monitor)
*/
/**************************************************************************/


void BLE_stop_adv() {

  ble.println("AT+GAPINTERVALS=,,5000,");
  ble.waitForOK();

  ble.println("AT+GAPDISCONNECT");
  ble.waitForOK();
  delay(1000);
  ble.println("AT+GAPSTOPADV");
  ble.waitForOK();
  #ifdef DEBUG_PRINTF
    Serial.println("*************************************************************");
    Serial.println("Advertisement stopped!!!!!!!");
    Serial.println("*************************************************************");
  #endif


}

void BLE_start_adv() {


  ble.println("AT+GAPINTERVALS=,,500,");
  ble.waitForOK();
  ble.println("AT+GAPSTARTADV");
  ble.waitForOK();
  
  #ifdef DEBUG_PRINTF
    Serial.println("*************************************************************");
    Serial.println("Advertisement started!!!!!!!");
    Serial.println("*************************************************************");
  #endif


}

void getUserInput(char buffer[], uint8_t maxSize)
{
  memset(buffer, 0, maxSize);
  while ( Serial.available() == 0 ) {
    delay(1);
  }

  uint8_t count = 0;

  do
  {
    count += Serial.readBytes(buffer + count, maxSize);
    delay(2);
  } while ( (count < maxSize) && !(Serial.available() == 0) );
}

