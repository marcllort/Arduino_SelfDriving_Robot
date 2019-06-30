#include <SPI.h>
#include <EEPROM.h>
#include <lib_aci.h>
#include <aci_setup.h>
#include "uart_over_ble.h"

#include "LSEPROM.h"

// Librerias de otros perifericos, tales como TFT, touch, acelerómetro, ...
// include the TouchSD library:
#include <AR1021.h>

// include the SD library:
//#include <SPI.h>
#include <LSMaker_SD.h>

#include "LSMotor.h"

// include display
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#define TFT_DC 43
#define chipSelect 53

const int LCD_WR     = 43;
const int LCD_RESET  = 32;

//include FXOS8700CQ
#include <FXOS8700CQ.h>
FXOS8700CQ sensor = FXOS8700CQ(0x1E);

//include FXAS21002C
#include <FXAS21002C.h>
FXAS21002C sensor_g = FXAS21002C(0x20);

LSEPROM MACADDR;

//variables
byte macAddress[6];

// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
Adafruit_ILI9341 tft = Adafruit_ILI9341(chipSelect, TFT_DC, LCD_RESET);

int i = 0;
bool ii = false;
LSMt Motoreta;

const int colors[19] = {
  ILI9341_BLACK,
  ILI9341_NAVY  ,
  ILI9341_DARKGREEN   ,
  ILI9341_DARKCYAN   ,
  ILI9341_MAROON      ,
  ILI9341_PURPLE     ,
  ILI9341_OLIVE       ,
  ILI9341_LIGHTGREY  ,
  ILI9341_DARKGREY    ,
  ILI9341_BLUE        ,
  ILI9341_GREEN       ,
  ILI9341_CYAN        ,
  ILI9341_RED         ,
  ILI9341_MAGENTA     ,
  ILI9341_YELLOW      ,
  ILI9341_WHITE       ,
  ILI9341_ORANGE     ,
  ILI9341_GREENYELLOW ,
  ILI9341_PINK
};

LSTouch Touch = LSTouch();

// set up variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;
SdFile root;

// Modulo wifi ESP8266
const int RESET_WIFI    = 30;
const int ENABLE_WIFI   = 31;



bool val1 = LOW;

//Variables de temps
unsigned long t3, t4, t5, t6;

//
const int HBEAT = 5;

bool beat = LOW;

int x_old, y_old, x_new, y_new;



/**
  Put the nRF8001 setup in the RAM of the nRF8001.
*/
#include "services.h"


//--------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------- LSBATTERY
//--------------------------------------------------------------------------------------------------

void LsBattery() {
  float battery;
  float Battery_array[6];
  for (int k = 0; k < 6; k++) {
    Battery_array[k] = (float)(analogRead(1) * 5 / 1024.0 * 42.4 / 10);
  }
  //Battery_index++;
  battery = 0;
  for (int k = 0; k < 6; k++) {
    battery = battery + Battery_array[k] / 6.0;
  }

  uint8_t battery_percentatge = (uint8_t)(100 / ((1.2 - 0.9) * 8) * battery - 100 * 0.9 / (1.2 - 0.9));
  if (battery_percentatge > 100) battery_percentatge = 100;
  if (battery_percentatge < 0) battery_percentatge = 0;

  tft.setRotation(3);
  tft.setTextSize(2);

  tft.fillRect(0, 0, 320, 15, ILI9341_BLACK);
  

  tft.setCursor(150, 0);
  tft.print(String(battery));
  tft.print("V");
  tft.drawRect(225, 0, 40, 16, ILI9341_WHITE);
  if (battery_percentatge < 40) {   // vermell
    tft.fillRect(226, 1, battery_percentatge * 40 / 100.0, 14, ILI9341_RED);
  } else {
    if (battery_percentatge < 70) { // taronja
      tft.fillRect(226, 1, battery_percentatge * 40 / 100.0, 14, ILI9341_ORANGE);
    } else {                        // verd
      tft.fillRect(226, 1, battery_percentatge * 40 / 100.0, 14, ILI9341_GREEN);
    }
  }
  tft.setCursor(270, 0);
  tft.print(battery_percentatge);
  tft.print("%");

  if (battery > 12.0) {
    tft.fillRect(0, 0, 320, 15, colors[(i) % 19]);

    tft.setCursor(120, 0);
    tft.print("Charging Battery");
  } else if (battery < 8) {
    tft.fillRect(0, 0, 320, 15, colors[(i) % 19]);

    tft.setCursor(120, 0);
    tft.print("Battery Discharged");
  }
  tft.setRotation(2);

//  Serial.println("");
//  Serial.println((String)"Bateria es de " + battery + "\tVolts");
//  Serial.println("");
}


//-------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------- LSVOLSCALIBRAR
//-------------------------------------------------------------------------------------------------------

void LsVolsCalibrar() {
  long tvc;
  int st = 0, n, m;
  float battery, battery_aux;
  while (st < 5) {
    Touch.Motor();
    switch (st) {
      case 0: tvc = millis();
        tft.fillRect(0, 20, 320, 200, ILI9341_BLACK);
        tft.fillCircle(160, 120, 80, ILI9341_WHITE);
        n = 5; m = 0;
        Touch.myWireFlush();
        st++;
        break;
      case 1: 
        Touch.myWireFlush();
        tft.fillCircle(160, 120, 70, ILI9341_BLACK);
        tft.setCursor(140, 90);
        tft.setTextSize(8);
        tft.print(n);
        tvc=millis();
        st++;
        break;
      case 2:
        Touch.myWireFlush();
        if(Touch.touched()){
          int x,y;
          Touch.getPoint();
          y = (int)(((long int)(Touch.y) * 240) / 1024);
          x = (int)(((long int)(Touch.x) * 320) / 1024);
          if ((y-120)*(y-120)+(x-160)*(x-160)<80*80) st=3;
        }
        if (millis() - tvc >= 250) {
          switch (m) {
            case 0: tft.drawLine(160, 120, 230, 120, ILI9341_WHITE); break;
            case 1: tft.drawLine(160, 120, 160, 190, ILI9341_WHITE); break;
            case 2: tft.drawLine(160, 120, 90, 120, ILI9341_WHITE); break;
            case 3: tft.drawLine(160, 120, 160, 50, ILI9341_WHITE); break;
          }
          m++;
          if (m == 4) {
            n--;
            m = 0;
            st--;
          }
          if (n < 0) st = 4;
          tvc = millis();
        }
        break;
      case 3: tft.fillCircle(160, 120, 80, ILI9341_RED);
        tft.fillCircle(160, 120, 70, ILI9341_BLACK);
        tft.fillRect(20, 90, 280, 60, ILI9341_NAVY);
        tft.setCursor(30, 105);
        tft.setTextSize(4);
        tft.print(String("Calibration"));

        Motoreta.LsMtCalibrate();
//        MACADDR.lsEPputMotorCalibrate(MaxTicsLF, MaxTicsRF, MaxTicsLB, MaxTicsRB,
//                                      MinSpeedLF, MinSpeedRF, MinSpeedLB, MinSpeedRB,
//                                      MaxSpeedLF, MaxSpeedRF, MaxSpeedLB, MaxSpeedRB);
        EEPputMotorCalibrate(MaxTicsLF, MaxTicsRF, MaxTicsLB, MaxTicsRB,
                                      MinSpeedLF, MinSpeedRF, MinSpeedLB, MinSpeedRB,
                                      MaxSpeedLF, MaxSpeedRF, MaxSpeedLB, MaxSpeedRB);
        st++;
        break;
      case 4: tft.fillCircle(160, 120, 80, ILI9341_WHITE);
        tft.fillCircle(160, 120, 70, ILI9341_BLACK);
        tft.fillCircle(160, 120, 60, ILI9341_WHITE);
        tft.fillCircle(160, 120, 50, ILI9341_BLACK);
        tft.fillCircle(160, 120, 40, ILI9341_WHITE);
        tft.fillCircle(160, 120, 30, ILI9341_BLACK);
        tft.fillCircle(160, 120, 20, ILI9341_WHITE);
        tft.fillCircle(160, 120, 10, ILI9341_BLACK);
        st++;
        break;
    }
  }
}

//-----------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------- LSPERIFERICS
//-----------------------------------------------------------------------------------------------------

void LsPeriferics() {

  float battery, battery_aux;

  
  Touch.Motor();


    beat = 1;
    digitalWrite(HBEAT, beat);



    // Query the sensor
    sensor.readAccelData();
   

    tft.setRotation(3);
    //tft.fillRect(0, 200, 320, 215, ILI9341_BLACK);

    //tft.setRotation(3);
    tft.setCursor(0, 200);
    tft.setTextSize(1);
    tft.print("Ace->");
    tft.setCursor(65, 200);
    tft.print("X:");
    tft.print((float)sensor.accelData.x * 9.8 / (float)(2 << 11));
    tft.setCursor(150, 200);
    tft.print("Y:");
    tft.print((float)sensor.accelData.y * 9.8 / (float)(2 << 11));
    tft.setCursor(240, 200);
    tft.print("Z:");
    tft.print((float)sensor.accelData.z * 9.8 / (float)(2 << 11));


  //envia dades només si en rep
  if (Serial3.available()) {
    //llegeix el byte d'entrada
    char byteRebut = Serial3.read();
    //          Serial.print(byteRebut);
  }
  if (Serial.available()) {
    //llegeix el byte d'entrada
    char byteRebut = Serial.read();
    Serial3.print(byteRebut);
  }


}





#ifdef SERVICES_PIPE_TYPE_MAPPING_CONTENT
static services_pipe_type_mapping_t
services_pipe_type_mapping[NUMBER_OF_PIPES] = SERVICES_PIPE_TYPE_MAPPING_CONTENT;
#else
#define NUMBER_OF_PIPES 0
static services_pipe_type_mapping_t * services_pipe_type_mapping = NULL;
#endif

/* Store the setup for the nRF8001 in the flash of the AVR to save on RAM */
static const hal_aci_data_t setup_msgs[NB_SETUP_MESSAGES] PROGMEM = SETUP_MESSAGES_CONTENT;


static struct aci_state_t aci_state;

/*
  Temporary buffers for sending ACI commands
*/
static hal_aci_evt_t  aci_data;
//static hal_aci_data_t aci_cmd;

/*
  Timing change state variable
*/
static bool timing_change_done          = false;

/*
  Used to test the UART TX characteristic notification
*/
static uart_over_ble_t uart_over_ble;
static uint8_t         uart_buffer[20];
static uint8_t         uart_buffer_len = 0;
static uint8_t         dummychar = 0;

/*
  Initialize the radio_ack. This is the ack received for every transmitted packet.
*/
//static bool radio_ack_pending = false;


/* Define how assert should function in the BLE library */
void __ble_assert(const char *file, uint16_t line)
{
  /*
    Serial.print("ERROR ");
    Serial.print(file);
    Serial.print(": ");
    Serial.print(line);
    Serial.print("\n");
  */
  while (1);
}




//----------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------- SETUP
//----------------------------------------------------------------------------------------------

void setup(void)
{
  String texto;
  Serial3.begin(9600);
  Serial.begin(9600);



  // print the type and size of the first FAT-type volume
  uint32_t volumesize;

  volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
  volumesize *= volume.clusterCount();       // we'll have a lot of clusters
  volumesize *= 512;                            // SD card blocks are always 512 bytes

  volumesize /= 1024;
  
  volumesize /= 1024;
  

  //  Serial.println("\nFiles found on the card (name, date and size in bytes): ");
  root.openRoot(volume);

  // list all files in the card with date and size
  root.ls(LS_R | LS_DATE | LS_SIZE);


  // put your setup code here, to run once:
  Touch.begin();

  //  Serial.println("ILI9341 Test!");

  tft.begin();

  // read diagnostics (optional but can help debug problems)
  uint8_t x = tft.readcommand8(ILI9341_RDMODE);
  //  Serial.print("Display Power Mode: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDMADCTL);
  //  Serial.print("MADCTL Mode: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDPIXFMT);
  //  Serial.print("Pixel Format: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDIMGFMT);
  //  Serial.print("Image Format: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDSELFDIAG);
  //  Serial.print("Self Diagnostic: 0x"); Serial.println(x, HEX);

  // filled of circles
  
  tft.setRotation(2);
  tft.fillScreen(ILI9341_MAGENTA);


  // AFEGIT !!
  pinMode(49, OUTPUT);
  pinMode(48, OUTPUT);
  pinMode(47, OUTPUT);
  digitalWrite(49, HIGH);
  digitalWrite(48, LOW);
  digitalWrite(47, LOW);

  /**********************************************************************

          Accelerometer configuration

   ******************************************************************** */
  // Reset Acelerometro
  pinMode(38, OUTPUT);

  digitalWrite(38, LOW);

  // Reset Giroscopio
  pinMode(35, OUTPUT);
  digitalWrite(35, HIGH);

  // Initialize the FXOS8700CQ
  sensor.init();
  // Initialize the FXAS21002C
  sensor_g.init();

  /**********************************************************************

          wifi configuration

   ******************************************************************** */

  digitalWrite(RESET_WIFI, HIGH);
  digitalWrite(ENABLE_WIFI, HIGH);


  t3 = millis();


  Motoreta.begin();

  tft.fillScreen(ILI9341_MAGENTA);

  //  tft.setColor(0,255,0);
  tft.setRotation(3);

  if (!EEPcalibrated()) LsVolsCalibrar();     // AQUI

  tft.fillScreen(ILI9341_BLACK);

  tft.setCursor(50, 60);
  tft.setTextSize(6);
  tft.setTextColor(ILI9341_DARKCYAN);
  tft.println("RAYITOH");

  tft.setCursor(60, 120);
  tft.setTextSize(3);
  tft.setTextColor(ILI9341_YELLOW);
  tft.println("EL RAPIDITO");

  tft.setCursor(60, 170);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_RED);

  MACADDR.lsGetMAC(macAddress);

  tft.print("Alex I MA(R)C: ");//XX:XX:XX:XX:XX:XX");
  tft.setCursor(60, 200);
  for (int k = 0; k < 5; k++) {
    tft.print(macAddress[k], HEX);
    tft.print("-");
  }
  tft.print(macAddress[5], HEX);

  LsBattery();

  delay(1000);

  if (EEPcalibrated()) Serial.println("Calibrated");
  else Serial.println("Not calibrated");

  PercLF = 1.0;
  PercLB = 1.0;
  PercRF = 1.0;
  PercRB = 1.0;
  EEPgetMotorCalibrated(&MaxTicsLF, &MaxTicsRF, &MaxTicsLB, &MaxTicsRB,
                        &MinSpeedLF, &MinSpeedRF, &MinSpeedLB, &MinSpeedRB,
                        &MaxSpeedLF, &MaxSpeedRF, &MaxSpeedLB, &MaxSpeedRB);

  Serial.println("FORWARD:");
  Serial.print("MinSPeed: "); Serial.print(MinSpeedLF); Serial.print(" "); Serial.println(MinSpeedRF);
  Serial.print("MaxSpeed: "); Serial.print(MaxSpeedLF); Serial.print(" "); Serial.println(MaxSpeedRF);
  Serial.print("MaxTics:  "); Serial.print(MaxTicsLF); Serial.print(" "); Serial.println(MaxTicsRF);
  Serial.println("BACKWARD:");
  Serial.print("MinSPeed: "); Serial.print(MinSpeedLB); Serial.print(" "); Serial.println(MinSpeedRB);
  Serial.print("MaxSpeed: "); Serial.print(MaxSpeedLB); Serial.print(" "); Serial.println(MaxSpeedRB);
  Serial.print("MaxTics:  "); Serial.print(MaxTicsLB); Serial.print(" "); Serial.println(MaxTicsRB);


  //Serial.begin(115200);

  //Wait until the serial port is available (useful only for the Leonardo)
  //As the Leonardo board is not reseted every time you open the Serial Monitor
#if defined (__AVR_ATmega32U4__)
  while (!Serial)
  {}
  delay(5000);  //5 seconds delay for enabling to see the start up comments on the serial board
#elif defined(__PIC32MX__)
  delay(1000);
#endif

  //  Serial.println(F("Arduino setup"));
  //  Serial.println(F("Set line ending to newline to send data from the serial monitor"));

  /**
    Point ACI data structures to the the setup data that the nRFgo studio generated for the nRF8001
  */
  if (NULL != services_pipe_type_mapping)
  {
    aci_state.aci_setup_info.services_pipe_type_mapping = &services_pipe_type_mapping[0];
  }
  else
  {
    aci_state.aci_setup_info.services_pipe_type_mapping = NULL;
  }
  aci_state.aci_setup_info.number_of_pipes    = NUMBER_OF_PIPES;
  aci_state.aci_setup_info.setup_msgs         = (hal_aci_data_t*) setup_msgs;
  aci_state.aci_setup_info.num_setup_msgs     = NB_SETUP_MESSAGES;

  /*
    Tell the ACI library, the MCU to nRF8001 pin connections.
    The Active pin is optional and can be marked UNUSED
  */
  aci_state.aci_pins.board_name = BOARD_DEFAULT; //See board.h for details REDBEARLAB_SHIELD_V1_1 or BOARD_DEFAULT
  aci_state.aci_pins.reqn_pin   = 53; //SS for Nordic board, 9 for REDBEARLAB_SHIELD_V1_1                                 // era 9
  aci_state.aci_pins.rdyn_pin   = 18; //3 for Nordic board, 8 for REDBEARLAB_SHIELD_V1_1                                  // era 8
  aci_state.aci_pins.mosi_pin   = MOSI;
  aci_state.aci_pins.miso_pin   = MISO;
  aci_state.aci_pins.sck_pin    = SCK;

  aci_state.aci_pins.spi_clock_divider      = SPI_CLOCK_DIV8;//SPI_CLOCK_DIV8  = 2MHz SPI speed
  //SPI_CLOCK_DIV16 = 1MHz SPI speed

  aci_state.aci_pins.reset_pin              = 28; //4 for Nordic board, UNUSED for REDBEARLAB_SHIELD_V1_1                  // era 4
  aci_state.aci_pins.active_pin             = 29; //29
  aci_state.aci_pins.optional_chip_sel_pin  = UNUSED;

  aci_state.aci_pins.interface_is_interrupt = false; //Interrupts still not available in Chipkit
  aci_state.aci_pins.interrupt_number       = 5;                                                                          // era 1

  //We reset the nRF8001 here by toggling the RESET line connected to the nRF8001
  //If the RESET line is not available we call the ACI Radio Reset to soft reset the nRF8001
  //then we initialize the data structures required to setup the nRF8001
  //The second parameter is for turning debug printing on for the ACI Commands and Events so they be printed on the Serial
  lib_aci_init(&aci_state, false); //Marcos: I change false by true

  pinMode(HBEAT, OUTPUT);
  digitalWrite(HBEAT, HIGH);

  t3 = millis();
  t4 = millis();
  t5 = millis();
	
	
	while(ii==0){
		aci_loop();
	}
  //  Motoreta.begin();
  //LsMtInit();     // AFEGIT

}


//---------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------- UART
//---------------------------------------------------------------------------------------------

void uart_over_ble_init(void)
{
  uart_over_ble.uart_rts_local = true;
}

bool uart_tx(uint8_t *buffer, uint8_t buffer_len)
{
  bool status = false;

  if (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_TX_TX) &&
      (aci_state.data_credit_available >= 1))
  {
    status = lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, buffer, buffer_len);
    if (status)
    {
      aci_state.data_credit_available--;
    }
  }
  return status;
}

bool uart_process_control_point_rx(uint8_t *byte, uint8_t length)
{
  bool status = false;
  aci_ll_conn_params_t *conn_params;

  if (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_CONTROL_POINT_TX) )
  {
    Serial.println(*byte, HEX);
    switch (*byte)
    {
      /*
        Queues a ACI Disconnect to the nRF8001 when this packet is received.
        May cause some of the UART packets being sent to be dropped
      */
      case UART_OVER_BLE_DISCONNECT:
        /*
          Parameters:
          None
        */
        lib_aci_disconnect(&aci_state, ACI_REASON_TERMINATE);
        status = true;
        break;

      /*
        Queues an ACI Change Timing to the nRF8001
      */
      case UART_OVER_BLE_LINK_TIMING_REQ:
        /*
          Parameters:
          Connection interval min: 2 bytes
          Connection interval max: 2 bytes
          Slave latency:           2 bytes
          Timeout:                 2 bytes
          Same format as Peripheral Preferred Connection Parameters (See nRFgo studio -> nRF8001 Configuration -> GAP Settings
          Refer to the ACI Change Timing Request in the nRF8001 Product Specifications
        */
        conn_params = (aci_ll_conn_params_t *)(byte + 1);
        lib_aci_change_timing( conn_params->min_conn_interval,
                               conn_params->max_conn_interval,
                               conn_params->slave_latency,
                               conn_params->timeout_mult);
        status = true;
        break;

      /*
        Clears the RTS of the UART over BLE
      */
      case UART_OVER_BLE_TRANSMIT_STOP:
        /*
          Parameters:
          None
        */
        uart_over_ble.uart_rts_local = false;
        status = true;
        break;

      /*
        Set the RTS of the UART over BLE
      */
      case UART_OVER_BLE_TRANSMIT_OK:
        /*
          Parameters:
          None
        */
        uart_over_ble.uart_rts_local = true;
        status = true;
        break;
    }
  }
  return status;
}

//-------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------- ACI_LOOP
//-------------------------------------------------------------------------------------------------

void aci_loop()
{
  static bool setup_required = false;

  // We enter the if statement only when there is a ACI event available to be processed
  if (lib_aci_event_get(&aci_state, &aci_data))
  {
    aci_evt_t * aci_evt;
    aci_evt = &aci_data.evt;

    switch (aci_evt->evt_opcode)
    {
      /**
        As soon as you reset the nRF8001 you will get an ACI Device Started Event
      */
      case ACI_EVT_DEVICE_STARTED:
        {
          aci_state.data_credit_total = aci_evt->params.device_started.credit_available;
          switch (aci_evt->params.device_started.device_mode)
          {
            case ACI_DEVICE_SETUP:
              /**
                When the device is in the setup mode
              */
              Serial.println(F("Evt Device Started: Setup"));
              setup_required = true;
              break;

            case ACI_DEVICE_STANDBY:
              Serial.println(F("Evt Device Started: Standby"));
              //Looking for an iPhone by sending radio advertisements
              //When an iPhone connects to us we will get an ACI_EVT_CONNECTED event from the nRF8001
              if (aci_evt->params.device_started.hw_error)
              {
                delay(20); //Magic number used to make sure the HW error event is handled correctly.
              }
              else
              {
                lib_aci_connect(180/* in seconds */, 0x0050 /* advertising interval 50ms*/);
                Serial.println(F("Advertising started"));
              }
              break;
          }
        }
        break; //ACI Device Started Event

      case ACI_EVT_CMD_RSP:
        //If an ACI command response event comes with an error -> stop
        if (ACI_STATUS_SUCCESS != aci_evt->params.cmd_rsp.cmd_status)
        {
          //ACI ReadDynamicData and ACI WriteDynamicData will have status codes of
          //TRANSACTION_CONTINUE and TRANSACTION_COMPLETE
          //all other ACI commands will have status code of ACI_STATUS_SCUCCESS for a successful command
          Serial.print(F("ACI Command "));
          Serial.println(aci_evt->params.cmd_rsp.cmd_opcode, HEX);
          Serial.print(F("Evt Cmd respone: Status "));
          Serial.println(aci_evt->params.cmd_rsp.cmd_status, HEX);
        }
        if (ACI_CMD_GET_DEVICE_VERSION == aci_evt->params.cmd_rsp.cmd_opcode)
        {
          //Store the version and configuration information of the nRF8001 in the Hardware Revision String Characteristic
          lib_aci_set_local_data(&aci_state, PIPE_DEVICE_INFORMATION_HARDWARE_REVISION_STRING_SET,
                                 (uint8_t *) & (aci_evt->params.cmd_rsp.params.get_device_version), sizeof(aci_evt_cmd_rsp_params_get_device_version_t));
        }
        break;

      case ACI_EVT_CONNECTED:
        Serial.println(F("Evt Connected"));
        uart_over_ble_init();
        timing_change_done              = false;
        aci_state.data_credit_available = aci_state.data_credit_total;

        /*
          Get the device version of the nRF8001 and store it in the Hardware Revision String
        */
        lib_aci_device_version();
        break;

      case ACI_EVT_PIPE_STATUS:
        Serial.println(F("Evt Pipe Status"));
        if (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_TX_TX) && (false == timing_change_done))
        {
          lib_aci_change_timing_GAP_PPCP(); // change the timing on the link as specified in the nRFgo studio -> nRF8001 conf. -> GAP.
          // Used to increase or decrease bandwidth
          timing_change_done = true;
        }
        break;

      case ACI_EVT_TIMING:
        Serial.println(F("Evt link connection interval changed"));
        lib_aci_set_local_data(&aci_state,
                               PIPE_UART_OVER_BTLE_UART_LINK_TIMING_CURRENT_SET,
                               (uint8_t *) & (aci_evt->params.timing.conn_rf_interval), /* Byte aligned */
                               PIPE_UART_OVER_BTLE_UART_LINK_TIMING_CURRENT_SET_MAX_SIZE);
        break;

      case ACI_EVT_DISCONNECTED:
        Serial.println(F("Evt Disconnected/Advertising timed out"));
        lib_aci_connect(180/* in seconds */, 0x0100 /* advertising interval 100ms*/);
        Serial.println(F("Advertising started"));
        break;

      case ACI_EVT_DATA_RECEIVED:

        //Serial.print(F("Pipe Number: "));
        //Serial.println(aci_evt->params.data_received.rx_data.pipe_number, DEC);
        ii=!ii;
        break;

      case ACI_EVT_DATA_CREDIT:
        aci_state.data_credit_available = aci_state.data_credit_available + aci_evt->params.data_credit.credit;
        break;

      case ACI_EVT_PIPE_ERROR:
        //See the appendix in the nRF8001 Product Specication for details on the error codes
        Serial.print(F("ACI Evt Pipe Error: Pipe #:"));
        Serial.print(aci_evt->params.pipe_error.pipe_number, DEC);
        Serial.print(F("  Pipe Error Code: 0x"));
        Serial.println(aci_evt->params.pipe_error.error_code, HEX);

        //Increment the credit available as the data packet was not sent.
        //The pipe error also represents the Attribute protocol Error Response sent from the peer and that should not be counted
        //for the credit.
        if (ACI_STATUS_ERROR_PEER_ATT_ERROR != aci_evt->params.pipe_error.error_code)
        {
          aci_state.data_credit_available++;
        }
        break;

      case ACI_EVT_HW_ERROR:
        Serial.print(F("HW error: "));
        Serial.println(aci_evt->params.hw_error.line_num, DEC);

        for (uint8_t counter = 0; counter <= (aci_evt->len - 3); counter++)
        {
          Serial.write(aci_evt->params.hw_error.file_name[counter]); //uint8_t file_name[20];
        }
        Serial.println();
        lib_aci_connect(180/* in seconds */, 0x0050 /* advertising interval 50ms*/);
        Serial.println(F("Advertising started"));
        break;

    }
  }
  else
  {
    //Serial.println(F("No ACI Events available"));
    // No event in the ACI Event queue and if there is no event in the ACI command queue the arduino can go to sleep
    // Arduino can go to sleep now
    // Wakeup from sleep from the RDYN line
  }

  /* setup_required is set to true when the device starts up and enters setup mode.
     It indicates that do_aci_setup() should be called. The flag should be cleared if
     do_aci_setup() returns ACI_STATUS_TRANSACTION_COMPLETE.
  */
  if (setup_required)
  {
    if (SETUP_SUCCESS == do_aci_setup(&aci_state))
    {
      setup_required = false;
    }
  }
}


// Velocitat i Gir rebut del mobil que fa de mando, retorna les velocitats (tics/s) de cada motor
void LSMandoTuneig(char Velocitat,char Gir,float *Lsp,float *Rsp) {
    int Radi;
    float Den = 175.0;
    float kL,kR;
  //Serial.print("  Velocitat: "); Serial.print(Velocitat,DEC);
  //Serial.print("  Gir: "); Serial.print(Gir,DEC);

    if (Velocitat > 100) Velocitat = 100;
    if (Velocitat < -100) Velocitat = -100;

  // SIGMOIDE
  

  // MANDO VERSIO 1.5
    if (Gir >-3 && Gir <3) {                  // recte
      (*Rsp) = Velocitat;
      (*Lsp) = Velocitat;
    }else {
      if (Gir > 0) {                          // esquerra
        (*Rsp) = Velocitat * (1 + Gir / Den);
        if ((*Rsp) > 100.0) (*Rsp) = 100.0;
        if ((*Rsp) < -100.0) (*Rsp) = -100.0;
        (*Lsp) = (*Rsp) * (1 - 2 * Gir / Den);
      } else {                                // dreta
        (*Lsp) = Velocitat * (1 - Gir / Den);
        if ((*Lsp) > 100.0) (*Lsp) = 100.0;
        if ((*Lsp) < -100.0) (*Lsp) = -100.0;
        (*Rsp) = (*Lsp) * (1 + 2 * Gir / Den);
      }
    }

  
}



bool stringComplete = false;  // whether the string is complete
uint8_t stringIndex = 0;      //Initialize the index to store incoming chars



void magia(){
  
   float Lsp1, Rsp1;
   int opcio;
   float snsrY = (float)sensor.accelData.y * 9.8 / (float)(2 << 11);
   float snsrX = (float)sensor.accelData.x * 9.8 / (float)(2 << 11);

   if (snsrY >= 0){
      LSMandoTuneig(100,0,&Lsp1,&Rsp1);
      Motoreta.LsMtMando(Lsp1, Rsp1, 1);
      Motoreta.LsMtMotor();
     if (-2 < snsrX < 2){
       delay(1000);
      LSMandoTuneig(100,0,&Lsp1,&Rsp1);
      Motoreta.LsMtMando(Lsp1, Rsp1, 1);
      Motoreta.LsMtMotor();
     }
     else{
       while(-1 > snsrX || snsrX < 1){
        if (snsrX < 0){
          while (snsrX < -1){
            //Gira esquerra
            LSMandoTuneig(100,80,&Lsp1,&Rsp1);
            Motoreta.LsMtMando(Lsp1, Rsp1, 1);
            Motoreta.LsMtMotor();
            //Actualitza dades sensor
            sensor.readAccelData();
            snsrX = (float)sensor.accelData.x * 9.8 / (float)(2 << 11);
          }
        }
        else{
          while (snsrX > 1){
            //Gira dreta
            LSMandoTuneig(100,-80,&Lsp1,&Rsp1);
            Motoreta.LsMtMando(Lsp1, Rsp1, 1);
            Motoreta.LsMtMotor();
            //Actualitza dades sensor
            sensor.readAccelData();
            snsrX = (float)sensor.accelData.x * 9.8 / (float)(2 << 11); 
          }
        }
        sensor.readAccelData();
            snsrX = (float)sensor.accelData.x * 9.8 / (float)(2 << 11); 
       }
     }

    }
    else{
      
     if (-2 < snsrX < 2){
       //Gira 180 graus
      int snsrY2= (int) snsrY;
      
      while((int) snsrY != -snsrY2){
        LSMandoTuneig(100,80,&Lsp1,&Rsp1);
        Motoreta.LsMtMando(Lsp1, Rsp1, 1);
        Motoreta.LsMtMotor();
        sensor.readAccelData();
        snsrY = (float)sensor.accelData.y * 9.8 / (float)(2 << 11);
      }
      
     }
     else{
       if (snsrX < 0){
         while (snsrX < -1){
           //Gira esquerra
           LSMandoTuneig(100,80,&Lsp1,&Rsp1);
           Motoreta.LsMtMando(Lsp1, Rsp1, 1);
           Motoreta.LsMtMotor();
           //Actualitza dades sensor
           sensor.readAccelData();
           snsrX = (float)sensor.accelData.x * 9.8 / (float)(2 << 11);
         }
       }
       else{
         while (snsrX > 1){
           //Gira dreta
           LSMandoTuneig(100,-80,&Lsp1,&Rsp1);
           Motoreta.LsMtMando(Lsp1, Rsp1, 1);
           Motoreta.LsMtMotor();
           //Actualitza dades sensor
           sensor.readAccelData();
           snsrX = (float)sensor.accelData.x * 9.8 / (float)(2 << 11); 
         }
       }
     }
    }
}



//---------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------- LOOP
//---------------------------------------------------------------------------------------------

void loop() {

  LsPeriferics();
  if (ii==true){
    magia();
  }
 
}

/*
  COMMENT ONLY FOR ARDUINO
  SerialEvent occurs whenever a new data comes in the
  hardware serial RX.  This routine is run between each
  time loop() runs, so using delay inside loop can delay
  response.  Multiple bytes of data may be available.
  Serial Event is NOT compatible with Leonardo, Micro, Esplora
*/
void serialEvent() {

  while (Serial.available() > 0) {
    // get the new byte:
    dummychar = (uint8_t)Serial.read();
    if (!stringComplete)
    {
      if (dummychar == '\n')
      {
        // if the incoming character is a newline, set a flag
        // so the main loop can do something about it
        stringIndex--;
        stringComplete = true;
      }
      else
      {
        if (stringIndex > 19)
        {
          Serial.println("Serial input truncated");
          stringIndex--;
          stringComplete = true;
        }
        else
        {
          // add it to the uart_buffer
          uart_buffer[stringIndex] = dummychar;
          stringIndex++;
        }
      }
    }
  }
}
