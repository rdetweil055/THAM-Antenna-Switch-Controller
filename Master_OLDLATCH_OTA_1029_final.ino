// Clearn up tasks
// 
// 1) merge config structure into IAS field
//       Does not look possible yet,
//
// This version can be use with or without the IOTAppStory library.
//      the USE_APPSTORY define controls the compile option.
//
// This device firmware is a relay control and status display.
// This firmware is designed for a 2X6 antenna switch, however
// with some modification it could server any number of inputs
// and outputs.  The hardware selection is 8266.  If the device
// is to be mounted on a tower some distance from the radion 
// station an ability to do a OTA update is needed.  Also an 
// external antenna is needed.  I used the WROOM-02U as the 
// 8266 device that has a IPEX anenna connector and a 6db gain
// panel mount 2.4 ghz antenna in a weather proof enclosure.
//
// In many routines there is a parameter call loopFlag,  due to
// the async nature of the program we do the heavy lifting of any
// function in the main loop(). This lets the handlers
// format a HTTML return string to the device as fast as possible.
// In order to use a routine the routine must first be called
// with a false loopFlag, this allows the handlers to be nonblocking
// Then the routine must be called again with a true loopFlag.
// The routine will then do the function.
// 
// Do be careful with the pre processor definition "#".  I use them
// extensively and sometimes to a fault.
//
// As I have several old style UCN5812 serial latches on hand, 
// I made this firmware for them.  In the future I will use an
// I2C GPIO extender and opto isolators to power the relays.
// Programming for this is not included here.  A future update
// will have a USE_I2C_EXTENDER directive to include that code.
//
// As this is designed to be mounted far away from a computer,
// this design is to be able to detect that a firmware update is 
// available for the device and automaticly download the new
// firmware.  
//
// The design does not update automatically,  the user must 
// requests the firmware update check via the main web page.
//
// As these devices control antenna relays that may have significant power
// running through them, the device must only do the firmware update when 
// directed by the user through the main web page request.
//
// 2X6 switch notes
// 
// NOTE:  Only 1 antenna port can be active.  if more the device has more 
//        than one radio input, then device must prevent any other request
//        for that antenna port.
//
// NOTE:  8266 can only handle 9 safe GPIO pins, 2 more if Serial line 
//        is used,then no debugging...
//        Anything more needs a serial latch driver chip
//
// CORE FUNCTIONS
// 1) Allow user to connect a radio port to an antenna port
// 2) Send commands to relay driver to effect the user command.
// 3) Display Relay Status to the user
// 4) Function independent of internet when needed
// 6) Allow for Firmware update.
// 7) Operate at a long distance from the station.
//
// Sub Functions
// 1) ability connect to WiFi network as a client.
// 2) ability to function as an AP (access point) without internet
// 3) display through Web pages on PC or Smart device
// 4) Ability to rename Antenna ports
//
// 
#define DEBUGLEVEL 1                    // Debug print level.
#include "DebugUtils.h"                 // set DEBUG Serial.print definitions
                                        // local file...
#define USE_APPSTORY

#ifdef USE_APPSTORY
// NOTE,  IAS library config.h is modified...
//#define CFG_PAGE_INFO    false           // include the info page in Config mode
//#define CFG_PAGE_IAS     false           // include the IAS page in Config mode


#define COMPDATE __DATE__ __TIME__      // set up for IOTAppStory library
#define MODEBUTTON 0                    // Button pin on the esp for selecting modes. D3 for the Wemos!

#include <IOTAppStory.h>                // bring in the IotAppStory.com library which brings in all pre-requisites
#else // USE_APPSTORY

#define SERIAL_SPEED                                            115200

#ifdef ESP32
                #include <WiFi.h>
                #include <WiFiMulti.h>
                #include <ESPmDNS.h>
                #include "esp32/boardInfo.h"
                #include <AsyncTCP.h>                    // https://github.com/me-no-dev/AsyncTCP
                //#include <HTTPClient.h>
#elif defined  ESP8266
                #include <ESP8266WiFi.h>
                #include <ESP8266WiFiMulti.h>
                #include <ESP8266mDNS.h>
                #include <ESPAsyncTCP.h>                 // https://github.com/me-no-dev/ESPAsyncTCP
#endif

        #include "ESPhttpUpdateIasMod.h"
        #include <DNSServer.h>
        #include <EEPROM.h>
        #include <ESPAsyncWebServer.h>               // https://github.com/me-no-dev/ESPAsyncWebServer
        
#if defined  ESP8266
  #define EEPROM_SIZE                                     4096    // EEPROM_SIZE depending on device
#elif defined ESP32
  #define EEPROM_SIZE                                     1984
#else
  #define EEPROM_SIZE                                     512    // this is a guess.
#endif  // ESP8266

#endif  // USE_APPSTORY

#define SETUP_LED_TIME   300            // interval used during setup and in 'C' config mode
#define RUN_LED_TIME    1000            // interval used after setup and in 'N' normal mode

#define UCN5812_DRIVE                   // use old UCH5812 serial latch.

#include <Ticker.h>                     // for the runLED & some debug timing

#ifdef USE_APPSTORY
  IOTAppStory IAS(COMPDATE, MODEBUTTON);  // Initialize IOTAppStory Library
  char* runLED     = "2";                 // A run led helps to see it flashing
#else
  #define runLED 2
#endif

Ticker tickerRunLED;                    // RunLED Structure

#define NUMRADIOINPUTS 2                // number of input ports
#define NUM_ANTENNA_PORTS 6             // number of antenna ports
#define ANTENNA_NAME_SIZE 16            // antenna name length
#if NUM_ANTENNA_PORTS > 7
  NOTE: Compile Error - currently APPStory library can not have more than 8 parameters...
#endif
// array for current state of device
bool antennaArray[NUMRADIOINPUTS][NUM_ANTENNA_PORTS];

#ifdef USE_APPSTORY
// Firmware Update status.
#define FW_IDLE            0
#define FW_REQUESTED       1
#define FW_DOWNLOADING     2
#define FW_DOWNLOAD_ERROR  3
#define FW_UPDATED         4
#define FW_EXITED          5

uint8_t fwUpdateStatus     = FW_IDLE;

bool doCheckUpdateRequest     = false;  // update firmware request flag
#endif
bool doThamConfigSaveRequest  = false;  // request flag to save Configurations
bool doConfigModeRequest      = false;  // request flag to enter config mode
bool doRelayCommandRequest    = false;  // request antenna relay shift out.
bool doThamAPRequest          = false;  // request flag for THAM AP mode
bool doThamSTARequest         = false;  // request flag for THAM STA mode
bool wifidnsStarted           = false;  // true if DNS is running

#ifndef USE_APPSTORY
bool handleWifiWaitFlag       = false;  // Wifi Manager function.
#endif

#ifdef USE_APPSTORY
char* ThamConfigWord = "00000000";         // A config word stored in EEPROM ( Not Used )
// NOTE: this array must allocate the space here.
char* antennaName[NUM_ANTENNA_PORTS] = {   // Antenna Names Array {
  "Antenna 1       " 
#if NUM_ANTENNA_PORTS > 1                  // method to preset structure dynamicly
  ,"Antenna 2       "
#endif
#if NUM_ANTENNA_PORTS > 2
  ,"Antenna 3       "
#endif
#if NUM_ANTENNA_PORTS > 3
  ,"Antenna 4       "
#endif
#if NUM_ANTENNA_PORTS > 4
  ,"Antenna 5       "
#endif
#if NUM_ANTENNA_PORTS > 5
  ,"Antenna 6       ",
#endif
#if NUM_ANTENNA_PORTS > 6
  ,"Antenna 7       "
#endif
#if NUM_ANTENNA_PORTS > 7
  ,"Antenna 8       "
#endif
  };
  
#else

#define WIFI_SSID_LEN 64
#define WIFI_PWD_LEN  32

        typedef struct {
                char wifiSSID[WIFI_SSID_LEN+1];                         // wifi SSID
                char wifiPassword[WIFI_PWD_LEN+1];                      // wifi password
                char thamConfigWord[8+1];                               // currently not used, future.
                char antennaName[NUM_ANTENNA_PORTS][ANTENNA_NAME_SIZE+1];
                const char checkBytes[3+1];
        } strThamConfig;
        
strThamConfig thamConfig = {
  "",
  "",
  "00000000",
  "Antenna 1",
#if NUM_ANTENNA_PORTS > 1  // method to preset structure dynamicly
  "Antenna 2",
#endif
#if NUM_ANTENNA_PORTS > 2
  "Antenna 3",
#endif
#if NUM_ANTENNA_PORTS > 3
  "Antenna 4",
#endif
#if NUM_ANTENNA_PORTS > 4
  "Antenna 5",
#endif
#if NUM_ANTENNA_PORTS > 5
  "Antenna 6",
#endif
#if NUM_ANTENNA_PORTS > 6
  "Antenna 7",
#endif
#if NUM_ANTENNA_PORTS > 7
  "Antenna 8",
#endif
  "^ok"
};
#endif //USE_APPSTORY

const char *softAP_ssid = "THAM_Network_2G";  // SSID if AP
//const char *softAP_password = "Tham9600";     // AP password.

// thamConfigWord definitions ( currently not used )
#define Undefindeflag0  0               // defines for the position of theconfig word
#define Undefindeflag1  1
#define Undefindeflag2  2
#define Undefindeflag3  3
#define Undefindeflag4  4
#define Undefindeflag5  5
#define Undefindeflag6  6
#define Undefindeflag7  7

IPAddress apIP(192, 168, 4, 1);         // IP address in AP

#ifndef DNS_PORT
  #define DNS_PORT 53                   // defined also by IOTAppStory.
#endif
DNSServer dnsServer;                    // DNS Structure

AsyncWebServer webServer(80);           // WebService structure

// ============================================= HTML text =============================================
// For THAM device
#define THAM_DEVICE_FUNCTION "THAM 2X6 Antenna Relay Controller"

// General form for unified look and feel.
const char HTTP_APP_HEAD[] PROGMEM         = "<!DOCTYPE html><html lang='en'><head><meta charset='UTF-8'>{mr}<meta name='viewport' content='width=device-width, initial-scale=1, user-scalable=no'/><title>{v}</title>";
const char HTTP_APP_STYLE[] PROGMEM        = "<style>.c{text-align: center;} div,input{padding:5px;font-size:1em;} input{width:95%;} body{text-align: center;font-family:verdana;} button{border:0;border-radius:0.3rem;background-color:#1fa3ec;color:#fff;line-height:2.4rem;font-size:1.2rem;width:100%;} .q{float: right;width: 70px;text-align: right;}</style>";
#ifndef USE_APPSTORY
const char HTTP_APP_SCRIPT[] PROGMEM       = "<script>function c(l){document.getElementById('s').value=l.innerText||l.textContent;document.getElementById('p').focus();}</script>";
#endif
const char HTTP_APP_HEAD_END[] PROGMEM     = "</head><body>{h}<div style='text-align:left;display:inline-block;min-width:290px;'>";

// ROOT FORM HTTP
const char HTTP_ROOTTEXT[] PROGMEM         = "<h2>Welcome To THam!</h2>"
                                             "<p>This is the " THAM_DEVICE_FUNCTION ".</p>"
                                             "<p>You are connected to SSID: {n}.</p>"
                                             "<p>The Web portal is {ip}.</P>";
const char HTTP_THAMNET[] PROGMEM          = "<p>My THam Network can be used.</p>"
                                             "<p>It is limited to 4 connections.</p>";
                                                                                           
const char HTTP_ROOT_WIFI_TXT[] PROGMEM    = "<p>To change a WIFI network, click \"Use THAM Network\" first.</p>";
const char HTTP_ROOT_THAM_TXT[] PROGMEM    = "<p>It is recommended to connect to an isolated station network router.</p>"
                                             "<p>To connect a WIFI network click \"Connect Network\".</p>";

const char HTTP_ROOT_FORM[] PROGMEM        = "<form action='showbuttons' method='POST'>"
                                             "<button type='submit'>Antenna Switch Display</button>"
                                             "<br><br><button type='submit' formaction='antennasetup'>Configure Antenna Names</button>";
#ifdef USE_APPSTORY
const char HTTP_ROOT_FWBUTTON[] PROGMEM    = "<br><br><button type='submit' formaction='checkupdate'>Check for Updates</button>";
#endif
                                             
// can have wifi button or tham button, not both.
const char HTTP_ROOT_WIFIBUTTON[] PROGMEM  = "<br><br><button type='submit' formaction='wifisetup'>Connect Network</button>";
const char HTTP_ROOT_THAMBUTTON[] PROGMEM  = "<br><br><button type='submit' formaction='thamnetwork'>Use THAM Network</button>";

//const char HTTP_NEWPORTAL[] PROGMEM        = "<p>Please reconnect on the web portal</P>";

#ifndef USE_APPSTORY
// WIFI Config HTTP
const char HTTP_ITEM_HEAD[] PROGMEM        = "<div>SSID&nbsp;<span class='q'>Strength</span></div>";
const char HTTP_ITEM[] PROGMEM             = "<div><a href='#p' onclick='c(this)'>{v}</a>&nbsp;<span class='q'>{i}{r}%</span></div>";

const char HTTP_WIFI_FORM_START[] PROGMEM  = "<form action='wifisave' method='POST'><input id='s' name='s' length=63 placeholder='SSID'><br/><input id='p' name='p' length=31 type='password' placeholder='password'><br/>";
const char HTTP_WIFI_BUTTONS[] PROGMEM     = "<br><br><button type='submit'>Save</button>"
                                             "<br><br><button type='submit' formaction='wifisetup'>Scan again</button>"
//                                           "<br><br><button type='submit' formaction='thamnetwork'>Use THAM Network</button>"
                                             "<br><br><button type='submit' formaction='/'>Back to Home</button>";
#endif

// Antenna Name Form
const char HTTP_NAME_FORM[] PROGMEM        = "<form action='antennasave' method='POST'>";
const char HTTP_NAME_ITEM[] PROGMEM        = "<br/><input id='{id}' name='{id}' length={l} {p}>";
const char HTTP_NAME_BUTTONS[] PROGMEM     = "<br><br><button type='submit'>Save</button>"
                                             "<br><br><button type='submit' formaction='/'>Back to Home</button>";

// global HTTP endings             
const char HTTP_FORM_END[] PROGMEM          = "</form>";
const char HTTP_END[] PROGMEM               = "</div></body></html>";

// HTTP messages
const char HTTP_REFRESH_TO_PAGE[] PROGMEM   = "<meta http-equiv='refresh' content='{t};URL={u}'>";
const char HTTP_ANTNAMESAVED_MSG[] PROGMEM  = "<h2>Antenna names updated.</h2>";
const char HTTP_WIFISAVED_MSG[] PROGMEM     = "<h2>WiFi credentials updated.</h2><h2>Connecting...</h2>";
const char HTTP_WIFIFAILED_MSG[] PROGMEM    = "<h2>WiFi connect failed.</h2><h2>Please try again.</h2>";
const char HTTP_WIFINOENTRY_MSG[] PROGMEM   = "<h2>Please select SSID</h2>";
const char HTTP_NOTONWIFI_MSG[] PROGMEM     = "<h2>Connect to WiFi</h2><h2>to update firmware.</h2>";
const char HTTP_WIFISETUP_MSG[] PROGMEM     = "<h2>WiFi scan requested.</h2>";
const char HTTP_CONFIG_MSG[] PROGMEM        = "<h2>Configuration mode requested.</h2>";
const char HTTP_WIFIDONE_MSG[] PROGMEM      = "<h2>Reconnect to {e}.</h2><h2> Enter {ea} in browser.</h2>";
const char HTTP_THAMNET_MSG[] PROGMEM       = "<h2>THAM network requested.</h2><h2>Reconnect to {g}.</h2><h2> Enter {ga} in browser.</h2>";

#ifdef USE_APPSTORY
const char HTTP_FWREQUEST_MSG[] PROGMEM     = "<h2>Firmware check requested.</h2><h2>Antenna Relays off.</h2>";
const char HTTP_FWDOWNLOAD_MSG[] PROGMEM    = "<h2>Downloading new firmware.</h2>";
const char HTTP_FWERROR_MSG[] PROGMEM       = "<h2>Download Error</h2><h2>Try again later.</h2>";
const char HTTP_FWUPDATED_MSG[] PROGMEM     = "<h2>Firmware updated.</h2><h2>Rebooting...</h2>";
const char HTTP_FWEXITED_MSG[] PROGMEM      = "<h2>Firmware check completed.</h2>";
#endif

// HTTP Antenna buttons form
const char HTTP_ANTENNA_PAGE[] PROGMEM      = "<html><head>"
                                           // "<meta name='viewport' content='width=device-width, initial-scale=1'/>"
           "<style>"
             "body{background:#000;}"
             ".bt{display:block;width:90px;height:90px;padding:10px;margin:10px;text-align:center;border-radius:5px;color:white;font-weight:bold;font-size:70px;text-decoration:none;}"
             ".bx{display:block;width:450px;height:90px;padding:10px;margin:10px;text-align:center;border-radius:5px;color:white;font-weight:bold;font-size:60px;text-decoration:none;}"
             ".r{background:#933;}"
             ".g{background:#363;}"
             ".y{background:#EE0;height:90px;width:90px;border-radius:45px;}"
             ".b{background:#000;height:90px;width:90px;border-radius:45px;}"
             ".a{font-size:30px;} td{vertical-align:middle;}"
          "</style>"
          "</head><body><table><tbody >";
          
const char HTTP_ANTENNA_START[] PROGMEM     = "<tr>";
const char HTTP_ANTENNA_LED[] PROGMEM       = "<td><div class='{a}'></div></td>";
const char HTTP_ANTENNA_BUTTON[] PROGMEM    = "<td><a class='{c}' href='/showbuttons?v={v1}&a={v2}'>{m}</a></td>";
const char HTTP_ANTENNA_HOME[] PROGMEM      = "<td><a class='bt g a' href='/'>Home</a></td>";
const char HTTP_ANTENNA_TEXT[] PROGMEM      = "<td><div class='bx'>{n}</div></td>";
const char HTTP_ANTENNA_END[] PROGMEM       = "</tr>";
const char HTTP_ANTENNA_PAGEEND[] PROGMEM   = "</tbody></table></body></html>";

// =============================================Start of Code=============================================
// EEPROM management
#ifndef USE_APPSTORY
        
#ifdef ESP8266
// =======================================================================================================
// sometimes need to print out binary data
#ifdef DEBUGLEVEL >=3
String charDisplay(char value) {
  if (value>=32 && value<=126){
    return String(value);
  } else if (value == 0){
    return ".";
  } else {
    return String("[" + String(value, DEC) + "]");
  }
}
#endif
#if DEBUGLEVEL >= 1
// =======================================================================================================
// Erase the config area,  maybe config only, maybe everything known.
void eraseThamConfig(int eeStart, int eeEnd) {
  int i = 0;
  
  DEBUGPRINTF1("in eraseThamConfig:%d %d\n", eeStart, eeEnd);
  EEPROM.begin(EEPROM_SIZE);
  for (i = eeStart; i < eeEnd; i++) EEPROM.write(i, 0);
  EEPROM.commit();
  EEPROM.end();
}
#endif
// =======================================================================================================

// write the config buffer.
void writeThamConfig() {
  int i = 0;

  DEBUGPRINTLN1(F("in writeThamConfig"));
  EEPROM.begin(EEPROM_SIZE);
  for (int i = 0; i < sizeof(strThamConfig); i++) {
    EEPROM.write(i, *((char*)&thamConfig + i));
    DEBUGPRINT3(charDisplay(*((char*)&thamConfig + i)));
  }
  EEPROM.commit();
  EEPROM.end();
  DEBUGPRINTLN3();
}

// =======================================================================================================
// readCofig Buffer
void readThamConfig() {
  int i = 0;
  int checkStart = sizeof(strThamConfig) - 4;
  char readChar = 0;
  
  DEBUGPRINTLN1(F("in readThamConfig"));
  EEPROM.begin(EEPROM_SIZE);
  // Check the checkBytes
  if(EEPROM.read(checkStart) == thamConfig.checkBytes[0] && 
     EEPROM.read(checkStart + 1) == thamConfig.checkBytes[1] && 
     EEPROM.read(checkStart + 2) == thamConfig.checkBytes[2]) {
    DEBUGPRINTLN1(F("EEPROM Good!"));
    for(i = 0; i < sizeof(strThamConfig) - 4; i++){
      readChar = EEPROM.read(i);
       *((char*)&thamConfig + i) = readChar;
       DEBUGPRINT3(charDisplay(readChar));
    }                        
    DEBUGPRINTLN3();
    EEPROM.end();
  }else{
    EEPROM.end();  // close now as we will re-open.
    DEBUGPRINTLN1(F("EEPROM bad..."));
    writeThamConfig();
  }
}
#else  // ESP8266
// =======================================================================================================
// ESP32 uses Preferences */
void readThamConfig() {
  DEBUGPRINTLN1(F("in readThamConfig Not done..."));
  //preferences.getString("strConfig", thamConfig, sizeof(strConfig));
}

// =======================================================================================================
/** Store WLAN credentials to Preference */
void writeThamConfig() {
  DEBUGPRINTLN1(F("in writeThamConfig Not done..."))
  preferences.putString("wifiSSID", wifiSSID);  // not correct yet...
}
#endif  // ESP8266

#endif  // USE_APPSTORY

#ifdef UCN5812_DRIVE
// ========================================== UCN5812AF Library ==========================================
// 
// NOTE:
// the UCN5812AF is an obsolete through hole chip used for relay source drive.
// I have a supply of these chips and some boards to mount them on.  
// I decided to use these up in the THAM project on the 2X6 antenna controllers.
// the new version of this program will use I2C Seral latch chips to optoisolators and NPN/FET Relay drives.
// Comment out the UCN5812_DRIVE line below to remove all the code
//
// The UCN5812AF is driven by ESP8266 GPIO Ports, their definitions follow.
//
// GPIO12 output Serial line data
// GPIO13 output Serial Line Clock
// GPIO14 output Serial line Strobe
//
#define SPI_SIO_PIN 12
#define SPI_CLK_PIN 13
#define SPI_STR_PIN 14 

#define SPI_NUMBITS 20    // using a single UCN5812AF for this. it has 20 bits

// 
// the shift register used has 20 bits, 16 used for side A & B
// the remaining 4 bits are defined as follows
// none yet  define TOPBITS to use routines
//
#ifdef UCN5812_TOPBITS
uint8_t antennaTop4Bits;
#endif

// ==========================================================================================
void setupUCN5812() {
   pinMode(SPI_SIO_PIN, OUTPUT);
   pinMode(SPI_CLK_PIN, OUTPUT);
   pinMode(SPI_STR_PIN, OUTPUT);
   digitalWrite(SPI_SIO_PIN, LOW);
   digitalWrite(SPI_CLK_PIN, LOW);
   digitalWrite(SPI_STR_PIN, LOW);
}

/******************************************************************************
** doRelayCommand - write data to serial clocked bus for older serail
**                   chips before the SPI erra.
** RDC_Library
**
** example  doRelayCommand(shiftFlag);  
** PORT must be POINTER to GPIO address
** SPI1_SIN is the Serial in/out pin,  It also defines that the
**          clock and STROBE muste be deifined .
** numBits is the number of bits to shift out
** data is a 32 bit data element to send ( thus maximum is 32 )
******************************************************************************/
// ==========================================================================================
void doRelayCommand(bool loopFlag) {
  uint8_t radioNum = 0;
  uint8_t portNum = 0;
  uint8_t i_int;
  uint8_t antennaRelayBitSide[NUMRADIOINPUTS];
  uint32_t antennaSPIData = 0 ;  // (32 bit)

  // interrupt handle routines should call with false,  
  // flag to set relays on next loop run.
  
  if ( !loopFlag ) {
    doRelayCommandRequest = true;
    return;
  }
  if ( !doRelayCommandRequest ) {
    return;
  }
  doRelayCommandRequest = false;
  
#ifdef UCN5812_TOPBITS
   antennaSPIData = antennaTop4Bits;
#else
   antennaSPIData = 0;
#endif

  // Set antenna relay bits for shift out.
  for ( radioNum=0; radioNum < NUMRADIOINPUTS; radioNum++ ) {
    antennaRelayBitSide[radioNum] = 0;
  }

  for ( radioNum=0; radioNum < NUMRADIOINPUTS; radioNum++ ) {
    for ( portNum=0; portNum < NUM_ANTENNA_PORTS; portNum++ ) {
      if (antennaArray[radioNum][portNum]) {
        antennaRelayBitSide[radioNum] = 0x01 << portNum ;  // set relay bit.
      }
    }
  }
   // set in bit array as it will be in the serail latch
   //  handle top 4 bits, then add next 8 bits for radio 1 then next 8 bits for rdio 0
   antennaSPIData = (antennaSPIData << 16) | (antennaRelayBitSide[1] << 8) | antennaRelayBitSide[0];

   DEBUGPRINTF1("in doRelayCommand, antennaSPIData:%x\n",antennaSPIData);
 
   for ( i_int = SPI_NUMBITS;  i_int > 0; i_int-- ) { /* shift bits in reverse */
                                                       
      digitalWrite(SPI_SIO_PIN,(0x01 & (antennaSPIData >> i_int-1 )));          // set data if set
      DEBUGPRINTF3("SPI_SIO_PIN %x\n",(0x01 & (antennaSPIData >> i_int-1 )));  
      digitalWrite(SPI_CLK_PIN,HIGH);            // set the clock
      
      digitalWrite(SPI_CLK_PIN,LOW);             // clear the clock
   }
   digitalWrite(SPI_STR_PIN,HIGH);               // set latch 
   digitalWrite(SPI_STR_PIN,LOW);                // clear latch
}

#endif // UCN5812_DRIVE

// Antenna Relay control library
// ========================================= Antenna port code ===========================================
void antennaRadioClear(int radioNum) {  // Clear all antenna ports for 1 radio
  for (int j=0;j<NUM_ANTENNA_PORTS; j++) {
    antennaArray[radioNum][j] = false;
  } 
}

// ==========================================================================================
void antennaArrayClear() {    // Clear all antenna ports for all radios
  for (int i=0;i<NUMRADIOINPUTS; i++) {
    antennaRadioClear(i);
  }
}
  
// ==========================================================================================
bool antennaPortUsed(int portNum) {  // return true if the antenna port is in use
  for (int i=0;i<NUMRADIOINPUTS; i++) {
    if (antennaArray[i][portNum] ) return true;
  }
  return false;
}

// ==========================================================================================
bool antennaSetPort(int radioNum, int portNum) {  // Set the port 
  if ( radioNum < 0 || radioNum >= NUMRADIOINPUTS || portNum < 0 || portNum > NUM_ANTENNA_PORTS ) {
    return false;
  }
  if ( !antennaArray[radioNum][portNum]) {  // check already set.
    if (antennaPortUsed(portNum)) {         // check for in use
      return false;
    }
  }
  antennaRadioClear(radioNum);  // clear any previous ports used for this radio
  antennaArray[radioNum][portNum] = true;  // set port
  
  return true;
}

// some debug functions
// ========================================= Some Debugging code ===========================================
#if DEBUGLEVEL >= 3
void showWifiStatus() {
  int r_stt;
  
  r_stt=WiFi.status();
  switch (r_stt) {
    case WL_CONNECTED : Serial.print("status is WL_CONNECTED: "); break;
    case WL_NO_SHIELD : Serial.print("status is WL_NO_SHIELD: "); break;
    case WL_IDLE_STATUS : Serial.print("status is WL_IDLE_STATUS: "); break;
    case WL_NO_SSID_AVAIL : Serial.print("status is WL_NO_SSID_AVAIL: "); break;
    case WL_SCAN_COMPLETED : Serial.print("status is WL_SCAN_COMPLETED: "); break;
    case WL_CONNECT_FAILED : Serial.print("status is WL_CONNECT_FAILED: "); break;
    case WL_CONNECTION_LOST : Serial.print("status is WL_CONNECTION_LOST: "); break;
    case WL_DISCONNECTED : Serial.print("status is WL_DISCONNECTED: "); break;
    default: Serial.print("status is UNKNOWN: ");
  }
  Serial.printf("%d\n",r_stt);
}
#endif

// ==========================================================================================
#if DEBUGLEVEL >= 3
void showWifiMode() {
  int r_stt;
  
  r_stt=WiFi.getMode();
  switch (r_stt) {
    case WIFI_AP : Serial.print("mode is WIFI_AP: "); break;
    case WIFI_STA : Serial.print("mode is WIFI_STA: "); break;
    case WIFI_AP_STA : Serial.print("mode is WIFI_AP_STA: "); break;
    default: Serial.print("mode is UNKNOWN: ");
  }
  Serial.printf("%d\n",r_stt);
}
#endif

// ========================================= THAM AP Functions ===========================================
// a function to stop all wifi services
void stopWifiService() {
  
    DEBUGPRINTLN1("in stopWifiService");

    webServer.reset();           // stop the web server
    dnsServer.stop();            // stop dns server
    WiFi.disconnect(true);       // turn off wifi STA
    WiFi.softAPdisconnect(true); // turn off wifi AP
    WiFi.mode(WIFI_OFF); // set mode OFF
    while (WiFi.status() != WL_DISCONNECTED) {  // wait for disconnect
      delay(10);
    }
}

// ==========================================================================================
void beginThamAP(){
  int r_stt = 0;   
  String strDeviceName(softAP_ssid);

  DEBUGPRINTF1("in beginThamAP %s\n",strDeviceName.c_str());
    
    stopWifiService();           // close all servers and wifi.  Clean up everything...
#ifdef USE_APPSTORY
    IAS.preSetWifi("","");       // clear the wifi address in case of reboot.
                                 // must go back to config mode to regain credentials.
                                 // only do this if THAM network specificly requested
                                     
    IAS.preSetDeviceName(strDeviceName);
    IAS.writeConfig(false);      // save it.

#endif

    DEBUGPRINT1("Wi-Fi mode set to WIFI_AP ...");
    r_stt = WiFi.mode(WIFI_AP);          // set AP mode
    DEBUGPRINTF1("%s\n", r_stt ? "Success" : "Failed!");
    WiFi.setAutoConnect(false);  // reset autoconnect

    DEBUGPRINT1("Setting soft-AP configuration ...");
    r_stt = WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
    DEBUGPRINTF1("%s\n", r_stt ? "Success" : "Failed!");
    
    // WiFi.softAP(softAP_ssid,softAP_password);  // unfortunately have to match the IAS library here...
    DEBUGPRINT1("Setting soft-AP ...");
    r_stt = WiFi.softAP(softAP_ssid);
    DEBUGPRINTF1("%s\n", r_stt ? "Success" : "Failed!");
        
    // start up a DNS, just point everything to us....
    dnsServer.setTTL(300);
    dnsServer.start(DNS_PORT, "*", apIP);
    wifidnsStarted = true;       // wifi DNS is now running...
}

// ====================================== functions =======================================

// IOTAppStory has a configuration structure, 
// if USE_APPSTORY, then we use the addField function to create space 
// in the library configuation structure.
// otherwise we maintain our own structure.
//

// ==========================================================================================
void doThamConfigSave(bool loopFlag) {
  if ( !loopFlag ) {
    doThamConfigSaveRequest = true;
    return;
  }
  if ( !doThamConfigSaveRequest ) {
    return;
  }
  doThamConfigSaveRequest = false;
  DEBUGPRINTLN1("in doThamConfigSave");
#ifdef USE_APPSTORY
  IAS.writeConfig(true);         // call the IOTAppStory save config routine with the flag true to save added fields.
#else
  writeThamConfig();
#endif
}
#ifdef USE_APPSTORY
// =================================== Check Firmware Update Functions ========================================
void handleIASUpdateStarted() {     // Called when the app starts to download updates.
  fwUpdateStatus = FW_DOWNLOADING ;
  DEBUGPRINTLN1("in handleIASUpdateStarted");
}
// ============================================================================================================
void handleIASUpdateError() {          // Called when downloading firmware updates end in error.
  fwUpdateStatus = FW_DOWNLOAD_ERROR ;
  DEBUGPRINTLN1("in handleIASUpdateError");
}
// ============================================================================================================
void handleIASUpdateDone() {         // Called when the downloaded firmware is successfully installed.
  fwUpdateStatus = FW_UPDATED ;
  DEBUGPRINTLN1("in handleIASUpdateDone");
}
// ============================================================================================================
#endif
#ifdef USE_APPSTORY
// check for firmware updates
void doCheckUpdate(bool loopFlag) {

  if ( !loopFlag ) {
    doCheckUpdateRequest = true;
    return;
  }
  if ( !doCheckUpdateRequest ) {
    return;
  }
  doCheckUpdateRequest = false;
  DEBUGPRINTF1("in doCheckUpdate: Update Status:%d\n",fwUpdateStatus);

  if (WiFi.isConnected() ) {
    fwUpdateStatus = FW_REQUESTED ;
    IAS.callHome();  // Check for updates ( no spiffs yet )
    if ( fwUpdateStatus == FW_REQUESTED ) fwUpdateStatus = FW_EXITED ;
  }
}
#endif  // USE_APPSTORY

// ================================== do Config Mode Change =====================================
void doConfigMode(bool loopFlag) {
  int r_stt = 0;
  
  if ( !loopFlag ) {
    doConfigModeRequest = true;
    return;
  }
  if ( !doConfigModeRequest ) {
    return;
  }
  doConfigModeRequest = false;
  DEBUGPRINTLN1("in doConfigMode");

#ifdef USE_APPSTORY

  delay(500);                // let any web page get sent.

  stopWifiService();          // stop all wifi services

  IAS.preSetAutoConfig(true); // Do config mode if Wifi Not found.

  IAS.espRestart('C');        // enter Configuration mode

#else

  if (( WiFi.getMode() & WIFI_AP ) &&
      ( strlen(thamConfig.wifiSSID) > 0 )) { // check if we have AP mode. & SSID
      DEBUGPRINT1("Wi-Fi mode set to WIFI_AP_STA ...");
      r_stt = WiFi.mode(WIFI_AP_STA);                    // reset mode to AP/STA
      DEBUGPRINTF1("Mode Set:%s\n", r_stt ? "Success" : "Failed!");

      WiFi.begin( thamConfig.wifiSSID, thamConfig.wifiPassword );  // now try with saved credentials
      DEBUGPRINTLN1("Tying saved credentials, connecting... ");
      r_stt = WiFi.waitForConnectResult();
      DEBUGPRINTF1( "Connect Result: %d\n:",r_stt );
  } else { 
    handleWifiWaitFlag = false;
  }
  

#endif // USE_APPSTORY
}

// ================================== Event and HTTP handle routines =====================================
// NOTE: keep all handle routines short,  blocking and yeild Not allowed, causes hangs and crashes.

// ==========================================================================================
void handleRunLED()
{
#ifdef USE_APPSTORY
  digitalWrite(IAS.dPinConv(runLED), !digitalRead(IAS.dPinConv(runLED)));   // toggle state
#else
  digitalWrite(runLED, !digitalRead(runLED));     // set pin to the opposite state
#endif
  DEBUGPRINTF3("Stations connected to soft-AP = %d\n", WiFi.softAPgetStationNum());
  DEBUGPRINTF3("Current Heap: %d\n",ESP.getFreeHeap());
}

// ==========================================================================================
// the hope is to use the IOTAppStory Wifi Manager,
// code is here in case we need it.
#ifndef USE_APPSTORY
// Convert rssi to percentage.
int getRSSIasQuality(int RSSI) {
  int quality = 0;

  if (RSSI <= -100) {
    quality = 0;
  } else if (RSSI >= -50) {
    quality = 100;
  } else {
    quality = 2 * (RSSI + 100);
  }
  return quality;
}
#endif
// ==========================================================================================
// send THAM Standard HTTP response function
void sendHTTP(AsyncWebServerRequest *request, String &toSend, String textType="text/html") {
  
  AsyncWebServerResponse *response = request->beginResponse(200, textType, toSend);
  response->addHeader(F("Cache-Control"), F("no-cache, no-store, must-revalidate"));
  response->addHeader(F("Pragma"), F("no-cache"));
  response->addHeader(F("Expires"), F("-1"));
  response->addHeader(F("Server"), F("THAM Web Server"));
  response->addHeader(F("Connection"), F("close"));
  response->addHeader(F("Content-Length"), String(toSend.length()));
  request->send(response);
}
// ==========================================================================================
// THAM Message function
void sendHTTPMessage(AsyncWebServerRequest *request, char* returnLink, char* refreshTime, const char* HTTP_Msg ) {
  int varIndex = 0;
  String toSend = "";

  DEBUGPRINTF1("in sendHTTPMessage, link:'%s'\n",returnLink);
  toSend += FPSTR(HTTP_APP_HEAD);
  toSend.replace("{v}", "THAM Wifi Config Request"); 
  toSend.replace("{mr}",FPSTR(HTTP_REFRESH_TO_PAGE));
  toSend.replace("{t}", refreshTime);
  toSend.replace("{u}", returnLink);
  
  toSend += FPSTR(HTTP_APP_STYLE);
  toSend += FPSTR(HTTP_APP_HEAD_END);
  toSend.replace("{h}", FPSTR(HTTP_Msg));
  
  varIndex = toSend.indexOf("{e}");  // some messages have a variable to fill in
  DEBUGPRINTF3("varIndex e:%d\n",varIndex);
  if ( varIndex > -1) {
    if ( WiFi.isConnected() ) {
      toSend.replace("{e}", WiFi.SSID() );
      toSend.replace("{ea}", WiFi.localIP().toString() );
    } else {
      DEBUGPRINTF1("NotConnected:\n");
      toSend.replace("{e}", softAP_ssid );
      toSend.replace("{ea}", apIP.toString() );
    }
  }
  varIndex = toSend.indexOf("{g}");
  DEBUGPRINTF3("varIndex g:%d\n",varIndex);
  if ( varIndex > -1) {
      toSend.replace("{g}", softAP_ssid );
      toSend.replace("{ga}", apIP.toString() );
  }  
  toSend += FPSTR(HTTP_END);
 
  sendHTTP(request, toSend);
}

// ==========================================================================================
// Every webserver needs a not found page.  Give us some diagnostids.
void handleNotFound(AsyncWebServerRequest *request){
  int headers = 0;
  int i = 0;
  int params = 0;
 
  String toSend = "File Not Found\n\n";
   
  DEBUGPRINTLN1("Page served handleNotFound");
  DEBUGPRINTF3("NOT_FOUND: ");
  switch ( request->method() ) {
      case HTTP_GET:     toSend += "GET"; 
                         break; 
      case HTTP_POST:    toSend += "POST"; 
                         break;
      case HTTP_DELETE:  toSend += "DELETE";
                         break;
      case HTTP_PUT:     toSend += "PUT";
                         break;
      case HTTP_PATCH:   toSend += "PATCH";
                         break;
      case HTTP_HEAD:    toSend += "HEAD";
                         break;
      case HTTP_OPTIONS: toSend += "OPTIONS";
                         break;
      default:           toSend += "UNKNOWN";
  }
  
  toSend += " http://" + request->host() + request->url() + "\n";
  
  if(request->contentLength()){
    toSend += "_CONTENT_TYPE: " + request->contentType() + "\n";
    toSend += "_CONTENT_LENGTH: " + String(request->contentLength()) + "\n";
  }

  headers = request->headers();
  for(i=0; i<headers; i++){
    AsyncWebHeader* h = request->getHeader(i);
    toSend += "_HEADER[" + h->name() + "]: " + h->value() + "\n";
  }

  params = request->params();
  for(i=0;i<params;i++){
    AsyncWebParameter* p = request->getParam(i);
    if(p->isFile()){
      toSend += "_FILE[" + p->name() + "]: " + p->value() + " size: " + String(p->size()) + "\n" ;
    } else if(p->isPost()){
      toSend += "_POST[" + p->name() + "]: " + p->value();
    } else {
      toSend += "_GET[" + p->name() + "]: " + p->value();
    }
  }
  request->send(404, "text/html", toSend);
  DEBUGPRINTLN3(toSend);
}

// ==========================================================================================
bool checkInProgress(AsyncWebServerRequest *request) {
  // the app has several modes it can be in, This check routine will handle any
  // unexpected page requests
  
  DEBUGPRINTLN1("in checkInProgress");
  
#ifndef USE_APPSTORY
  if (handleWifiWaitFlag) {  // send back to wifi wait.
    handleWifiWait(request);
    return(true) ; // don't do any other checks.
  }
#endif
#ifdef USE_APPSTORY
  if (fwUpdateStatus != FW_IDLE ) {
    handleCheckUpdate(request);
    return(true) ; // don't do any other checks.
  }    
#endif
  return(false);
}
// ==========================================================================================
#ifdef USE_APPSTORY
// Handle the firmware check request,  this is recalled by most firmware check messages.

void handleCheckUpdate(AsyncWebServerRequest *request) {
  String toSend = "";

  // refresh to here every 10 seconds until completed...

  DEBUGPRINTF1("Page served handleCheckUpdate, Update Status:%d\n",fwUpdateStatus); 

  if ( request->method() == HTTP_POST ) {  // first time entry?
    if (WiFi.isConnected()) {    // must be on WiFi
      antennaArrayClear();       // clear relay bits 
      doRelayCommand(false);     // request loop to send commands
      doCheckUpdate(false);      // request firmware update check
      fwUpdateStatus = FW_REQUESTED ;       // set status.
      sendHTTPMessage(request,"/checkupdate","10", HTTP_FWREQUEST_MSG);
    } else {
      sendHTTPMessage(request,"/","10", HTTP_NOTONWIFI_MSG);
    }
  } else {  // HTTP GET indicates we were called back by refresh...
    // we could see IDLE if rebooted or EXITED no updates done
    if ( fwUpdateStatus == FW_IDLE ) fwUpdateStatus = FW_EXITED ; // so just make it show exited... 
    switch (fwUpdateStatus) {
      case FW_REQUESTED :      sendHTTPMessage(request,"/checkupdate","10", HTTP_FWREQUEST_MSG);
                               break;
      case FW_DOWNLOADING :    sendHTTPMessage(request,"/checkupdate","10", HTTP_FWDOWNLOAD_MSG);
                               break;
      case FW_DOWNLOAD_ERROR : sendHTTPMessage(request,"/","10", HTTP_FWERROR_MSG);
                               fwUpdateStatus = FW_IDLE ;
                               break;
      case FW_UPDATED :        sendHTTPMessage(request,"/","10", HTTP_FWUPDATED_MSG);
                               fwUpdateStatus = FW_IDLE ;
                               break;
      case FW_EXITED :         sendHTTPMessage(request,"/","10", HTTP_FWEXITED_MSG);
                               fwUpdateStatus = FW_IDLE ;
                               break;
    }
  } 
}
#endif

// ==========================================================================================
void handleWifiSetup(AsyncWebServerRequest *request) {
  int quality = 0;
  int i;
  String scanItem = "";
  String rssiQ = "";
  String toSend = "";  
  
  DEBUGPRINTLN1("Page served handleWifiSetup");
  // if we get here and we in STA mode,  go back to root
  if (WiFi.getMode() == WIFI_STA) {       // isConnected may be misleading...
    handleRoot(request);
    return;
  }
 
  antennaArrayClear();                    // clear relay bits 
  doRelayCommand(false);                  // request loop to send commands

#ifdef USE_APPSTORY
  doConfigMode(false);                      // use IOTAppStory wifiManager request Config Mode.
  sendHTTPMessage(request,"/","30", HTTP_CONFIG_MSG); // cause refresh in 10 second. to root
#endif
#ifndef USE_APPSTORY
  // we are using Async Web server, so we can not block,
  // scan networks allows this with a 'true' argument.
 
  int n = WiFi.scanComplete();
  DEBUGPRINTF1("scanComplete: %d\n",n);
  if (n < 0) {                        // Not complete or not started...
    if ( n < -1 ) {                   // if not in progress, then start the scan
      WiFi.scanNetworks(true);        // start ASYNC scan ( automaticly switch to WIFI_AP_STA mode )
    }
    // send a message page...
    sendHTTPMessage(request,"/wifisetup","1", HTTP_WIFISETUP_MSG);  // cause refresh in 1 second.
  } else {

    // now send the wifi Config form.
    toSend += FPSTR(HTTP_APP_HEAD);
    toSend.replace("{v}", "Config THAM device");
    toSend.replace("{mr}","");
    toSend += FPSTR(HTTP_APP_SCRIPT);
    toSend += FPSTR(HTTP_APP_STYLE);
    toSend += FPSTR(HTTP_APP_HEAD_END);
    toSend.replace("{h}", "<p>THAM Wifi Configuation</p>");
    toSend += FPSTR(HTTP_ITEM_HEAD);
    
    if (n > 0) {   
      for (i = 0; i < n; i++) {
        scanItem = FPSTR(HTTP_ITEM);
        rssiQ="";
        quality = getRSSIasQuality(WiFi.RSSI(i));
        if (quality < 100 ) {
          rssiQ += " ";
        }
        rssiQ += quality;
        scanItem.replace("{v}", WiFi.SSID(i));
        scanItem.replace("{r}", rssiQ);
        if (WiFi.encryptionType(i) != ENC_TYPE_NONE) {
          scanItem.replace("{i}", "🔒");
        } else {
          scanItem.replace("{i}", "");
        }
        toSend += scanItem;
      }
    }
    toSend += "<br/>";
    toSend += FPSTR(HTTP_WIFI_FORM_START);
    toSend += FPSTR(HTTP_WIFI_BUTTONS);
    toSend += FPSTR(HTTP_FORM_END);
    //toSend += FPSTR(HTTP_SCAN_LINK);

    toSend += FPSTR(HTTP_END); 
    sendHTTP(request, toSend);
    WiFi.scanDelete(); // clear the saved buffer...
  }
#endif
}


// ==========================================================================================
#ifndef USE_APPSTORY

// Handle the WLAN wait and redirect to new WLAN home page again
void handleWifiWait(AsyncWebServerRequest *request) {
  int r_stt = 0;


  r_stt = WiFi.status();  // get current state of WiFi
  DEBUGPRINTF1("in handleWifiWait, status:%d\n",r_stt);
  switch (r_stt) {
    // success
    case WL_CONNECTED :      sendHTTPMessage(request,"/","600", HTTP_WIFIDONE_MSG); // done, tell user..
                             doThamSTA(false);   // request transition to STA mode
                             handleWifiWaitFlag = false;
                             DEBUGPRINTLN1("... Connected");
                             break;
    // working on it.
    case WL_IDLE_STATUS :    sendHTTPMessage(request,"/wifiwait","5", HTTP_WIFISAVED_MSG); // cause refresh in 5 second.
                             handleWifiWaitFlag = true;
                             DEBUGPRINTLN1("... Connecting");
                             break;
                             
    default :              sendHTTPMessage(request,"/","10", HTTP_WIFIFAILED_MSG); // cause refresh in 10 second.
                             WiFi.scanNetworks(true);        // start ASYNC scan 
                             handleWifiWaitFlag = false;
                             DEBUGPRINTLN1("... Failed");
  }
}
#endif // USE_APPSTORY
// ==========================================================================================
#ifndef USE_APPSTORY
// Handle the WLAN save and redirect to WLAN wait message
void handleWifiSave(AsyncWebServerRequest *request) {
  char parmSSID[64] = "";
  char parmPassword[32] = "";

  DEBUGPRINTLN1("Page served handleWifiSave");

  if (request->hasArg("s")) {  // have a valid parameter?
    request->arg("s").toCharArray(parmSSID, sizeof(parmSSID) - 1);
  
    if (request->hasArg("p")) request->arg("p").toCharArray(parmPassword, sizeof(parmPassword) - 1);
    DEBUGPRINTF1("Have SSID:'%s' PWD:'%s'\n",parmSSID,parmPassword);
    if ( strlen(parmSSID) > 0 ) {
      strcpy(thamConfig.wifiSSID,parmSSID);
      strcpy(thamConfig.wifiPassword,parmPassword);
      doThamConfigSave(false);
      doConfigMode(false);                      // request Config Mode after results.
      sendHTTPMessage(request,"/wifiwait","5", HTTP_WIFISAVED_MSG); // cause refresh to wifiwait
      handleWifiWaitFlag = true;                 // starting the WiFi wait function
    } else {
      sendHTTPMessage(request,"/wifisetup","5", HTTP_WIFINOENTRY_MSG); // cause refresh to root
      WiFi.scanNetworks(true);        // start ASYNC scan
    }
  } else {
    // no valid parameter,  must have hand entered address....
    checkInProgress(request);
  }
}
#endif  // USE_APPSTORY

// below code is from original code
// ==========================================================================================
// Handle the Antenna Name save form
void handleAntennaSave(AsyncWebServerRequest *request) {
  int i = 0;
  char pageArgName[3] = "";
  String toSend = "";
  
  DEBUGPRINTLN1("Page served handleAntennaSave");
  if (checkInProgress(request)) return;
  
  for ( i=0; i<NUM_ANTENNA_PORTS; i++ ) {
    sprintf(pageArgName,"a%d",i);
    if (request->hasArg(pageArgName))
#ifdef USE_APPSTORY
      request->arg(pageArgName).toCharArray(antennaName[i], ANTENNA_NAME_SIZE);
      DEBUGPRINTF1("Antenna save arg:'%s' val:'%s' \n",pageArgName, antennaName[i] );
      if ( antennaName[i][0] == 0) sprintf(antennaName[i], "%s %d","Antenna",i+1);
#else
      request->arg(pageArgName).toCharArray(thamConfig.antennaName[i], ANTENNA_NAME_SIZE);
      if ( thamConfig.antennaName[i][0] == 0) sprintf(thamConfig.antennaName[i], "%s %d","Antenna",i+1);
#endif
        
  }
  doThamConfigSave(false); // request save names
  
  // send names saved
  sendHTTPMessage(request,"/","5", HTTP_ANTNAMESAVED_MSG);  // cause refresh in 5 second.
}

// ==========================================================================================
void handleAntennaSetup(AsyncWebServerRequest *request) {
  int i = 0;
  char pageArgName[3] = "";
  char pageArgLength[3] = "";
  char antdefault[100];
  String itemBuf = "";
  String toSend = "";  
  
  DEBUGPRINTLN1("Page served handleAntennaSetup");
  if (checkInProgress(request)) return;

  toSend += FPSTR(HTTP_APP_HEAD);
  toSend.replace("{v}", "Config Antenna Names");
  toSend.replace("{mr}","");

  toSend += FPSTR(HTTP_APP_STYLE);
  toSend += FPSTR(HTTP_APP_HEAD_END);
  toSend.replace("{h}", "<p>Antenna Configuation</p>");
    
  toSend += FPSTR(HTTP_NAME_FORM);
  sprintf(pageArgLength,"%d",ANTENNA_NAME_SIZE);  // convert to character string
  for ( i=0; i<NUM_ANTENNA_PORTS; i++ ) {
    itemBuf = FPSTR(HTTP_NAME_ITEM);
    sprintf(pageArgName,"a%d",i);
    itemBuf.replace("{id}",pageArgName);
    itemBuf.replace("{l}",pageArgLength);
    
    // either use placeholder='{p}' or value={p}
#ifdef USE_APPSTORY
    if (antennaName[i][0] == 0) {
      sprintf(antdefault,"placeholder='%s %d'","Antenna",i+1);
    } else {
      sprintf(antdefault,"placeholder='%s' value='%s'",antennaName[i],antennaName[i]);
    }
#else
    if (thamConfig.antennaName[i][0] == 0) {
      sprintf(antdefault,"placeholder='%s %d'","Antenna",i+1);
    } else {
      sprintf(antdefault,"placeholder='%s' value='%s'",thamConfig.antennaName[i],thamConfig.antennaName[i]);
    }
#endif
    itemBuf.replace("{p}",antdefault);
    toSend += itemBuf;
  }
  toSend += "<br/>";

  toSend += FPSTR(HTTP_NAME_BUTTONS);
  toSend += FPSTR(HTTP_FORM_END);
  toSend += FPSTR(HTTP_END);

  sendHTTP(request, toSend);
}

// ==========================================================================================
void handleShowButtons(AsyncWebServerRequest *request) {
  int i = 0;
  int j = 0;
  int int_t;
  int inpRadio = 0;
  int inpPort = 0;
  String itemStr = "";
  String toSend = "";
  
  if (checkInProgress(request)) return;
     
  if (request->hasParam("v") && request->hasParam("a")) {  // did we get a button press?
    inpRadio = request->getParam("v")->value().toInt();
    inpPort = request->getParam("a")->value().toInt();
    DEBUGPRINTF1("Page served handleButtons R:%d P:%d\n", inpRadio,inpPort);
    if (inpRadio == 99 ) {
      antennaArrayClear();
    } else {
      if (inpPort == 99 ) {
        antennaRadioClear(inpRadio);
      } else {
        // set radio port to this button.
        antennaSetPort(inpRadio, inpPort);
      }
    }
    doRelayCommand(false);
  } else {
    DEBUGPRINTF1("Page served handleButtons No parameters\n");
  }

  toSend = FPSTR(HTTP_ANTENNA_PAGE);
  // set up buttons ( dynamicly )
  for (j=0;j<NUM_ANTENNA_PORTS; j++) {
    toSend += FPSTR(HTTP_ANTENNA_START) ;  // set the <tr>
    
    for (i=0;i<NUMRADIOINPUTS; i++) {
      
      //DEBUGPRINTF1("Button R:%d P:%d A:%d\n",i,j, antennaArray[i][j]);
      if ( i % 2 == 0) { // even number?
        itemStr  = FPSTR(HTTP_ANTENNA_LED);
        itemStr += FPSTR(HTTP_ANTENNA_BUTTON);
        itemStr += FPSTR(HTTP_ANTENNA_TEXT);
        // name needs to be configurable
#ifdef USE_APPSTORY
        itemStr.replace("{n}",antennaName[j]);  // set text name
#else
        itemStr.replace("{n}",thamConfig.antennaName[j]);  // set text name
#endif
      } else {
        itemStr  = FPSTR(HTTP_ANTENNA_BUTTON);
        itemStr += FPSTR(HTTP_ANTENNA_LED);
      }
      itemStr.replace("{m}", ""); // set button name

      if (antennaArray[i][j] ) {  // is relay on?
        itemStr.replace("{a}","y");  // LED on
        itemStr.replace("{c}","bt r");
      } else {
        itemStr.replace("{a}","b");  // LED off
        // decide if we want the button on other side to be red too.
        int_t = i;
        if (i % 2 == 0) { 
          if (int_t < (NUMRADIOINPUTS-1) )int_t++;  // set to other side.
        } else {
          int_t--;
        }
        if (antennaArray[int_t][j]) {    // check other side bit
            itemStr.replace("{c}","bt r");
        } else {
          itemStr.replace("{c}","bt g");
        }
      }
      // Set in return parameters
      itemStr.replace("{v1}", String(i));
      itemStr.replace("{v2}", String(j));
      
      toSend += itemStr;  // put in send buffer
    }
  }
  
  // Add in the All Off buttons
  toSend += FPSTR(HTTP_ANTENNA_START) ;  // set the <tr>
  for (i=0;i<NUMRADIOINPUTS; i++) { 
     
    if ( i % 2 == 0) { // even number?
      itemStr  = FPSTR(HTTP_ANTENNA_LED);
      itemStr += FPSTR(HTTP_ANTENNA_BUTTON);
    } else {
      itemStr  = FPSTR(HTTP_ANTENNA_BUTTON);
      itemStr += FPSTR(HTTP_ANTENNA_HOME);
    }
    itemStr.replace("{a}","b");           // set LED black
    itemStr.replace("{c}","bt g a");      // set button green
    // Set in return parameters
    itemStr.replace("{v1}", String(i));
    itemStr.replace("{v2}", "99");
    
    if ( NUMRADIOINPUTS > 1 ) {
        itemStr.replace("{m}", "Side OFF");    // set button label.
    } else {
      itemStr.replace("{m}", "All OFF");    // set button label.
    }
    if ( (NUMRADIOINPUTS > 1) && ( i % 2 == 0) ) { // if even side
      itemStr += FPSTR(HTTP_ANTENNA_BUTTON);
      // set up button for All OFF button
      itemStr.replace("{c}", "bx g");         // set button green
      itemStr.replace("{m}", "All OFF");    // set button label.
      itemStr.replace("{v1}", "99");
      itemStr.replace("{v2}", "99");
    }
    toSend += itemStr;
  }
  toSend += FPSTR(HTTP_ANTENNA_END) ;  // set the </tr>
  toSend += FPSTR(HTTP_ANTENNA_PAGEEND);  // set the ALL OFF and close the page

  //  response->addHeader("Connection", "keep-alive");  // may want to use keep alive for speed.
  sendHTTP(request, toSend);
}

// ==========================================================================================
void handleRoot(AsyncWebServerRequest *request) {
  String toSend = "";
  String pageText = "";
 
  DEBUGPRINTLN1("Page served handleRoot");
  if ( checkInProgress(request) ) return;

  toSend += FPSTR(HTTP_APP_HEAD);
  toSend.replace("{v}", "THAM Root");
  toSend.replace("{mr}","");

  toSend += FPSTR(HTTP_APP_STYLE);
  toSend += FPSTR(HTTP_APP_HEAD_END);
  toSend.replace("{h}", "");
  
  toSend += FPSTR(HTTP_ROOTTEXT);   
                                                                                        
  // xxx toSend.replace("{dn}",FPSTR(THAM_DEVICE_FUNCTION));
  // xxx toSend += FPSTR(HTTP_ONPORTAL);
  
  if (WiFi.getMode() == WIFI_STA) {
    toSend.replace("{n}", WiFi.SSID());
    toSend.replace("{ip}", WiFi.localIP().toString());
 
  } else {
    toSend.replace("{n}", softAP_ssid);
    toSend.replace("{ip}", apIP.toString());
  }
  toSend += FPSTR(HTTP_THAMNET);
  if (WiFi.getMode() == WIFI_STA) {
    toSend += FPSTR(HTTP_ROOT_WIFI_TXT);
  } else {
    toSend += FPSTR(HTTP_ROOT_THAM_TXT);
  }
  
  toSend += FPSTR(HTTP_ROOT_FORM);
  // add in the buttons
  if ((WiFi.isConnected()) ||
      (WiFi.getMode() == WIFI_STA )){  // check situation
    toSend += FPSTR(HTTP_ROOT_THAMBUTTON);
  } else {
     toSend += FPSTR(HTTP_ROOT_WIFIBUTTON);
  }
#ifdef USE_APPSTORY
   if (WiFi.isConnected()) {  // if on internet, show update button.
     toSend += FPSTR(HTTP_ROOT_FWBUTTON);
  }
#endif
  toSend += FPSTR(HTTP_FORM_END);
  toSend += FPSTR(HTTP_END);

  sendHTTP(request, toSend);
}

// ==========================================================================================
void handleThamNetwork(AsyncWebServerRequest *request){

  DEBUGPRINTLN1("Page served handleThamNetwork");

  if (checkInProgress(request)) return;

  doThamAP(false);                // setup THAM network in main loop
                      
  // send message
  sendHTTPMessage(request,"/","20", HTTP_THAMNET_MSG);

}

// ========================================= setup WebServer Functions ===========================================
//
// The webserver used the Async version also used by IAS
//
void setupWebServer() {

  webServer.onNotFound(handleNotFound);
  webServer.on("/", HTTP_GET+HTTP_POST, handleRoot);             // might come in either as GET or POST
  webServer.on("/showbuttons", HTTP_GET+HTTP_POST, handleShowButtons);
  webServer.on("/wifisetup", HTTP_GET+HTTP_POST, handleWifiSetup);

  webServer.on("/antennasetup", HTTP_POST, handleAntennaSetup);
  webServer.on("/antennasave", HTTP_POST, handleAntennaSave);
  webServer.on("/thamnetwork", HTTP_POST, handleThamNetwork);
#ifdef USE_APPSTORY
  webServer.on("/checkupdate", HTTP_GET+HTTP_POST, handleCheckUpdate);
#else
  webServer.on("/wifisave", HTTP_POST, handleWifiSave);
  webServer.on("/wifiwait", HTTP_GET+HTTP_POST, handleWifiWait);
#endif
  
  webServer.begin();     // no status,  assume it started ok...

  DEBUGPRINTLN1("Web server started");
}
// ================================== do THAM STA mode Change =====================================
void doThamSTA(bool loopFlag) {

  if ( !loopFlag ) {
    doThamSTARequest = true;
    return;
  }
  if ( !doThamSTARequest ) {
    return;
  }
  doThamSTARequest = false;
  DEBUGPRINTLN1("in doThamSTA");
    
  delay(500);                  // let any web pages get sent

  WiFi.softAPdisconnect(true); // turn off wifi AP

}
// ================================== do THAM AP mode Change =====================================
void doThamAP(bool loopFlag) {

  if ( !loopFlag ) {
    doThamAPRequest = true;
    return;
  }
  if ( !doThamAPRequest ) {
    return;
  }
  doThamAPRequest = false;
  DEBUGPRINTLN1("in doThamAP");
  delay(500);                    // let web page get sent.
    
  beginThamAP();                  // Setup AP mode now.
  setupWebServer();               // Now start the web service 
}  

// ===================================================================================
// want to try auto connect first,  most likely already have.
#ifndef USE_APPSTORY
void setupThamWiFi() {
  int r_stt = 0;
  char mySSID[64] = "";

  DEBUGPRINTLN1("in setupThamWiFi");
  while (true) {
    WiFi.SSID().toCharArray(mySSID,sizeof(mySSID));  // get any pre-existing SSID
    
    // print pre-existing information... 
    //WiFi.localIP().toString().toCharArray(myIP,sizeof(myIP));
    DEBUGPRINTF1("Have SSID: '%s'\n", mySSID ); 
    
    DEBUGPRINT1("Wi-Fi mode set to WIFI_STA ...");
    r_stt = WiFi.mode(WIFI_STA);                    // first try set station Mode,  will attempt connect.
    DEBUGPRINTF1("%s\n", r_stt ? "Success" : "Failed!");

    WiFi.setAutoConnect(true);                      // turn on Auto Connect
 
    if (strlen(mySSID) > 0) {                       // have a previous SSID?
      // setup WiFi, no need to config SSID and password
    
      WiFi.begin ();                                // start WiFi module
      DEBUGPRINT1("trying auto connect, status: ");
      DEBUGPRINTLN1( WiFi.waitForConnectResult() );
    }

#ifdef WIFI_DEBUG      // if we want to force it back off...
    if (WiFi.isConnected()) {                      // did we connect?
      Serial.println("Connected! ... DEBUG disconnecting...");
      WiFi.disconnect();  // testing clear out previous informaiton
      delay(500);
    }
#endif
    if (WiFi.isConnected()) {   // did we connect?
          DEBUGPRINT1("connected as ");
          DEBUGPRINTLN1(WiFi.localIP());
      break;                    // yes - done
    }
   
    if (strlen(thamConfig.wifiSSID) > 0) {
      WiFi.begin( thamConfig.wifiSSID, thamConfig.wifiPassword );  // now try with saved credentials
      DEBUGPRINT1("Tying saved credentials connect, status: ");
      r_stt = WiFi.waitForConnectResult();
      DEBUGPRINTLN1( r_stt );
    }

#ifdef WIFI_DEBUG1
    if (WiFi.status() == WL_CONNECTED) {   // did we connect?
      Serial.println("Connected! ... DEBUG disconnecting...");
      WiFi.disconnect();  // testing clear out previous informaiton
      delay(500);
    }
#endif
    if (WiFi.isConnected()) {   // did we connect?
          DEBUGPRINT1("connected as ");
          DEBUGPRINTLN1(WiFi.localIP());
      break;                    // yes - done
    }
    // Neither auto connect or saved credentials connect succeeded.
    // now we set up our own wifi access point and let user connect.
    beginThamAP();
   
    break;
  }
}
#endif // USE_APPSTORY

// ================================================ SETUP ================================================
void setup() {
  int i = 0;
  String strDeviceName(softAP_ssid);

#ifdef USE_APPSTORY
  pinMode(IAS.dPinConv(runLED), OUTPUT);
#else
  pinMode(runLED, OUTPUT);
#endif
  tickerRunLED.attach_ms(SETUP_LED_TIME, handleRunLED);      // Start the run LED in FAST time

#ifdef DEBUGLEVEL >=1
  if (!Serial){
    Serial.begin(SERIAL_SPEED);
    while (!Serial){
      delay(10);
    }
    delay(100);
    Serial.print(F("\n\n\n\n\n"));
  }
#endif
               
  // We just rebooted,  we don't know where the antenna relay bits are,  
  // We could save them.. and re-constitute the relay status. 
  //    however if it is a power off situation, then the relays are clear.
  // we could clear them...
  //    however if not a power off, then a transmitter could be using one.
  // we could wait for the user to press something...  
  //    The relay button display would show something then.  

#ifndef USE_APPSTORY
  // -- set up Tham Library ----------------------------
  readThamConfig();                                          // read configurations
  setupThamWiFi();                                           // set up the THAM Wifi
#else
  #ifdef  ESP8266
    IAS.onFirstBoot([]() {
      DEBUGPRINTLN1(F(" Manual reset necessary after serial upload!"));
      DEBUGPRINTLN1(F("*-------------------------------------------------------------------------*"));
      ESP.restart();
    });
  #endif  
  stopWifiService();                                         // clear everything from WiFi
  // -- set up IOTAppStory Library ----------------------------
  IAS.preSetDeviceName(strDeviceName);
  IAS.preSetAutoUpdate(false);                                // //do not check for firmware update on boot up
  IAS.preSetAutoConfig(false);                               // Do NOT go into config mode if Wifi Not found.  can manually call this. 
  IAS.preSetAppVersion(F("1.1.0"));
   
  IAS.addField( ThamConfigWord, "THAM Config Word", 8, 'N'); // a place for THAM persistent setting (Not Used)

  for (i=0; i<NUM_ANTENNA_PORTS; i++) {                      // set up Antenna Name storage in EEPROM
    DEBUGPRINTF3("Antenna Name Set:'%s'\n",antennaName[i]);
    IAS.addField(antennaName[i], "Antenna", ANTENNA_NAME_SIZE);   // addField treats the 2nd parameter as a constant, so I can't change it here.
  }                                                          // otherwise I would use "Antenna 1" etc etc...
  IAS.onFirmwareUpdateDownload( handleIASUpdateStarted ) ;   // Called when the app starts to download updates.
 
  IAS.onFirmwareUpdateError( handleIASUpdateError ) ;        // Called when downloading firmware updates end in error.
 
  IAS.onFirmwareUpdateSuccess( handleIASUpdateDone ) ;       // Called when the downloaded firmware is successfully installed.

  IAS.begin('L');                                            // Optional parameter: What to do with EEPROM on First boot of the app? 
                                                             // 'F' Fully erase | 'P' Partial erase(default) | 'L' Leave intact
                                                             // NOTE: begin is needed,  it will read the config and our app fields.
                                                             // the config mode call back will catch and avoid if in THAM wifi mode

#endif  // USE_APPSTORY

#if DEBUGLEVEL >=1  
  for (i=0; i<NUM_ANTENNA_PORTS; i++) {                        // set up Antenna Name storage in EEPROM
#ifdef USE_APPSTORY
    Serial.printf("Antenna Name after:'%s'\n",antennaName[i]);
#else
    Serial.printf("Antenna Name after:'%s'\n",thamConfig.antennaName[i]);
#endif
  }
#endif

#ifdef USE_APPSTORY 
  // We arrive here after configuration is complete and if not THAM wifi,  we are on a network.
 
  IAS.setCallHome(false);                                    // Call home is done by user request.
  IAS.setCallHomeInterval(600);                              // Set up anyway,

  //-------- IAS is set up, now set up the app ---------------
  
  if ( ! WiFi.isConnected() ) {                              // if we and not connected to wifi set THAM AP mode, 
    beginThamAP();                                           // set up the THAM AP
    WiFi.setAutoConnect(false);                              // reset autoconnect
  } else {
    WiFi.setAutoConnect(true);                               // set autoconnect
    WiFi.setAutoReconnect(true);                             // and reconnect if lost AP
  }
#endif // USE_APPSTORY

#ifdef UCN5812_DRIVE
   setupUCN5812();
#endif  

  setupWebServer();                                          // set up the THAM web server
  
  tickerRunLED.attach_ms(RUN_LED_TIME, handleRunLED);        // Start the run LED

}

// ================================================ LOOP =================================================
void loop() {

#ifdef USE_APPSTORY
// skipped as we do not automatically update,  this is a relay controller.  it is by user request only.
//IAS.loop();     // this routine handles the calling home on the configured itnerval 
                  // as well as reaction of the Flash button. 
                  // If short press: update of sketch, long press: Configuration
                 
  // other housekeeping
  // NOTE:  These routines only do the function if requested by a handler.
#endif

  doRelayCommand(true); // if requested, send data to shift registers 
  
  doThamConfigSave(true);  // if requested, save antenna names.

  if ( wifidnsStarted ) dnsServer.processNextRequest();
  doThamAP(true);       // if requested, set up THAM network 
  doThamSTA(true);      // if requested, transition to STA mode
  doConfigMode(true);   // if requested, do config mode change.

#ifdef USE_APPSTORY
  doCheckUpdate(true);  // if requested, do the firmware update.
#endif


}
