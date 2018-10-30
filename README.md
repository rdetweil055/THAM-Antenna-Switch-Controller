# THAM-Antenna-Switch-Controller
THAM Antenna Switch Controller is an ESP IoT Chip based device to control relays of an antenna switch device

   Clearn up tasks
   
   1) merge config structure into IAS field
         Does not look possible yet,

   This version can be use with or without the IOTAppStory library.
        the USE_APPSTORY define controls the compile option.

   This device firmware is a relay control and status display.
   This firmware is designed for a 2X6 antenna switch, however
   with some modification it could server any number of inputs
   and outputs.  The hardware selection is 8266.  If the device
   is to be mounted on a tower some distance from the radion 
   station an ability to do a OTA update is needed.  Also an 
   external antenna is needed.  I used the WROOM-02U as the 
   8266 device that has a IPEX anenna connector and a 6db gain
   panel mount 2.4 ghz antenna in a weather proof enclosure.

   In many routines there is a parameter call loopFlag,  due to
   the async nature of the program we do the heavy lifting of any
   function in the main loop(). This lets the handlers
   format a HTTML return string to the device as fast as possible.
   In order to use a routine the routine must first be called
   with a false loopFlag, this allows the handlers to be nonblocking
   Then the routine must be called again with a true loopFlag.
   The routine will then do the function.
   
   Do be careful with the pre processor definition "#".  I use them
   extensively and sometimes to a fault.

   As I have several old style UCN5812 serial latches on hand, 
   I made this firmware for them.  In the future I will use an
   I2C GPIO extender and opto isolators to power the relays.
   Programming for this is not included here.  A future update
   will have a USE_I2C_EXTENDER directive to include that code.

   As this is designed to be mounted far away from a computer,
   this design is to be able to detect that a firmware update is 
   available for the device and automaticly download the new
   firmware.  

   The design does not update automatically,  the user must 
   requests the firmware update check via the main web page.

   As these devices control antenna relays that may have significant power
   running through them, the device must only do the firmware update when 
   directed by the user through the main web page request.

   2X6 switch notes
   
   NOTE:  Only 1 antenna port can be active.  if more the device has more 
          than one radio input, then device must prevent any other request
          for that antenna port.

   NOTE:  8266 can only handle 9 safe GPIO pins, 2 more if Serial line 
          is used,then no debugging...
          Anything more needs a serial latch driver chip

   CORE FUNCTIONS
   1) Allow user to connect a radio port to an antenna port
   2) Send commands to relay driver to effect the user command.
   3) Display Relay Status to the user
   4) Function independent of internet when needed
   6) Allow for Firmware update.
   7) Operate at a long distance from the station.

   Sub Functions
   1) ability connect to WiFi network as a client.
   2) ability to function as an AP (access point) without internet
   3) display through Web pages on PC or Smart device
   4) Ability to rename Antenna ports

   
