/*
2017 Romain Acciari
Arduino MKRFOX1200 Bike beacon project.

######
SigFox message has a 12 bytes payload:
---
GPS position (3+3 bytes - 48bits):
Latitude and longitude are encoded on 3 bytes (24bits) each.
Each value is multiplied by 10000 and rounded.
The precision is around 11 meters.
* Divide each value by 10000.
---
Last acquisition time before sending message (2 bytes - 16bits):
Number of minutes since start of the year divided by 10 for a 6 minutes precision.
---
Voltage (1 byte - 8bits):
Maximum voltage of the board is 5V, the value is multiplied by 10 and rounded
(fits in 6bits for maximum compression).
* Divide the value by 10.
---
Available: 3 bytes
######


********************
SigFox custom message (12 bytes - 96 bits):
glatitude::int:24 glongitude::int:24 gdate::int:16 voltage::int:8
********************
*/

#include <SigFox.h>
#include <ArduinoLowPower.h>
#include <NMEAGPS.h>
#include <GPSport.h>
#include <ctime>

#ifndef GPSport_h
#define GPSport_h
/*
#if !defined( GPS_FIX_LOCATION )
  #error You must uncomment GPS_FIX_LOCATION in GPSfix_cfg.h!
#endif
*/
// GPS serial port connected to PIN 13 and 14 on the Arduino MKRFox1200 (Serial1)
#define gpsPort Serial1
#define GPS_PORT_NAME "Serial1"
#endif

// Port used to monitor serial port
#define DEBUG_PORT Serial

// GPS backup mode command (RXM-PMREQ)
const unsigned char ubxBackupMode [] = {
  0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x4D, 0x3B };
// GPS message to wake up from backup mode
const unsigned char ubxWakeUp [] = {
  0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x4C, 0x37};
// GPS command to set in Power Saving Mode
const unsigned char ubxPSM [] = {
  0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92 };

// Delay between GPS commands (millis)
const uint32_t GPS_COMMAND_DELAY = 1000;

// GPS acquisition duration limit (seconds)
#define GPS_ACQ_HARD_LIMIT = 1200
#define GPS_ACQ_SOFT_LIMIT = 120

// Disable DEBUG for production
#define DEBUG 1
// Low Power loop delay (recommended default 10 minutes: 10 * 60 * 1000)
#define NAPTIME 10 * 60 * 1000

// This parses the GPS characters
NMEAGPS gps;
// This holds on to the latest values
gps_fix fix;

// Set GPS module to PSM after first fix
bool gps_first_fix = true;

// Seconds since GPS last fix
unsigned long gps_last_fix;

// Struct of the SigFox message in bytes
struct sigfoxmsg_t {
  byte latitude[3];
  byte longitude[3];
  byte date[2];
  byte voltage[1];
} sfmsg;

// SigFox transmission errors since last successful
int sf_errors = 0;

// Arduino battery voltage value on A0
int voltage = 0;


// Print message to DEBUG_PORT if DEBUG is enabled
void printmsg(String msg, bool newline = true) {
  if (DEBUG) {
    if (newline) {
      DEBUG_PORT.println(msg);
    }
    else {
      DEBUG_PORT.print(msg);
    }
  }
}
// end of printmsg()

// Print error messages and show a SOS using onboard LED
void error(String msg, bool fatal = false) {
  printmsg("ERROR: " + msg);
  while(fatal) {
    led_sos();
  }
  led_sos();
}
// end of error()

// Return String from integers on 2 digits (used for date and time)
String print2digits(int number) {
  if (number < 10) {
    return("0" + String(number));
  }
  return(String(number));
}
// end of print2digits()

// Get battery voltage
// Is it A0 or ADC_BATTERY? Does it work with Vin?
// ADC_BATTERY is for Vbatt only?
// A0 works with USB and Vin ?
void get_voltage() {
  float voltage = analogRead(A0) * (5.0 / 1023.0);
  //voltage = analogRead(A0) * (50.0 / 1023.0);
  printmsg("Current A0 voltage is: " + String(voltage) );
  printmsg("Current ADC_BATTERY voltage is: " + String(analogRead(ADC_BATTERY) * (1.0 / 1023.0)) );
  sfmsg.voltage[0] = lowByte( (int)(voltage * 10) );
}
// end of get_voltage()

// Send UBX specific commands to uBlox GPS module
void sendUBX( const unsigned char *progmemBytes, size_t len )
{
  //printmsg("Sending UBX command to GPS module");
  gpsPort.write( 0xB5 ); // SYNC1
  gpsPort.write( 0x62 ); // SYNC2

  uint8_t a = 0, b = 0;
  while (len-- > 0) {
    uint8_t c = pgm_read_byte( progmemBytes++ );
    a += c;
    b += a;
    gpsPort.write( c );
  }

  gpsPort.write( a ); // CHECKSUM A
  gpsPort.write( b ); // CHECKSUM B
}
// end of sendUBX

// Set the GPS module to Power Saving Mode
void gps_set_PSM() {
  printmsg("Setting GPS module to Power Saving Mode");
  gpsPort.flush();
  delay(GPS_COMMAND_DELAY);
  sendUBX(ubxPSM, sizeof(ubxPSM));
  delay(GPS_COMMAND_DELAY);
}
// end of gps_set_PSM()

// Set the GPS module to Backup mode (for long delays)
void gps_set_backup_mode(int duration = 0) {
  printmsg("GPS entering backup mode");
  gpsPort.flush();
  delay(GPS_COMMAND_DELAY);
  sendUBX(ubxBackupMode, sizeof(ubxBackupMode));
  delay(GPS_COMMAND_DELAY);
}
// end of gps_set_backup_mode()

// Wake up the GPS module from Backup mode
void gps_wake_up() {
  printmsg("Waking up the GPS module");
  sendUBX(ubxWakeUp, sizeof(ubxWakeUp));
  delay(GPS_COMMAND_DELAY);
}
// end of gps_wake_up()

// Get a location from GPS module
void gps_get_loc() {
  gps_wake_up();
  float lat, lon;
  // TODO: Leave the loop after X minutes and try later on a smaller delay
  while( !gps.available(gpsPort)
         || !fix.valid.location
         || !fix.valid.time
         || !fix.valid.date
         || gps_last_fix == fix.dateTime ) {
    while( gps.available(gpsPort) ) {
      fix = gps.read();
    }
    delay(100);
  }
  if (fix.valid.location) {
    lat = fix.latitude();
    lon = fix.longitude();
    printmsg("GPS location: " + String(lat,6) + "," + String(lon,6));
    int lat_i = (int) (lat * 10000);
    int lon_i = (int) (lon * 10000);
    sfmsg.latitude[0] = (int) ((lat_i & 0xFF0000) >> 16 );
    sfmsg.latitude[1] = (int) ((lat_i & 0x00FF00) >> 8  );
    sfmsg.latitude[2] = (int) ((lat_i & 0X0000FF)       );
    sfmsg.longitude[0] = (int) ((lon_i & 0xFF0000) >> 16 );
    sfmsg.longitude[1] = (int) ((lon_i & 0x00FF00) >> 8  );
    sfmsg.longitude[2] = (int) ((lon_i & 0X0000FF)       );
  }
  else {
    error("GPS failed to get location.");
  }
  if (fix.valid.time && fix.valid.date) {
    fix.dateTime.year = 0;
    gps_last_fix = fix.dateTime;
    printmsg("Seconds since start of the year: " + String(fix.dateTime));
    sfmsg.date[0] = (int) (((gps_last_fix / (60 * 10)) & 0xFF00) >> 8);
    sfmsg.date[1] = (int) (((gps_last_fix / (60 * 10)) & 0x00FF)     );
  }
  else {
    error("GPS failed to get date and/or time.");
  }
}
// end of gps_get_loc()

void led_sos() {
  int ledoff_delay = 500;
  int ledon_sos [] = {500, 3000, 500};
  for(int i = 0; i < 9 ; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(ledon_sos[i/3]);
    digitalWrite(LED_BUILTIN, LOW);
    delay(ledoff_delay);
  }
  delay(1000);
}
// end of led_sos()

void sfSendMessage() {
  SigFox.begin();
  delay(100);
  SigFox.beginPacket();
  SigFox.write(sfmsg.latitude, sizeof(sfmsg.latitude));
  SigFox.write(sfmsg.longitude, sizeof(sfmsg.longitude));
  SigFox.write(sfmsg.date, sizeof(sfmsg.date));
  SigFox.write(sfmsg.voltage, sizeof(sfmsg.voltage));
  int ret = SigFox.endPacket();
  if (ret > 0) {
    error("Transmission to SigFox network failed.");
    sf_errors++;
  } else {
    if( sf_errors > 0 ) {
      printmsg(sf_errors + " unsuccessful attempts before sending message to the SigFox network.");
    }
    printmsg("Transmission to SigFox network successful.");
    sf_errors = 0;
  }
  if (SigFox.parsePacket()) {
    printmsg("SigFox response:");
    while (SigFox.available()) {
      printmsg("0x", false);
      if(DEBUG) {
        DEBUG_PORT.println(SigFox.read(), HEX);
      }
    }
  } else {
    error("Could not get any response from the SigFox network.");
  }
  printmsg("SigFox module SIGFOX status: " + String(SigFox.status(SIGFOX)));
  printmsg("SigFox module ATMEL status: " + String(SigFox.status(ATMEL)));
}

// Sleep mode
void sleep_mode() {
  printmsg("Sleeping for " + String(NAPTIME / 1000) + " seconds");
  gps_set_backup_mode();
  delay(NAPTIME);
  //LowPower.sleep(NAPTIME);
  printmsg("Waking up");
  printmsg("");
}
// end of sleep_mode()

void setup() {

  if (DEBUG) {
    pinMode(LED_BUILTIN, OUTPUT);
    DEBUG_PORT.begin(9600);
    while (!DEBUG_PORT) {};
  }

  if (!SigFox.begin()) {
    error("SigFox shield error or not present!");
    return;
  }

  if (DEBUG) {
    SigFox.debug();
    // Display module informations
    printmsg("Arduino MKRFox1200");
    printmsg("SigFox module firmware version: " + String(SigFox.SigVersion()));
    printmsg("SigFox device ID  = " + String(SigFox.ID()));
    printmsg("SigFox device PAC = " + String(SigFox.PAC()));
    printmsg("SigFox on-board module temperature: " + String(SigFox.internalTemperature(),0));
    printmsg("");
  }
  else {
    SigFox.noDebug();
    delay(100);
  }

  // Send the module to the deepest sleep
  delay(100);
  SigFox.end();

  printmsg("Looking for GPS device on " GPS_PORT_NAME);
  gpsPort.begin(9600);
}
// end of setup()

void loop() {
  if ( gps_first_fix && fix.valid.location ) {
    gps_set_PSM();
    gps_first_fix = false;
  }
  gps_get_loc();
  get_voltage();
  sfSendMessage();
  sleep_mode();
}
// end of loop()
