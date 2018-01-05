/*
2017 Romain Acciari
Bike Tracker project using LPWAN network.

This sketch runs on Arduino MKRFOX1200 with a uBlox Neo-6M GPS module.
It sends the GPS position every 10 minutes using the SigFox network.

######
SigFox message has a 12 bytes payload:
---
GPS position (3+3 bytes - 48bits):
Latitude and longitude are encoded on 3 bytes (24bits) each.
Each value is multiplied by 10000 and rounded.
The precision is around 11 meters.
* Divide each value by 10000.
---
Minutes since the start of the current year, rounded to the nearest 10 (2 bytes - 16bits):
Number of minutes since start of the year divided by 10 for a 10 minutes precision.
* Multiply by 10 to have approx minutes from start and calculate the date.
---
Voltage of the battery (1 byte - 8bits):
Maximum voltage of the board is 5V, the value is multiplied by 10 and rounded
(fits in 6bits for maximum compression).
* Divide the value by 10.
---
Available: 3 bytes
######


********************
SigFox custom message (12 bytes - 96 bits):
glatitude::int:24 glongitude::int:24 gdate::int:16 voltage::int:8
*/

#include <SigFox.h>
#include <ArduinoLowPower.h>
#include <NMEAGPS.h>
#include <GPSport.h>
#include <ctime>

// Verify GPS defines from NeoGPS library
#ifndef GPSport_h
#define GPSport_h
#if !defined( GPS_FIX_DATE )
  #error You must uncomment GPS_FIX_LOCATION in GPSfix_cfg.h!
#endif
#if !defined( GPS_FIX_TIME )
  #error You must uncomment GPS_FIX_LOCATION in GPSfix_cfg.h!
#endif
#if !defined( GPS_FIX_LOCATION )
  #error You must uncomment GPS_FIX_LOCATION in GPSfix_cfg.h!
#endif
// GPS serial port connected to PIN 13 and 14 on the Arduino MKRFox1200 (Serial1)
#define gpsPort Serial1
#define GPS_PORT_NAME "Serial1"
#endif

// Port used to monitor serial port when DEBUG is true
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

// Delay after GPS commands (millis)
const uint32_t GPS_COMMAND_DELAY = 1000; // default to 1000

// Delay after SigFox commands (millis)
const uint32_t SF_COMMAND_DELAY = 200; // default to 200

// GPS acquisition duration limit (seconds)
const uint32_t GPS_ACQ_HARD_LIMIT = 1200; // Default to 1200
const uint32_t GPS_ACQ_SOFT_LIMIT = 120;  // default to 120

// Set acquisition duration to hard limit after 7200 seconds
const uint32_t GPS_FORCE_REFRESH = 7200;

const uint32_t GPS_ACCURACY_LIMIT = 10000;

// Disable DEBUG for production
#define DEBUG 0 // Default to 0
// Low Power loop delay (millis)
// Default to 10 minutes as it is the maximum you can get from the
// SigFox network. Do not use a smaller delay or you will abuse the network
// and SigFox can shutoff the device message link.
#define NAPTIME 10 * 60 * 1000 // Default to 10 * 60 * 1000

// This parses the GPS characters
NMEAGPS gps;
// This holds on to the latest values
gps_fix fix;

// Set GPS module to PSM after first fix
bool gps_first_fix = true;

// Time of the last fix from the GPS module (in seconds)
unsigned long time_gps_last_fix_s = 0;
// Time of the last fix using Arduino internal millis() function
unsigned long time_gps_last_fix_ms = 0;
// Current GPS location
float latitude;
float longitude;
// Last transmitted GPS location
int last_transmitted_latitude;
int last_transmitted_longitude;

// Struct of the SigFox message in bytes
struct sigfoxmsg_t {
  byte latitude[3];
  byte longitude[3];
  byte date[2];
  byte voltage[1];
} sfmsg;

// SigFox transmission errors since last successful
int sf_errors = 0;


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
void error(String msg, bool fatal = false, bool onboard_led = false) {
  printmsg("ERROR: " + msg);
  // If fatal, you enter an infinite loop here
  while(fatal) {
    led_sos();
  }
  // Blink for visual signal. Avoid in production.
  if( onboard_led ) {
    led_sos();
  }
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
bool get_voltage() {
  float voltage = analogRead(A0) * (5.0 / 1023.0);
  //voltage = analogRead(A0) * (50.0 / 1023.0);
  printmsg("Current A0 voltage is: " + String(voltage) );
  printmsg("Current ADC_BATTERY voltage is: " + String(analogRead(ADC_BATTERY) * (1.0 / 1023.0)) );
  sfmsg.voltage[0] = lowByte( (int)(voltage * 10) );
  return true;
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
// Return true if a new fix is acquired
bool gps_get_loc() {
  gps_wake_up();
  unsigned long timeout;
  // Set the GPS acquisition duration limit
  if( (time_gps_last_fix_ms + GPS_FORCE_REFRESH) < millis() ) {
    printmsg("Setting soft timeout to get a GPS fix.");
    timeout = millis() + (GPS_ACQ_SOFT_LIMIT * 1000);
  }
  else {
    printmsg("Setting HARD timeout to get a GPS fix.");
    timeout = millis() + (GPS_ACQ_HARD_LIMIT * 1000);
  }
  while( ( !gps.available(gpsPort)
          || !fix.valid.location
          || !fix.valid.time
          || !fix.valid.date
          || time_gps_last_fix_s == fix.dateTime )
         && millis() < timeout ) {
    while( gps.available(gpsPort) ) {
      fix = gps.read();
    }
    delay(500);
  }
  // Exit if unable to get a new fix
  if( fix.valid.time && fix.valid.date && time_gps_last_fix_s == fix.dateTime ) {
    printmsg("Failed to get a new fix.");
    return false;
  }
  if (fix.valid.location) {
    latitude = fix.latitude();
    longitude = fix.longitude();
    printmsg("GPS location: " + String(latitude,6) + "," + String(longitude,6));
    // Cast to integer with loss of precision
    int lat_i = (int) (latitude * GPS_ACCURACY_LIMIT);
    int lon_i = (int) (longitude * GPS_ACCURACY_LIMIT);
    // Cast to bytes array
    sfmsg.latitude[0] = (int) ((lat_i & 0xFF0000) >> 16 );
    sfmsg.latitude[1] = (int) ((lat_i & 0x00FF00) >> 8  );
    sfmsg.latitude[2] = (int) ((lat_i & 0X0000FF)       );
    sfmsg.longitude[0] = (int) ((lon_i & 0xFF0000) >> 16 );
    sfmsg.longitude[1] = (int) ((lon_i & 0x00FF00) >> 8  );
    sfmsg.longitude[2] = (int) ((lon_i & 0X0000FF)       );
  }
  else {
    error("GPS failed to get location.");
    return false;
  }
  if (fix.valid.time && fix.valid.date) {
    fix.dateTime.year = 0;
    time_gps_last_fix_s = fix.dateTime;
    time_gps_last_fix_ms = millis();
    printmsg("Seconds since start of the year: " + String(fix.dateTime));
    sfmsg.date[0] = (int) (((time_gps_last_fix_s / (60 * 10)) & 0xFF00) >> 8);
    sfmsg.date[1] = (int) (((time_gps_last_fix_s / (60 * 10)) & 0x00FF)     );
  }
  else {
    error("GPS failed to get date and/or time.");
    return false;
  }
  return true;
}
// end of gps_get_loc()

// Save transmitted location
void save_location() {
  last_transmitted_latitude = (int) (latitude * GPS_ACCURACY_LIMIT);
  last_transmitted_longitude = (int) (longitude * GPS_ACCURACY_LIMIT);
}
// end of save_location()

// Compare GPS location
bool gps_location_differs() {
  if( last_transmitted_latitude  != (int) (latitude * GPS_ACCURACY_LIMIT) &&
      last_transmitted_longitude != (int) (longitude * GPS_ACCURACY_LIMIT) ) {
    printmsg("Location is new.");
    return true;
  }
  printmsg("Location is the same.");
  return false;
}
// end of gps_location_differs()

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

bool sfSendMessage() {
  printmsg("Starting SigFox module.");
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
    sf_end();
    return false;
  }
  else {
    if( sf_errors > 0 ) {
      printmsg("WARNING: " + String(sf_errors) + " unsuccessful attempts before sending message to the SigFox network.");
    }
    printmsg("Transmission to SigFox network successful.");
    sf_errors = 0;
  }
  // Listen for a response from the SigFox network
  if (SigFox.parsePacket()) {
    printmsg("SigFox response:");
    while (SigFox.available()) {
      printmsg("0x", false);
      if(DEBUG) {
        DEBUG_PORT.println(SigFox.read(), HEX);
      }
    }
  }
  else {
    printmsg("No response from the SigFox network.");
  }
  sf_get_status();
  sf_end();
  return true;
}

// Send the module to the deepest sleep
void sf_end() {
  printmsg("Sending SigFox module to deep sleep.");
  delay(SF_COMMAND_DELAY);
  SigFox.end();
}
// end of sf_end()

void sf_get_status() {
  printmsg("SigFox module SIGFOX status: " + String(SigFox.status(SIGFOX)));
  printmsg("SigFox module ATMEL status: " + String(SigFox.status(ATMEL)));
}
// end of sf_get_status()

// Sleep mode
void sleep_mode() {
  printmsg("Sleeping for " + String(NAPTIME / 1000) + " seconds");
  gps_set_backup_mode();
  //delay(NAPTIME);
  LowPower.sleep(NAPTIME);
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

  delay(SF_COMMAND_DELAY);
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
  }
  sf_end();

  printmsg("Looking for GPS device on " GPS_PORT_NAME);
  gpsPort.begin(9600);
}
// end of setup()

void loop() {
  if( gps_get_loc() && get_voltage() && gps_location_differs() ) {
    if( sfSendMessage() ) {
      save_location();
    }
  }
  // Set GPS in PSM mode after first fix (avoid reset bug)
  if ( gps_first_fix && fix.valid.location ) {
    gps_set_PSM();
    gps_first_fix = false;
  }
  sleep_mode();
}
// end of loop()

