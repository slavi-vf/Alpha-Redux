// Interlock/Position Sensor for the 37N Alpha-Redux gantry system
// VulcanForms
//
// Author: Peter Reeves-Hall 
//
// Version History
//
// 2022-11-09   V3 add DO5 as a highspeed camera trigger (Slavi B)
// 2022-08-08   v2 invert logic for optical slot sensors (0V when blocked, 3.3V when open)
// 2022-08-05   v1 Initial release - *** need to check logic levels on slot sensors LOW vs HIGH ****
//
// Info / Notes:
// Setup IDE : https://learn.adafruit.com/adafruit-feather-m0-basic-proto/setup
// pins: https://learn.adafruit.com/adafruit-feather-m0-basic-proto/pinouts


#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  // Required for Serial on Zero based boards
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif

//pins
#define LASER_PIN 13      // on PCB LED and interlock output
#define HEAD_PIN 12     // optical slot detector on gantry laser head
#define CHAMBER_PIN 11  // chamber interlock
#define BEAM_PIN 10     // Optical beam analysis bay interlock 
#define OPM_PIN 9      // Optical power meter bay interlock
#define SYNC_PIN 5

#include <math.h>

unsigned long micro_time; // micro-second resolution timer
unsigned long p_counter;  // counter for serial messages
int laser_state; // 1= laser enable, 0= laser disable

int i;

int serial_byte;  

int head;    // laser head interlock status
int chamber; // chamber interlock status
int beam;    // Optical beam analysis bay interlock status
int opm;     // Optical power meter bay interlock status2

int head_old;    // old laser head interlock status
int chamber_old; // old chamber interlock status
int beam_old;    // old Optical beam analysis bay interlock status
int opm_old;     // old Optical power meter bay interlock status

// the setup function runs once when you press reset or power the board
void setup() {

  //configure Digital Outputs
  pinMode(LASER_PIN, OUTPUT);
  pinMode(SYNC_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW); // Red LED / laser  = OFF
  digitalWrite(SYNC_PIN, LOW);
  laser_state=0;

  //configure digital inputs
  pinMode(HEAD_PIN, INPUT_PULLUP);  // test if pull up mode required
  pinMode(CHAMBER_PIN, INPUT_PULLUP);
  pinMode(BEAM_PIN, INPUT_PULLUP);
  pinMode(OPM_PIN, INPUT_PULLUP);

  delay(500);
  Serial.begin(115200);  // use high baud rate to empty serial buffer faster, helps prevent main loop delay if sending lots of serial data. 
  Serial.println("Boot up"); // serial message
  p_counter=0;

  // fill old interlock status with off
  head_old=0;    
  chamber_old=0;
  beam_old=0;  
  opm_old=0;  
}

void get_inputs() {
  //invert logic , a low digital in is slot blocked
if (digitalRead(HEAD_PIN)==LOW) {head=1;} else {head=0;}     // get laser head interlock status
if (digitalRead(CHAMBER_PIN)==LOW) {chamber=1;} else {chamber=0;} // get chamber interlock status
if (digitalRead(BEAM_PIN)==LOW) {beam=1;} else {beam=0;}   // get Optical beam analysis bay interlock status
if (digitalRead(OPM_PIN)==LOW) {opm=1;} else {opm=0;}    // get Optical power meter bay interlock status
}

// the loop function runs over and over again forever
void loop() {

get_inputs();

i=0;
if (chamber==1) i+=1;
if (beam==1) i+=1;
if (opm==1) i+=1;

if (i==1) { //if only 1 interlock active then proceed with head slot interlock check. 
 if (head_old!=head) {  // rising/falling edge detection

  micro_time=micros(); // record time
  if (head==1) {
    laser_state=1;
    digitalWrite(LASER_PIN, HIGH); // change state of laser output
    digitalWrite(SYNC_PIN, HIGH);
  }
  else
  {
    laser_state=0;
    digitalWrite(LASER_PIN, LOW); // Red LED / laser  = OFF
    digitalWrite(SYNC_PIN, LOW);
  }

  //send serial udpate
  send_serial_update();
 }

}
else //If <>1 inerlock active then stay off 
{
 digitalWrite(LASER_PIN, LOW); // Red LED / laser  = OFF
 digitalWrite(SYNC_PIN, LOW);
 laser_state=0;
}

if (Serial.available() > 0) {  //listen for single byte receive e.g. <CR> then send serial status message
  serial_byte=Serial.read();  // clear the serial buffer
  micro_time=micros(); //  micro-second resolution timer, numeric rolls over every ~70 minutes
  send_serial_update();
}

//make new status the old status for next itteration of loop
head_old=head;    
chamber_old=chamber;
beam_old=beam;  
opm_old=opm;    
}

void send_serial_update(){
  p_counter+=1;  //increment serial message counter

  //serial output
  Serial.print("GI,");
  Serial.print(p_counter);
  Serial.print(",");
  Serial.print(micro_time);
  Serial.print(",");
  Serial.print(laser_state);
  Serial.print(",");
  Serial.print(head);
  Serial.print(",");
  Serial.print(chamber);
  Serial.print(",");
  Serial.print(beam);
  Serial.print(",");
  Serial.print(opm);
  Serial.print(",");
  Serial.print(head_old);
  Serial.println(""); // line end
}
