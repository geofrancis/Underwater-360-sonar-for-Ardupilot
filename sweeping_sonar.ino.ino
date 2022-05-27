// Includes
#include "mavlink/common/mavlink.h"        // Mavlink interface
#include "mavlink/common/mavlink_msg_obstacle_distance.h"
#include <Servo.h>
#include <SoftwareSerial.h>

#define POTI_PIN A0
#define EXTERNAL_BAUDRATE 1500000

int potiValue = 0;
int FOV = 120; //multiple of res and even(res is 3 degree for the TF02-pro)
int lidarAngle = 0;
int messageAngle = 0;
int res = 3;
uint16_t distances[72];
  
  
int pinRX = 10;
int pinTX = 11;
SoftwareSerial mySerial(pinRX, pinTX);
unsigned char data_buffer[4] = {0};
int adjusted = 0;
int distance = 0;
int range = 0;
unsigned char CS;
uint8_t Index;
byte received;


Servo myservo;  // create servo object to control a servo
int posi = 0;    // variable to store the servo position


char serial_buffer[15];



//=======================================
void setup() {
  Serial.begin(EXTERNAL_BAUDRATE); // USB
  mySerial.begin(9600);
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  
  memset(distances, UINT16_MAX, sizeof(distances)); // Filling the distances array with UINT16_MAX
 }
//======================================


// Scanning fuction. Adapt to your needs
int16_t tfDist = 0;    // Distance to object in centimeters
uint16_t pulses = 0;





void loop() {

potiValue = analogRead(POTI_PIN);
lidarAngle = map(potiValue, 185, 815, -90, 90);          // Adjust for the poti you use
messageAngle = map(lidarAngle, -FOV/2, FOV/2, 0, FOV);
  
  
  if (lidarAngle <= -FOV/2) 
  {
 for (posi = 0; posi <= 180; posi += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(posi);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
  }

  if (lidarAngle >= FOV/2)
  {
 for (posi = 180; posi >= 0; posi -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(posi);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
  
  }
   
  if(lidarAngle%res == 0){ // get a distance reading for each res (3 degree) step
    readsonar();
    send_pos();
  }

}
void readsonar(){     
   if (mySerial.available() > 0) {
 
    delay(4);
 
    // Check for packet header character 0xff
    if (mySerial.read() == 0xff) {
      // Insert header into array
      data_buffer[0] = 0xff;
      // Read remaining 3 characters of data and insert into array
      for (int i = 1; i < 4; i++) {
        data_buffer[i] = mySerial.read();
      }
 
      //Compute checksum
      CS = data_buffer[0] + data_buffer[1] + data_buffer[2];
      // If checksum is valid compose distance from data
      if (data_buffer[3] == CS) {
        range = 0.1 * (data_buffer[1] << 8) + data_buffer[2];
         //adjusting for the speed of sound in water
        distance = range * 4.126;
      }
    }
  }
}

void send_pos(){

//MAVLINK DISTANCE MESSAGE
  int sysid = 1;                   
  //< The component sending the message.
  int compid = 158;    

  uint64_t time_usec = 0; /*< Time since system boot*/
  uint8_t sensor_type = 0;
  distances[messageAngle/res] = tfDist-2.0f; //UINT16_MAX gets updated with actual distance values
  uint8_t increment = 3;
  uint16_t min_distance = 5; /*< Minimum distance the sensor can measure in centimeters*/
  uint16_t max_distance = 4000; /*< Maximum distance the sensor can measure in centimeters*/
  float increment_f = 0;
  float angle_offset = -FOV/2;
  uint8_t frame = 12;
  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
 mavlink_msg_obstacle_distance_pack(sysid,compid,&msg,time_usec,sensor_type,distances,increment,min_distance,max_distance,increment_f,angle_offset,frame);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes) 
  //delay(1);
  Serial.write(buf, len);

}
