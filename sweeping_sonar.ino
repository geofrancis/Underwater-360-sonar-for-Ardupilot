// Includes
#include "mavlink/common/mavlink.h"        // Mavlink interface
#include "mavlink/common/mavlink_msg_obstacle_distance.h"
#include <Servo.h>


#define POTI_PIN A0
#define FCbaud 1500000
#define sonarbaud 9600

int pothigh = 700;
int potlow = 300;



int potiValue = 0;
int FOV = 120; //multiple of res and even(res is 3 degree for the TF02-pro)
int lidarAngle = 0;
int messageAngle = 0;
int res = 2;
uint16_t distances[72];
int target = 0;

unsigned char data_buffer[4] = {0};
int adjusted = 0;
int distance = 0;
int range = 0;
unsigned char CS;
uint8_t Index;
byte received;

HardwareSerial Serial1(USART1);
HardwareSerial Serial2(USART2);

Servo myservo;  // create servo object to control a servo
int posi = 0;    // variable to store the servo position

char serial_buffer[15];


void setup() {/////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Serial1.begin(FCbaud); // USB
  Serial2.begin(sonarbaud);
  
  myservo.attach(A1);  // attaches the servo on pin 9 to the servo object
  myservo.write(180); 

  memset(distances, UINT16_MAX, sizeof(distances)); // Filling the distances array with UINT16_MAX
 }
//======================================


// Scanning fuction. Adapt to your needs
int16_t tfDist = 0;    // Distance to object in centimeters


void loop() {///////////////////////////////////////////////////////////////////////////////////////////////////////////////

potiValue = analogRead(POTI_PIN);
//Serial.print ("potValue "); 
//Serial.println (potiValue);
lidarAngle = map(potiValue, potlow, pothigh, -60, 60);          // Adjust for the poti you use
//Serial.print ("message angle ");
//Serial.println (messageAngle);
//Serial.print ("lidar angle ");
//Serial.println (lidarAngle);
messageAngle = map(lidarAngle, -FOV/2, FOV/2, 0, FOV);

if(lidarAngle%res <=target)
  { // get a distance reading for each res (3 degree) step

 
//Serial.print ("posi ");
//Serial.println (posi);

readsonar();
moveservo();
send_pos();

  
  }

}

void moveservo(){

 if (lidarAngle <= -FOV/2)  
  {
    
 for (posi = 0; posi <= 180; posi += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(posi);              // tell servo to go to position in variable 'pos'

  }
  }
    if (lidarAngle >= FOV/2)
 
  for (posi = 180; posi >= 0; posi -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(posi);              // tell servo to go to position in variable 'pos'

  }
 }


  
void readsonar(){
 
 
   if (Serial2.available() > 0) {
    Serial2.write(0x55);
       delay(1);
    // Check for packet header character 0xff
    if (Serial2.read() == 0xff) {
      // Insert header into array
      data_buffer[0] = 0xff;
      // Read remaining 3 characters of data and insert into array
      for (int i = 1; i < 4; i++) {
        data_buffer[i] = Serial2.read();
      }

      //Compute checksum
      CS = data_buffer[0] + data_buffer[1] + data_buffer[2];
      // If checksum is valid compose distance from data
      if (data_buffer[3] == CS) {
        range = 0.1 * (data_buffer[1] << 8) + data_buffer[2];
         //adjusting for the speed of sound in water
       
      tfDist = range;
      }
    }
  }
}


void send_pos(){///////////////////////////////////////////////////////////////////////////////////////////////////////////
//MAVLINK DISTANCE MESSAGE
  int sysid = 1;                   
  //< The component sending the message.
  int compid = 158;    
  uint64_t time_usec = 0; /*< Time since system boot*/
  uint8_t sensor_type = 0;
  distances[messageAngle/res] = tfDist-2.0f; //UINT16_MAX gets updated with actual distance values
  uint8_t increment = 3;
  uint16_t min_distance = 30; /*< Minimum distance the sensor can measure in centimeters*/
  uint16_t max_distance = 500; /*< Maximum distance the sensor can measure in centimeters*/
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
  Serial1.write(buf, len);

}
