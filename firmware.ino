#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <WiFiServer.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include "Radio.pb.h"
#include "robot.pb.h"
#include "pb.h"
#include "pb_common.h"
#include "pb_decode.h"

//--------
#define ROBOT_ID 0
#define MODULE_VERSION 2 //Only v2 is supported for now
#define NETWORK_SSID "<SSID>"
#define NETWORK_PASS "<PASSWORD>"
#define WIFI_AP_PASS "AP"
#define WIFI_AP_SSID ""
//--------

#define SPI_SONIC_COMMAND_MOTOR0_SPEED 0x01
#define SPI_SONIC_COMMAND_MOTOR1_SPEED 0x02
#define SPI_SONIC_COMMAND_MOTOR0_RPM 0x03
#define SPI_SONIC_COMMAND_MOTOR1_RPM 0x04
#define SPI_SONIC_COMMAND_MOTOR_SPEED_FWD_POS 12
#define SPI_SONIC_COMMAND_MOTOR_SPEED_BRK_POS 13
#define SPI_SONIC_COMMAND_MOTOR_SPEED_EN_POS 14

#define SPI_DJANGO_COMMAND_KICK       0x01 //0x01, 0x??, 0x?? (16-bit value: time in ms)
#define SPI_DJANGO_COMMAND_CHIP       0x02 //0x02, 0x??, 0x?? (16-bit value: time in ms)
#define SPI_DJANGO_COMMAND_DRIBBLER   0x03 //0x03, 0x?? (8-bit value: dribbler PPM value)
#define SPI_DJANGO_COMMAND_CHARGE     0x04 //0x04, 0x??, 0x?? (16-bit value: voltage. 0x00 is OFF) 
#define SPI_DJANGO_COMMAND_BALLSENSOR 0x05 //0x05, 0x00, 0x00 (Read 8bit: direction, Read 8bit: distance)

#define MAX_RPM 2500.0f
#define MAX_ROTATIONAL_RPM 0.3 * MAX_RPM

#define SCK 14
#define MISO 12
#define MOSI 13

#define DUMMY 0x00

#define MODULE_DJANGO 1
#define MODULE_MC0 3
#define MODULE_MC1 4
#define MODULE_CHARGE 6
#define MODULE_SENSOR 7
#define MODULE_MOTHERBOARD 8

#define SPI_SPEED 10000000 //10MHz

double SPEED_DECREASE_FACTOR = 0.9; //0.99

#define UDP_PORT 10001
#define MULTICAST_ADDRESS "224.0.0.1"

#define WIFI_BUFFER_SIZE 255

#define MAX_ROTATION_SPEED 2 * 5.069265259

#define MOTOR1_DIRECTION_CALC_COS cos(130.0f * DEG_TO_RAD)
#define MOTOR1_DIRECTION_CALC_SIN sin(130.0f * DEG_TO_RAD)
#define MOTOR2_DIRECTION_CALC_COS cos(230.0f * DEG_TO_RAD)
#define MOTOR2_DIRECTION_CALC_SIN sin(230.0f * DEG_TO_RAD)
#define MOTOR3_DIRECTION_CALC_COS cos(310.0f * DEG_TO_RAD)
#define MOTOR3_DIRECTION_CALC_SIN sin(310.0f * DEG_TO_RAD)
#define MOTOR4_DIRECTION_CALC_COS cos(50.0f * DEG_TO_RAD)
#define MOTOR4_DIRECTION_CALC_SIN sin(50.0f * DEG_TO_RAD)

uint8_t updDataBuffer[WIFI_BUFFER_SIZE];
char formatBuffer[WIFI_BUFFER_SIZE];

float robotVelocityX = 0.0f;
float robotVelocityY = 0.0f;
float robotVelocityR = 0.0f;
//float robotChip = 0.0f;
//float robotKick = 0.0f;

WiFiUDP Udp;
SoftwareSerial swSer(-1, 2); //TX only

unsigned long nextMillis = 0;
uint32_t pktCnt = 0;

bool processWifi = true;
bool useHack = false;

void setup() {
  Serial.begin(115200);
  swSer.begin(9600);
  setupGPIO();
  setupSPI();


  /*while(1) {
    testMode();
    delay(10);
  }*/

  while(1) {
    djangoTest(0);
    delay(10);
  }
  
  setupWiFi();

  delay(1000);

  swSer.println("t0"); //Disable chargeboard
  
  Serial.println("This is robot #"+String(ROBOT_ID));

  Serial.println("Press button to switch to testmode!");
  bool goToTest = false;
  for (uint8_t i = 0; i<10; i++) {
    delay(100);
    Serial.print(".");
    if (digitalRead(0)==0) { goToTest = true; break; }
  }
  Serial.println();

  if (goToTest) testMode();
  
  swSer.println("t400"); //Enable chargeboard
}

#define SERIN_LEN 16
uint8_t serinPos = 0;
char serin[SERIN_LEN] = {0};

uint16_t motor0Value = 0;
uint16_t motor1Value = 0;
uint16_t motor2Value = 0;
uint16_t motor3Value = 0;

bool t = false;

void updateMotorValues() {
  float speedFactor = MAX_RPM * sqrtf( (robotVelocityX * robotVelocityX) + (robotVelocityY * robotVelocityY) );
  float velocityAngle = atan2(robotVelocityY, robotVelocityX) - robotVelocityR;
  float velocityAngleSin = sin(velocityAngle);
  float velocityAngleCos = cos(velocityAngle);
  
  float motor0Value = constrain((MOTOR1_DIRECTION_CALC_COS * velocityAngleCos + MOTOR1_DIRECTION_CALC_SIN * velocityAngleSin) * speedFactor + ((robotVelocityR / MAX_ROTATION_SPEED) * MAX_ROTATIONAL_RPM), -2500.0, 2500.0);
  float motor1Value = constrain((MOTOR2_DIRECTION_CALC_COS * velocityAngleCos + MOTOR2_DIRECTION_CALC_SIN * velocityAngleSin) * speedFactor + ((robotVelocityR / MAX_ROTATION_SPEED) * MAX_ROTATIONAL_RPM), -2500.0, 2500.0);
  float motor2Value = constrain((MOTOR3_DIRECTION_CALC_COS * velocityAngleCos + MOTOR3_DIRECTION_CALC_SIN * velocityAngleSin) * speedFactor + ((robotVelocityR / MAX_ROTATION_SPEED) * MAX_ROTATIONAL_RPM), -2500.0, 2500.0);
  float motor3Value = constrain((MOTOR4_DIRECTION_CALC_COS * velocityAngleCos + MOTOR4_DIRECTION_CALC_SIN * velocityAngleSin) * speedFactor + ((robotVelocityR / MAX_ROTATION_SPEED) * MAX_ROTATIONAL_RPM), -2500.0, 2500.0); 

  writeMotorValue(0x00, abs(motor0Value), motor0Value > 0, false);
  writeMotorValue(0x01, abs(motor1Value), motor1Value > 0, false);
  writeMotorValue(0x02, abs(motor2Value), motor2Value > 0, false);
  writeMotorValue(0x03, abs(motor3Value), motor3Value > 0, false);
}

void djangoTest(uint16_t d) {
  selectModule(MODULE_DJANGO, true);
  SPI.transfer(0x42);
  SPI.transfer(0x13);
  SPI.transfer(0x37);
  selectModule(MODULE_DJANGO, false);
}

void handleSerialCommands() {
  if (Serial.available()) {
    char in = Serial.read();
    if (in=='\n') {
      char cmd = serin[0];
      uint16_t value = atoi(serin+1);
      for( uint8_t p = 0; p<SERIN_LEN; p++) serin[p] = 0;
      if (cmd=='k') { swSer.println("k"+String(value)); Serial.println("Kick pulse "+String(value)+"ms"); }
      if (cmd=='c') { swSer.println("c"+String(value)); Serial.println("Chip pulse "+String(value)+"ms"); }
      if (cmd=='t') { swSer.println("t"+String(value)); Serial.println("Charge target "+String(value)+"v"); }
      if (cmd=='s') { swSer.println("s"+String(value)); Serial.println("Dribbler PPM value "+String(value)); }
      if (cmd=='x') { robotVelocityX = value/1000.0; Serial.println("VelocityX: "+String(robotVelocityX)); }
      if (cmd=='X') { robotVelocityX = -value/1000.0; Serial.println("VelocityX: "+String(robotVelocityX)); }
      if (cmd=='y') { robotVelocityY = value/1000.0; Serial.println("VelocityY: "+String(robotVelocityY)); }
      if (cmd=='Y') { robotVelocityY = -value/1000.0; Serial.println("VelocityY: "+String(robotVelocityY)); }
      if (cmd=='r') { robotVelocityR = value/1000.0; Serial.println("VelocityR: "+String(robotVelocityR)); }
      if (cmd=='R') { robotVelocityR = -value/1000.0; Serial.println("VelocityR: "+String(robotVelocityR)); }
      if (cmd=='W') { processWifi = value; Serial.println("WiFi: "+String(processWifi)); }
      if (cmd=='H') { useHack = value; Serial.println("TEST: "+String(useHack)); }      
      if (cmd=='d') { SPEED_DECREASE_FACTOR = value/1000.0; Serial.println("Speed decrease factor: "+String(SPEED_DECREASE_FACTOR)); }
      if (cmd=='q') {djangoTest(value);}
      //Serial.println("Set: "+String(value));
      //upd = true;
      serinPos = 0;
    } else {
      if (serinPos<(SERIN_LEN-1)) {
        serin[serinPos++] = in;
      }
    }
  }
}

void loop() {
  if (processWifi) readProtobuf();
  updateMotorValues();
  handleSerialCommands();

  if (millis()>nextMillis) {
    nextMillis = millis()+1000;
    Serial.println("Packets per second: "+String(pktCnt));
    pktCnt = 0;
  }

  robotVelocityX *= SPEED_DECREASE_FACTOR;
  robotVelocityY *= SPEED_DECREASE_FACTOR;
  robotVelocityR *= SPEED_DECREASE_FACTOR;
}

void testMode() {  
  for (uint8_t motor = 0; motor<4; motor++) {
    Serial.println("Test "+String(motor)+": motor "+String(motor)+" ON >");
    writeMotorValue(motor, 200, false, false);
    delay(1000);
    /*Serial.println("Test "+String(motor)+": motor "+String(motor)+" OFF");
    writeMotorValue(motor, 0, false, false);
    delay(100);
    Serial.println("Test "+String(motor)+": motor "+String(motor)+" BREAK");
    writeMotorValue(motor, 0, false, true);
    delay(100);
    Serial.println("Test "+String(motor)+": motor "+String(motor)+" ON <");
    writeMotorValue(motor, 200, true, false);
    delay(100);
    Serial.println("Test "+String(motor)+": motor "+String(motor)+" OFF");
    writeMotorValue(motor, 0, false, false);
    delay(100);*/
  }
  return;
  Serial.println("Test chargeboard: Charging...");
  swSer.println("t400");
  delay(8000);
  Serial.println("Test chargeboard: Kick (1ms)");
  swSer.println("k1");
  delay(8000);
  Serial.println("Test chargeboard: Chip (1ms)");
  swSer.println("c1");
  delay(8000);
  Serial.println("Test chargeboard: Kick (5ms)");
  swSer.println("k5");
  delay(8000);
  Serial.println("Test chargeboard: Chip (5ms)");
  swSer.println("c5");
  delay(8000);
  Serial.println("Test dribbler: ON");
  swSer.println("s60");
  delay(5000);
  Serial.println("Test dribbler: OFF");
  swSer.println("s50");
  Serial.println("---TESTS COMPLETED---");

  Serial.println("Please reset the CPU.");
  while (true) delay(1000); //Wait forever.
}


void setupGPIO() {
  pinMode( 0, INPUT); //Button
  pinMode( 2, OUTPUT); //LED
  pinMode( 4, OUTPUT); //SS_SEL_A
  pinMode( 5, OUTPUT); //SS_SEL_B
  pinMode(15, OUTPUT); //SS_SEL_C
  pinMode(16, OUTPUT); //SS_DISABLE
  digitalWrite(16, HIGH); //Turn SS off
  digitalWrite(2, HIGH);   //Turn LED off
  digitalWrite(SCK, LOW);
}

void setupWiFi() {
  Serial.print("Connecting to: '");
  Serial.print(NETWORK_SSID);
  Serial.print("'");
  // Connect to the RoboCup network
  WiFi.begin(NETWORK_SSID, NETWORK_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println(" Connected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  Serial.print("Setting up UDP server... ");
  Udp.begin(UDP_PORT);
  Serial.println("OK");
}

void setupAP() {
  uint8_t mac[WL_MAC_ADDR_LENGTH];
  
  WiFi.mode(WIFI_AP);
  WiFi.softAPmacAddress(mac);
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
  
  Udp.begin(UDP_PORT);
  Serial.println("Finished setting up access point mode, started udp server.");
}

void setupSPI() {  
  // Start SPI
  SPI.begin();
  // Setting SPI frequency
  SPI.setFrequency(SPI_SPEED);
}

void selectModule(const uint8_t &module, bool state) {
  if (state) {
    bool a = 0;
    bool b = 0;
    bool c = 0;
    switch(module) {
      case 1: //Y5 (101)
        a = 1;
        c = 1;
        break;
      case 2: //Y0 (000)
        break;
      case 3: //Y6 (110)
        b = 1;
        c = 1;
        break;
      case 4: //Y1 (001)
        a = 1;
        break;
      case 5: //Y4 (100)
        c = 1;
        break;
      case 6: //Y2 (010)
        b = 1;
        break;
      case 7: //Y3 (011)
        a = 1;
        b = 1;
        break;
      default: //This is the "module" on the motherboard
      case 8: //Y7 (111)
        a = 1;
        b = 1;
        c = 1;
        break;
    }
    digitalWrite( 4, a);
    digitalWrite( 5, b);
    digitalWrite(15, c);
    digitalWrite(16, false);
  } else {
    digitalWrite(16, true);
  }
}

void sendCommand16(const uint8_t &module, const uint8_t &command, const uint16_t &data) {
  selectModule(module, true);
  // Sending command
  SPI.transfer(command);
  // Sending high byte
  uint16_t data1 = SPI.transfer(data >> 8);
  // Sending low byte
  uint16_t data2 = SPI.transfer(data & 0xFF);
  delayMicroseconds(50);
  selectModule(module, false);
}

#define SPI_SONIC_NEW_MOTOR0 0
#define SPI_SONIC_NEW_MOTOR1 1
#define SPI_SONIC_NEW_CMD_RPM 0x01
#define SPI_SONIC_NEW_CMD_BRAKE 0x05
#define SPI_SONIC_NEW_CMD_ENABLE 0x04

void sendSonicCommand(uint8_t module, uint8_t command0, uint8_t command1, uint64_t data) {
  selectModule(module, true);
  SPI.transfer(command0);
  SPI.transfer(command1);
  selectModule(module, false);
  delay(50);
  selectModule(module, true);  
  SPI.transfer((data&0xFF000000)>>(8*3));
  SPI.transfer((data&0x00FF0000)>>(8*2));
  SPI.transfer((data&0x0000FF00)>>(8*1));
  SPI.transfer((data&0x000000FF)>>(8*0));
  selectModule(module, false);
}

void setMotorRPM(uint8_t module, uint8_t motorid, uint64_t rpm) {
  sendSonicCommand(module, SPI_SONIC_NEW_CMD_RPM, motorid, rpm);
}

void setMotorBrake(uint8_t module, uint8_t motorid, uint64_t brake) {
  sendSonicCommand(module, SPI_SONIC_NEW_CMD_BRAKE, motorid, brake);
}

void setMotorEnable(uint8_t module, uint8_t motorid, uint64_t enable) {
  sendSonicCommand(module, SPI_SONIC_NEW_CMD_ENABLE, motorid, enable);
}

/*void writeMotorValue(uint8_t motor, uint16_t value, bool forward, bool brake){
  uint8_t module = 0x00;
  uint8_t motorid = 0x00;
    
  switch(motor) {
    case 0x00:
      module = MODULE_MC0;
      motorid = SPI_SONIC_NEW_MOTOR0;  
      break;

    case 0x01:
      module = MODULE_MC1;
      motorid = SPI_SONIC_NEW_MOTOR1;
      break;

    case 0x02:
      module = MODULE_MC1;
      motorid = SPI_SONIC_NEW_MOTOR0;
      break;

    case 0x03:
      module = MODULE_MC0;
      motorid = SPI_SONIC_NEW_MOTOR1;
      break;

    default:
      Serial.println("Wrong motor index: "+String(motor));
      return;
  }

  setMotorRPM(module, motorid, value);
  delay(50);
  setMotorBrake(module, motorid, false);
  delay(50);  
  setMotorEnable(module, motorid, true);
  delay(50);
}*/


void writeMotorValue(uint8_t motor, uint16_t value, bool forward, bool brake){
  //if (value>0) Serial.println("writeMotorValue: "+String(motor)+", "+String(value)+", "+String(forward)+", "+String(brake));
  uint16_t data = (value & 0x7FF) 
                  | ((forward & 0x01) << SPI_SONIC_COMMAND_MOTOR_SPEED_FWD_POS) 
                  | ((brake & 0x01) << SPI_SONIC_COMMAND_MOTOR_SPEED_BRK_POS)
                  | (0x01 << SPI_SONIC_COMMAND_MOTOR_SPEED_EN_POS);
  uint8_t module = 0x00;
  uint8_t command = 0x00;
  
  switch(motor) {
    case 0x00:
      module = MODULE_MC0;
      command = SPI_SONIC_COMMAND_MOTOR0_SPEED;  
      break;

    case 0x01:
      module = MODULE_MC1;
      command = SPI_SONIC_COMMAND_MOTOR1_SPEED;
      break;

    case 0x02:
      module = MODULE_MC1;
      command = SPI_SONIC_COMMAND_MOTOR0_SPEED;
      break;

    case 0x03:
      module = MODULE_MC0;
      command = SPI_SONIC_COMMAND_MOTOR1_SPEED;
      break;

    default:
      Serial.println("Wrong motor index: "+String(motor));
  }

  if (module && command)
    sendCommand16(module, command, data);
}

uint8_t oldDribblerInt = 0;
uint8_t maybeSendAgain = 0;
void setDribbler(float dribblerValue) {
  uint8_t dribblerValueInt = 55;
  if (dribblerValue>=0) {
    dribblerValue *= 5.0;//35.0; //Multiply
    dribblerValueInt += dribblerValue;
  }
  if ((dribblerValueInt!=oldDribblerInt)||(maybeSendAgain>200)) {
    oldDribblerInt = dribblerValueInt;
    swSer.println("s"+String(dribblerValueInt));
    Serial.println("Dribbler: "+String(dribblerValueInt));
    maybeSendAgain = 0;
  }
  maybeSendAgain++;
}

bool ignoreKick = false;
bool ignoreChip = false;

void readProtobuf() {
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    int len = Udp.read(updDataBuffer, 255);
    if (len) {
      pb_istream_t inputStream = pb_istream_from_buffer((const pb_byte_t*) updDataBuffer, len); 
      Command cmd = Command_init_zero;
      
      if (pb_decode(&inputStream, Command_fields, &cmd)) {
        //Serial.println(String(cmd.id)+", "+String(cmd.move.x)+", "+String(cmd.move.y)+", "+String(cmd.move.r)+", "+String(cmd.action.dribble)+", "+String(cmd.action.kick)+", "+String(cmd.action.chip));
        // Check if packet was meant for the robot
        //Serial.println("Pkt for robot "+String(radioProtocolCommand.robot_id));
        if (cmd.id == ROBOT_ID) {
          robotVelocityX = -cmd.move.y;
          robotVelocityY = -cmd.move.x;
          robotVelocityR = cmd.move.r;
          if (useHack) {
            SPEED_DECREASE_FACTOR = 0.5 + cmd.action.dribble*0.5;
            Serial.println("HACK FOR SPEED DECREASE FACTOR: "+String(SPEED_DECREASE_FACTOR));
          } else {
            setDribbler(cmd.action.dribble);
          }
          if (cmd.action.kick!=0.0) {
            if (!ignoreKick) {
              Serial.println("Kick: "+String(cmd.action.kick));
              swSer.println("k3");
            } else {
              Serial.println("Ignored repeated kick.");
            }
            ignoreKick = true;
          }else if (cmd.action.chip!=0.0) {
            if (!ignoreChip) {
              Serial.println("Chip: "+String(cmd.action.chip));
              swSer.println("c3");
            } else {
              Serial.println("Ignored repeated chip.");
            }
            ignoreChip = true;
          } else {
            ignoreKick = false;
            ignoreChip = false;
          }

          pktCnt++;
          // Call on command receive callback
          //onCommandReceive(radioProtocolCommand);
          //Serial.println("Received command for me!");
        } else {
          //Serial.println("Received command for robot "+String(cmd.id));
        }
        return;
      } else {
        Serial.println("Could not decode as protobuf3.");
      }

      //Try protobuf 2
      /*RadioProtocolCommand radioProtocolCommand = RadioProtocolCommand_init_zero;
      if (pb_decode(&inputStream, RadioProtocolCommand_fields, &radioProtocolCommand)) {
        if (radioProtocolCommand.robot_id == ROBOT_ID) {
          onCommandReceive(radioProtocolCommand);
        } else {
          Serial.println("Received PB2 command for robot "+String(cmd.id));
        }
      } else {
        Serial.println("Could not decode as protobuf2.");
      }*/
    }
  }
}

/*bool decodeRadioProtocolCommandCallback(pb_istream_t *radioProtocolStream, const pb_field_t *field, void **arg) {
  // Allocate space for the decoded message.
  RadioProtocolCommand radioProtocolCommand = RadioProtocolCommand_init_zero;
  // Decode the given RadioProtocolCommand.
  if (pb_decode(radioProtocolStream, RadioProtocolCommand_fields, &radioProtocolCommand)) {
    // Check if packet was meant for the robot
    if (radioProtocolCommand.robot_id == ROBOT_ID) {
      // Call on command receive callback
      onCommandReceive(radioProtocolCommand);
    }
    // Decoding completed
    return true;
  }
  // Decoding failed
  return false;
}*/
/*
void onCommandReceive(RadioProtocolCommand radioProtocolCommand) {
  robotVelocityX = radioProtocolCommand.velocity_y;
  robotVelocityY = radioProtocolCommand.velocity_x;
  robotVelocityR = radioProtocolCommand.velocity_r;

  float dribblerValue = radioProtocolCommand.dribbler_spin;
  uint8_t dribblerValueInt = 55;
  if (dribblerValue>=0) {
    dribblerValue *= 35.0; //Multiply
    dribblerValueInt += dribblerValue;
  }

  //Serial.println("DribblerValueInt: "+String(dribblerValueInt));
  //myservo.write(dribblerValueInt);
  if (dribblerValueInt!=oldDribblerInt) {
    oldDribblerInt = dribblerValueInt;
    swSer.println("s"+String(dribblerValueInt));
    Serial.println("Dribbler: "+String(dribblerValueInt));
  }
}*/
