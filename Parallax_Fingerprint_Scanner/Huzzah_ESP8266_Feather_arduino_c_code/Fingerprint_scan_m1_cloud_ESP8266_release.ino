/***************************************************************************
  This is an example of program for connected the Adafruit Huzzah and Parallax
  FingerPrint Scan Module
  to the Medium One Prototyping Sandbox.  Visit www.medium.one for more information.
  Author: Medium One
  Last Revision Date: Aug 1, 2018

  The program includes a library and portions of sample code from Adafruit
  with their description below:
  
  This is a library for the BMP280 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BMEP280 Breakout 
  ----> http://www.adafruit.com/products/2651

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required 
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/


 
/*-------------------------------------------------------------------------*
 *          Includes: 
 *-------------------------------------------------------------------------*/
 
#include <PubSubClient.h>
#include <ESP8266WiFi.h>  // for ESP8266
#include <WiFiClientSecure.h>
#include <SoftwareSerial.h>  // for UART interface with the FingerPrint Scanner Module.
/*-------------------------------------------------------------------------*
 * Constants: (mqtt + LED)
 *-------------------------------------------------------------------------*/
// ongoing timer counter for heartbeat
static int heartbeat_timer = 0;

// ongoing timer counter for sensor
static int sensor_timer = 0;

// set heartbeat period in milliseconds
static int heartbeat_period = 60000;

// set sensor transmit period in milliseconds
static int sensor_period = 5000;

// track time when last connection error occurs
long lastReconnectAttempt = 0;

// set pin for LED
const int BLULED_PIN = 2;            //(Blue led)
const int REDLED_PIN = LED_BUILTIN;  //(RED led)

const int LED_ON = LOW;   // hardware setup for ESP8266
const int LED_OFF = HIGH;

/*********************************************************************************
 * Constants (For the FingerPrint Scanner Module)
 *********************************************************************************/

#define PASS      1
#define FAIL      0
 
#define CMD_LEN   8

#define CMDFRAMESTART_POS    0  // Command Type (Command)
#define CMDTYPE_POS     1  // Command Frame Byte
#define CMDPAR1_POS     2  // Command Parameter 1
#define CMDPAR2_POS     3  // Command Parameter 2
#define CMDPAR3_POS     4  // Command Parameter 3
#define CMDBYTE6        5  // Command byte 6
#define CMDCHKSUM_POS   6  // Command Checksum
#define CMDFRAMEEND_POS 7  // Command Frame Byte

#define RSP_LEN   8

#define RESFRAMESTART_POS 0  // Response Frame Byte
#define RESTYPE_POS     1  // Response Type (Command)
#define RESPAR1_POS     2  // Response Parameter 1
#define RESPAR2_POS     3  // Response Parameter 2
#define RESPAR3_POS     4  // Response Parameter 3
#define RESBYTE6        5  // Response byte 6
#define RESCHKSUM_POS   6  // Response Checksum
#define RESFRAMEEND_POS 7  // Response Frame Byte

#define IDDATA_LEN    199  // Eigenvalue data length + 3 bytes(F5 start, Checksum, F5 end)

#define IDDATAFRAMESTART_POS      0   // Response Frame Byte
#define IDDATAFSTBYTE             1
#define IDDATALASTBYTE            196
#define IDDATACHKSUM_POS          197
#define IDDATAFRAMEEND_POS        198  // Response Frame Byte


//#define IDDATASEND_LEN    202  // Eigenvalue data length + 6 bytes(F5 start, user id hi, user id low, user priv, Checksum, F5 end)

//#define IDDATASENDFRAMESTART_POS      0   // Response Frame Byte
//#define IDDATASENDUSERHI              1
//#define IDDATASENDUSERLO              2
//#define IDDATASENDUSERPRI             3
//#define IDDATASENDFSTBYTE             4
//#define IDDATASENDLASTBYTE            199
//#define IDDATASENDCHKSUM_POS          200
//#define IDDATASENDFRAMEEND_POS        201  // Response Frame Byte

/* 
 *  FingerPrint Response status
 */
#define ACK_SUCCESS                       0x00 // Operation successfully
#define ACK_FAIL                          0x01 // Operation failed
#define ACK_FULL                          0x04 // Fingerprint database is full
#define ACK_NOUSER                        0x05 // No such user
#define ACK_USER_EXISTS                   0x07 // already exists
#define ACK_TIMEOUT                       0x08 // Acquisition timeout

/* 
 *  FingerPrint Command
 */
#define CMD_SLEEP                         0x2C // Sleeps the device
#define CMD_SET_MODE                      0x2D

#define NUMSCAN                           3    // scan 3 times
#define CMD_ADD_FINGERPRINT_1             0x01
#define CMD_ADD_FINGERPRINT_2             0x02
#define CMD_ADD_FINGERPRINT_3             0x03

#define CMD_DELETE_USER                   0x04
#define CMD_DELETE_ALL_USERS              0x05
#define CMD_GET_USERS_COUNT               0x09
#define CMD_SCAN_COMPARE_1_TO_1           0x0B
#define CMD_SCAN_COMPARE_1_TO_N           0x0C
#define CMD_READ_USER_PRIVLAGE            0x0A
#define CMD_SENSITIVITY                   0x28
#define CMD_SCAN_GET_IMAGE                0x24
#define CMD_SCAN_GET_EIGENVALS            0x23
#define CMD_SCAN_PUT_EIGENVALS            0x44
#define CMD_PUT_EIGENVALS_COMPARE_1_TO_1  0x42
#define CMD_PUT_EIGENVALS_COMPARE_1_TO_N  0x43
#define CMD_GET_USER_EIGENVALS            0x31
#define CMD_PUT_USER_EIGENVALS            0x41
#define CMD_GET_USERS_INFO                0x2B
#define CMD_SET_SCAN_TIMEOUT              0x2E // How long to try scanning - multiples of ~0.25s

#define FRAME_BYTE                        0xF5

/*-------------------------------------------------------------------------*
 * Globals: (WiFi Secruity Credential related)
 *-------------------------------------------------------------------------*/
// wifi client with security
WiFiClientSecure wifiClient; 

// MQTT Connection info

char pub_topic[]="0/<Project MQTT ID>/<User MQTT>/esp8266/";
char sub_topic[]="1/<Project MQTT ID>/<User MQTT>/esp8266/event";
char mqtt_username[]="<Project MQTT ID>/<User MQTT>";
char mqtt_password[]="<API Key>/<User Password>";

char server[] = "mqtt.mediumone.com";
int port = 61620;

char WIFI_SSID[]      = "<WIFI_SSID>";
char WIFI_PASSWORD[]  = "<WIFI_PaSSWORD>";

/*********************************************************************************
 * Globals / Class Instantiation (For the FingerPrint Scanner Module)
 *********************************************************************************/
SoftwareSerial FPSerial(14, 12);  // RX | TX (ESP8266)
int            FP_baud = 19200;   // full at mode when en pin is tied to 3.3V

////////////////////////////////
//   FingerPrint Scan Variables
////////////////////////////////
bool g_new_command_flag = false;
int  g_user_command;

/*********************************************************************************
 * Subroutine Prototypes
 *********************************************************************************/
void FingerPrint_Send_Commmand(char command, char param1,char param2, char param3);
int  FingerPrint_Get_Response(char *command,char *param1,char *param2,char *param3);
void FingerPrint_Print_Response(char command, char param1,char param2, char param3);

char FingerPrint_add_new_user (byte userid, byte userpriv);
char FingerPrint_delete_all_users (void);
char FingerPrint_compare_N_users (void);

int FingerPrint_Get_Response_And_Data(char *command,char *param1,char *param2,char *param3, char *data_str_ptr);
int FingerPrint_Get_Data_Packet(char *command, char *array_ptr, int len);
char FingerPrint_Print_and_Checksum (char *data_str, int len);
void FingerPrint_Send_Commmand_And_Data(char command, char userid,char userpriv, char *data_str_ptr);

/*-------------------------------------------------------------------------*
 * Main Program Start: (Setup)
 *-------------------------------------------------------------------------*/
void setup (){
  
  // initialize serial communication
  Serial.begin(115200);     // Serial UART to the PC's USB port (COM).
  while(!Serial){}
  Serial.setTimeout(60000);  // 60 sec
  FPSerial.begin(FP_baud);  // Serial Soft UART to the fingerprint scanner UART port. 

  // wifi setup
  WiFi.mode(WIFI_STA);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);   // new from the new release

  //not sure this is needed
  delay(5000);
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("Failed to connect, resetting"));
    //ESP.reset();  // not supported by esp32 WiFi library
  }

  // optinally
  //while (WiFi.status() != WL_CONNECTED) {}
  
  // if you get here you have connected to the WiFi
  Serial.println(F("Connected to Wifi!"));

  Serial.println(F("Init hardware LED"));
  pinMode(BLULED_PIN, OUTPUT);
  pinMode(REDLED_PIN, OUTPUT);
  digitalWrite(BLULED_PIN, LED_OFF);  // turn off
  digitalWrite(REDLED_PIN, LED_OFF);  // turn off
  
  // connect to MQTT broker to send board reset msg
  connectMQTT();

  Serial.println("Complete Setup");
}

/*-------------------------------------------------------------------------*
 *  A callback function used by Wifi Client to pass the recived message from
 *  the MQTT broker to the MQTT client.
 *
 *  For this example, only 1 value (0 or 1) is expected and is used to turn
 *  on or off the LED light.
 *
 *  input:  topic (MQTT topic string)
 *          payload (MQTT message payload string)
 *          length  (MQTT message payload string's length)
 *  Return : None
 *  
 *-------------------------------------------------------------------------*/ 
void callback(char* topic, byte* payload, unsigned int length) {
  // handle message arrived
  int i = 0;
  char message_buff[length + 1];
  int led;     // which led to select (1 = red, 2 = green)
  int level;   // led level (1 = on, 0 = off)

  for(i=0; i < length; i++) {
    message_buff[i] = payload[i];
  }
  message_buff[i] = '\0';

  Serial.println(F("This is WiFi Task."));
  Serial.print(F("Received some data from the MQTT broker: "));
  Serial.println(String(message_buff));

  switch (message_buff[0]) {
     case 'G':   //G<led>:<level>        
       // Process message to turn LED on and off          
       if (message_buff[1] == '2') {  // blue led        
          Serial.print(F("BLUE LED command from M1."));
          if (message_buff[3] == '1') { 
           // Turn off LED
           digitalWrite(BLULED_PIN, LED_ON);
          } else if (message_buff[3] == '0') { 
           // Turn on LED
           digitalWrite(BLULED_PIN, LED_OFF);
          }
       } else if (message_buff[1] == '1') {  // red led
          Serial.print(F("RED LED command from M1."));                         
          if (message_buff[3] == '1') { 
           // Turn off LED
           digitalWrite(REDLED_PIN, LED_ON);
          } else if (message_buff[3] == '0') { 
           // Turn on LED
           digitalWrite(REDLED_PIN, LED_OFF);
          }
       }
       break;
     case 'C':   //C<1,2,3>  1 - add, 2 - delete, 3 - verify
        Serial.print(F("Command from M1."));         
        if (message_buff[1] == '1') { 
           g_new_command_flag = true;
           g_user_command  = 1;            
        } else if (message_buff[1] == '2') { 
           g_new_command_flag = true;
           g_user_command  = 2;            
        } else if (message_buff[1] == '3') { 
           g_new_command_flag = true;
           g_user_command  = 3;            
        }       
        break;      
     default:
        Serial.print(F("Unsupported Command."));         
        break;
  }
}

PubSubClient client(server, port, callback, wifiClient);

boolean connectMQTT()
{    
  // Important Note: MQTT requires a unique id (UUID), we are using the mqtt_username as the unique ID
  // Besure to create a new device ID if you are deploying multiple devices.
  // Learn more about Medium One's self regisration option on docs.mediumone.com
  if (client.connect((char*) mqtt_username,(char*) mqtt_username, (char*) mqtt_password)) {
    Serial.println(F("Connected to MQTT broker"));

    // send a connect message
    if (client.publish((char*) pub_topic, "{\"event_data\":{\"mqtt_connected\":true}}")) {
      Serial.println("Publish connected message ok");
    } else {
      Serial.print(F("Publish connected message failed: "));
      Serial.println(String(client.state()));
    }

    // subscrive to MQTT topic
    if (client.subscribe((char *)sub_topic,1)){
      Serial.println(F("Successfully subscribed"));
    } else {
      Serial.print(F("Subscribed failed: "));
      Serial.println(String(client.state()));
    }
  } else {
    Serial.println(F("MQTT connect failed"));
    Serial.println(F("Will reset and try again..."));
    abort();
  }
  return client.connected();
}

/*-------------------------------------------------------------------------*
 * Main Program Start: (loop [task])
 *-------------------------------------------------------------------------*/
void fingerprint_loop();

void loop() {
  if (!client.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 1000) {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      if (connectMQTT()) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    // Client connected
    client.loop();
  }
  heartbeat_loop();
  fingerprint_loop();
}

void fingerprint_loop() {

  char returnstatus;
  
  if (g_new_command_flag) {   // Recieved new command from the cloud.
    g_new_command_flag = false;
    digitalWrite(REDLED_PIN, LED_OFF);
    digitalWrite(BLULED_PIN, LED_OFF);
    delay (200);    
    digitalWrite(BLULED_PIN, LED_ON);
    delay (200);    
    switch(g_user_command) {
      case 1 : 
        Serial.println (F("Add a new user."));
        returnstatus = FingerPrint_add_new_user ((char)1,(char)1);
        break;    
      case 2 : 
        Serial.println (F("Delete a new user."));
        returnstatus = FingerPrint_delete_all_users ();
        break;    
      case 3 : 
        Serial.println (F("Verify a new user."));
        returnstatus = FingerPrint_compare_N_users ();
        break;    
      default:
        break;
    }
    delay (3000);    
    digitalWrite(BLULED_PIN, LED_OFF);
    Serial.println (F("Finished."));
    if (returnstatus == FAIL) {
      Serial.println (F("Error: The Operation failed."));
      digitalWrite(REDLED_PIN, LED_ON);  // turn on red led.
    }

    String payload = "{\"event_data\":{\"operation_status\":";    
    if (returnstatus == FAIL) {
        payload += "\"fail\"}}";
    } else {
        payload += "\"pass\"}}";
    }
    
    if (client.loop()){
      Serial.print(F("Sending heartbeat: "));
      Serial.println(payload);
  
      if (client.publish((char *) pub_topic, (char*) payload.c_str()) ) {
        Serial.println(F("Publish ok"));
      } else {
        Serial.print(F("Failed to publish operation status: "));
        Serial.println(String(client.state()));
      }
    }  
    delay(20);
  }
}

void heartbeat_loop() {
  if ((millis()- heartbeat_timer) > heartbeat_period) {
    heartbeat_timer = millis();
    String payload = "{\"event_data\":{\"millis\":";
    payload += millis();
    payload += ",\"heartbeat\":true}}";
    
    if (client.loop()){
      Serial.print(F("Sending heartbeat: "));
      Serial.println(payload);
  
      if (client.publish((char *) pub_topic, (char*) payload.c_str()) ) {
        Serial.println(F("Publish ok"));
      } else {
        Serial.print(F("Failed to publish heartbeat: "));
        Serial.println(String(client.state()));
      }
    }
  }
}
/*********************************************************************************
 * Subroutine Body (For the Finger Print Scan Module)
 *********************************************************************************/

char FingerPrint_add_new_user (char userid, char userpriv) {
  int iteration;
  char command;
  char param1, param2, param3;
  char op_status = PASS;
  
  for (iteration = 0; iteration < NUMSCAN; iteration++) {
    switch (iteration){
      case 0 :
        Serial.println("CMD : CMD_ADD_FINGERPRINT_1.");
        FingerPrint_Send_Commmand(CMD_ADD_FINGERPRINT_1, 0x00, userid, userpriv); 
        break;
      case 1 :
        Serial.println("CMD : CMD_ADD_FINGERPRINT_2.");
        FingerPrint_Send_Commmand(CMD_ADD_FINGERPRINT_2, 0x00, userid, userpriv); 
        break;
      case 2 :
        Serial.println("CMD : CMD_ADD_FINGERPRINT_3.");
        FingerPrint_Send_Commmand(CMD_ADD_FINGERPRINT_3, 0x00, userid, userpriv); 
        break;
  }
  
  if (PASS == FingerPrint_Get_Response(&command, &param1, &param2, &param3)) {
    FingerPrint_Print_Response(command,param1,param2,param3);
    if (param3 == ACK_SUCCESS)
      Serial.println("Response : ACK_SUCCESS.");
    else if (param3 == ACK_FAIL) 
      Serial.println("Response : ACK_FAIL.");
    else if (param3 == ACK_FULL) 
      Serial.println("Response : ACK_FULL.");
    else if (param3 == ACK_TIMEOUT) 
      Serial.println("Response : ACK_TIMEOUT.");
    else
      Serial.println("Response : Unknown ACK.");
    if (param3 != ACK_SUCCESS)
       op_status = FAIL;
  } else {
    FingerPrint_Print_Response(command,param1,param2,param3);
    Serial.println("Response : Checksum Error.");
    op_status = FAIL;
  }
  Serial.println(" ");
  delay (50); // Need some delay for esp8266 background processing.    
  delay (1000); // Need some delay for esp8266 background processing.    
 }
 return op_status;
}


char FingerPrint_compare_N_users (void) {
  char command;
  char param1, param2, param3;
  char op_status = PASS;

  FingerPrint_Send_Commmand(CMD_SCAN_COMPARE_1_TO_N, 0x00, 0x00, 0x00); 
  if (PASS == FingerPrint_Get_Response(&command, &param1, &param2, &param3)) {
    FingerPrint_Print_Response(command,param1,param2,param3);
    if ((param3 == ACK_NOUSER) || (param3 == ACK_TIMEOUT))
      op_status = FAIL;
    if (param3 >= 1 && param3 <=3) {
        Serial.print("Response : ACK_SUCCESS.  [user privilege is ");
        Serial.print(param3,DEC);
        Serial.println("]");
    } else if (param3 == ACK_NOUSER) 
        Serial.println("Response : ACK_NOUSER.");
      else if (param3 == ACK_TIMEOUT) 
        Serial.println("Response : ACK_TIMEOUT.");
      else
        Serial.println("Response : Unknown ACK.");
   } else {
      FingerPrint_Print_Response(command,param1,param2,param3);
      Serial.println("Response : Checksum Error.");
      op_status = FAIL;
  }
  Serial.println(" ");
  return op_status;
}



char FingerPrint_delete_all_users (void) {

  char command, param1, param2, param3;
  char op_status = PASS;

  Serial.println("CMD : CMD_DELETE_ALL_USERS.");
  FingerPrint_Send_Commmand(CMD_DELETE_ALL_USERS, 0x00, 0x00, 0x00); 
  if (PASS == FingerPrint_Get_Response(&command, &param1, &param2, &param3)) {
    FingerPrint_Print_Response(command,param1,param2,param3);
    if (param3 != ACK_SUCCESS)
       op_status = FAIL;
    if (param3 == ACK_SUCCESS)
      Serial.println("Response : ACK_SUCCESS.");
    else 
      Serial.println("Response : ACK_FAIL.");
  } else {
    FingerPrint_Print_Response(command,param1,param2,param3);
    Serial.println("Response : Checksum Error.");
    op_status = FAIL;
  }
  Serial.println(" ");
  return op_status;  
}

/*
 *  Description : Send a command to the finger print module
 *  
 *  Input: Command and parameters (CMD, P1, P2, P3)
 *  Ouput: None
 * 
 */
 
void FingerPrint_Send_Commmand(char command, char param1,char param2, char param3){
  char command_str[CMD_LEN];
  int  value;
  char checksum;


  command_str[CMDFRAMESTART_POS] = FRAME_BYTE;  // byte 1
  command_str[CMDTYPE_POS] = command;
  command_str[CMDPAR1_POS] = param1;
  command_str[CMDPAR2_POS] = param2;
  command_str[CMDPAR3_POS] = param3;
  command_str[CMDBYTE6]    = 0;         // always zero
  command_str[CMDCHKSUM_POS] = command_str[1] ^ command_str[2] ^ command_str[3] ^ command_str[4] ^ command_str[5];
  command_str[CMDFRAMEEND_POS] = FRAME_BYTE;  // byte 8

  // Send the command to the device.
  for (int i=0; i<CMD_LEN; i++) {
    FPSerial.print(command_str[i]);  
  }    
  
  Serial.print("Send Command : ");
  checksum = FingerPrint_Print_and_Checksum(command_str, CMD_LEN);
}

/*
 *  Description : Send a command and id data to the finger print module
 *  
 *  Input: Command and parameters (CMD, P1, P2, P3) and id data 
 *  Ouput: None
 * 
 */
void FingerPrint_Send_Commmand_And_Data(char command, char userid,char userpriv, char *data_str_ptr){
  char command_str[CMD_LEN];
  int  value;
  char checksum;
  char datasend_str[IDDATA_LEN];

  command_str[CMDFRAMESTART_POS]  = FRAME_BYTE;  // byte 1
  command_str[CMDTYPE_POS]        = command;
  command_str[CMDPAR1_POS]        = 0;       // length Hi
  command_str[CMDPAR2_POS]        = 0xc4;    // length Lo
  command_str[CMDPAR3_POS]        = 0;       // Don't care
  command_str[CMDBYTE6]           = 0;       // always zero
  command_str[CMDCHKSUM_POS]      = command_str[1] ^ command_str[2] ^ command_str[3] ^ command_str[4] ^ command_str[5];
  command_str[CMDFRAMEEND_POS]    = FRAME_BYTE;  // byte 8

  checksum = 0;
  for (int i=0; i < IDDATA_LEN ; i++){
    if (i == 1 ) 
      datasend_str[i] = 0;  // user id hi
    if (i == 2)
      datasend_str[i] = userid;  // user id lo
    if (i == 3)
      datasend_str[i] = userpriv;  // user priv
    if (i > 3 && i <  IDDATACHKSUM_POS)
      datasend_str[i] = i;
      //datasend_str[i] = data_str_ptr[i];        
    if (i > 0 && i < IDDATACHKSUM_POS)
       checksum = checksum ^ datasend_str[i];        
  }
  datasend_str[0]                 = FRAME_BYTE;
  datasend_str[IDDATACHKSUM_POS]  = checksum;
  datasend_str[IDDATA_LEN-1]      = FRAME_BYTE;

  Serial.print("Send Command : ");
  checksum = FingerPrint_Print_and_Checksum(command_str, CMD_LEN);

  Serial.println("Send Data : ");
  checksum = FingerPrint_Print_and_Checksum(datasend_str, IDDATA_LEN);

  // Send the command to the device.
  for (int i=0; i<CMD_LEN; i++) {
    FPSerial.print(command_str[i]);  
  }    

  // Send the command to the device.
  for (int i=0; i<IDDATA_LEN; i++) {
    FPSerial.print(datasend_str[i]);  
  }    
  
}
/*
 *  Description : Get a response from the finger print module
 *   
 *  Input: None
 *  Output: Command (Response Type) and parameters (CMD, Q1, Q2, Q3)
 * 
 */
int FingerPrint_Get_Response(char *command,char *param1,char *param2,char *param3){
  char response_str[RSP_LEN];
  char checksum;
  int  value;

  Serial.println("Waiting for response");
  for (int i=0; i<RSP_LEN; i++) {
    while (!FPSerial.available())  {
      //yield();    // esp8266
      delay (1);  // esp8266
      // loop forever.      
    }    
    response_str[i] = FPSerial.read();             
    
  }
  Serial.print("Get Response : ");
  checksum = FingerPrint_Print_and_Checksum(response_str, RSP_LEN);

  *command =  response_str[RESTYPE_POS]; 
  *param1  =  response_str[RESPAR1_POS]; 
  *param2  =  response_str[RESPAR2_POS]; 
  *param3  =  response_str[RESPAR3_POS]; 


  if (response_str[RESCHKSUM_POS] == checksum)
      return (1); // pass status
  else
      return (0); // fail status    
}
/*
 *  Description : Get a response and a data packet from the finger print module.
 *  
 *  Input: None
 *  Output: Command (Response Type) and parameters (CMD, Q1, Q2, Q3) and data array
 * 
 */
int FingerPrint_Get_Response_And_Data(char *command,char *param1,char *param2,char *param3, char *data_str_ptr){
  char response_str[RSP_LEN];
  char checksum_resp, checksum_data;
  int value;           // for print purpose
  int datalength;      // data length excluding F5 and CHK and F5 (3 bytes)

  //
  Serial.println("Waiting for response and data");
  for (int i=0; i<RSP_LEN; i++) {
    while (!FPSerial.available())  {
      //yield();    // esp8266
      delay (1);  // esp8266
      // loop forever.      
    }    
    response_str[i] = FPSerial.read();             
    
  }
  for (int i=0; i<IDDATA_LEN; i++) {
    while (!FPSerial.available())  {
      //yield();    // esp8266
      delay (1);  // esp8266
      // loop forever.      
    }    
    *(data_str_ptr+i) = FPSerial.read();                
  }

  *command =  response_str[RESTYPE_POS]; 
  *param1  =  response_str[RESPAR1_POS]; 
  *param2  =  response_str[RESPAR2_POS]; 
  *param3  =  response_str[RESPAR3_POS];   
  
  Serial.print("Get Response : ");
  checksum_resp = FingerPrint_Print_and_Checksum(response_str, RSP_LEN);

  Serial.println("Get Data : ");
  checksum_data = FingerPrint_Print_and_Checksum(data_str_ptr, IDDATA_LEN);

  if (response_str[RESCHKSUM_POS] == checksum_resp && data_str_ptr[IDDATACHKSUM_POS] == checksum_data) {
      Serial.println("PASS : Both response's and data's checksums are good.");
      return (1); // pass status
  } else {
      Serial.print("FAIL : ");
      if (response_str[RESCHKSUM_POS] != checksum_resp) 
          Serial.print("Response Checksum is wrong.  ");
      if (data_str_ptr[IDDATACHKSUM_POS] != checksum_data)
          Serial.print("Data Checksum is wrong.  ");
      Serial.println(" ");
      //Serial.println(data_str[IDDATACHKSUM_POS],HEX);
      //Serial.println(checksum_data,HEX);
      return (0); // fail status    
  }
}

/*
 *  Description : Print the data array and calculate the checksum value
 *  
 *  Input: Array pointer
 *  Output: Checksum
 * 
 */
char FingerPrint_Print_and_Checksum (char *data_str, int len){
  char checksum;
  int value;

  checksum = 0;
  for (int i=0; i<len; i++){
    value = data_str[i];
    value = value & 0x000000FF;
    // For single hex digit, add a leading zero.
    if (value < 16) 
      Serial.print(0,HEX);  
    Serial.print(value,HEX);
    if (i < len-1) Serial.print(",");   
    if (i > 0 && i < len-2)  // there are 3 bytes required
                             // to be excluded.
      checksum = checksum ^ data_str[i];
    if (i%10 == 9)           // display 10 bytes per line
      Serial.println(" ");   
  }  
  Serial.print(" [checksum = ");
  value = checksum;
  value = value & 0x000000FF;
  // For single hex digit, add a leading zero.
  if (value < 16) 
      Serial.print(0,HEX);  
  Serial.print(value,HEX);  
  Serial.println("] ");  
 
  return checksum;
}


void FingerPrint_Print_Response(char command, char param1,char param2, char param3) {
  int value;
  
  Serial.print("Get Response : ");
  Serial.print(command,HEX);
  Serial.print(",");    
  Serial.print(param1,HEX);
  Serial.print(",");    
  Serial.print(param2,HEX);
  Serial.print(",");    
  Serial.print(param3,HEX);
  Serial.println(" ");   
}





