#include "sbus.h"

//#define RXD2 16
//#define TXD2 17
// ########################## DEFINES ##########################
#define HOVER_SERIAL_BAUD   115200      // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD         115200      // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME         0xABCD       // [-] Start frme definition for reliable serial communication
#define TIME_SEND           100         // [ms] Sending time interval
#define SPEED_MAX_TEST      300         // [-] Maximum speed for testing
#define SPEED_STEP          5          // [-] Speed step
// #define DEBUG_RX                        // [-] Debug received data. Prints all bytes to serial (comment-out to disable)

// hoverboard
#define RXD2 16
#define TXD2 17

// TBS crossfire
#define RXD1 5 //3
#define TXD1 22

#include <SoftwareSerial.h>
SoftwareSerial HoverSerial(RXD2,TXD2);        // RX, TX

// Global variables
uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

typedef struct{
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct{
   uint16_t start;
   int16_t  cmd1;
   int16_t  cmd2;
   int16_t  speedR_meas;
   int16_t  speedL_meas;
   int16_t  batVoltage;
   int16_t  boardTemp;
   uint16_t cmdLed;
   uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;

/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&Serial2, RXD1, TXD1, false);
/* SBUS object, writing SBUS */
//bfs::SbusTx sbus_tx(&Serial2, RXD1, TXD1, false);
/* SBUS data */
bfs::SbusData data;

// ########################## SETUP ##########################
void setup() {
  /* Serial to display data */
  Serial.begin(SERIAL_BAUD);
  HoverSerial.begin(HOVER_SERIAL_BAUD);
  
  while (!Serial) {}
  /* Begin the SBUS communication */
  sbus_rx.Begin();
//  sbus_tx.Begin();
  Serial.println("Setup done");
}

// ########################## SEND ##########################
void Send(int16_t uSteer, int16_t uSpeed)
{
  // Create command
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

//  Serial.println(Command.speed);

  // Write to Serial
  HoverSerial.write((uint8_t *) &Command, sizeof(Command)); 
}

unsigned long iTimeSend = 0;
int iTest = 0;
int iStep = SPEED_STEP;
int Speed = 0;

void loop () {
  unsigned long timeNow = millis();
  
  if (sbus_rx.Read()) {
    /* Grab the received data */
    data = sbus_rx.data();
    /* Display the received data */

    Speed = data.ch[1];
    
    Serial.println(Speed);
//    Serial.print("\t");

    // Failsafe trigger
    if (Speed > 300) {
      Send(0, Speed);
    }



    
//    Serial.print(data.ch[i]);
    
//    for (int8_t i = 0; i < data.NUM_CH; i++) {
//      Serial.print(data.ch[i]);
//      Serial.print("\t");
//    }
    /* Display lost frames and failsafe data */
//    Serial.print(data.lost_frame);
//    Serial.print("\t");
//    Serial.println(data.failsafe);
    /* Set the SBUS TX data to the received data */
//    sbus_tx.data(data);
    /* Write the data to the servos */
//    sbus_tx.Write();
  }
}
