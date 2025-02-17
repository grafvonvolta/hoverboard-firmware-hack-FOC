#include "sbus.h"

//#define RXD2 16
//#define TXD2 17
// ########################## DEFINES ##########################
#define HOVER_SERIAL_BAUD   115200      // [-] Baud rate for Serial1 (used to communicate with the hoverboard)
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

#define SSR  23
#define GAS  33
#define BREAK  25

//#include <SoftwareSerial.h>
//SoftwareSerial Serial1(RXD2,TXD2);        // RX, TX
//Serial3(RXD2,TXD2);        // RX, TX


// Global variables
uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

int gasValue = 0;
int breakValue = 0;

typedef struct {
  uint16_t start;
  int16_t  steer;
  int16_t  speed;
  uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct {
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
  Serial1.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, RXD2, TXD2);

  while (!Serial) {}
  /* Begin the SBUS communication */
  sbus_rx.Begin();
  //  sbus_tx.Begin();
  //  Serial.println("Setup done");

  pinMode(SSR, OUTPUT);
  digitalWrite(SSR, LOW);
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
  Serial1.write((uint8_t *) &Command, sizeof(Command));
}

unsigned long iTimeSend = 0;
int iTest = 0;
int iStep = SPEED_STEP;
int Speed = 0;
int SSR_toogle = 0;
int Overwrite = 0;
int averageGasValue = 0;
int averageBreakValue = 0;
int GasMappedValue = 0;
int BreakMappedValue = 0;

// ########################## RECEIVE ##########################
void Receive()
{
  // Check for new data availability in the Serial buffer
  if (Serial1.available()) {
    incomingByte    = Serial1.read();                                   // Read the incoming byte
    bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev;       // Construct the start frame
  }
  else {
    return;
  }

  // If DEBUG_RX is defined print all incoming bytes
#ifdef DEBUG_RX
  Serial.print(bufStartFrame);
  return;
#endif

  // Copy received data
  if (bufStartFrame == START_FRAME) {                     // Initialize if new data is detected
    p       = (byte *)&NewFeedback;
    *p++    = incomingBytePrev;
    *p++    = incomingByte;
    idx     = 2;
  } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data
    *p++    = incomingByte;
    idx++;
  }

  // Check if we reached the end of the package
  if (idx == sizeof(SerialFeedback)) {
    uint16_t checksum;
    checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
                          ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

    // Check validity of the new data
    if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
      // Copy the new data
      memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));

      // Print data to built-in Serial
      //            Serial.print(" 1: ");  Serial.print(Feedback.cmd1);
      Serial.print(" Input: ");  Serial.print(Feedback.cmd2);
      Serial.print(" SpeedL: ");  Serial.print(Feedback.speedL_meas);
      Serial.print(" SpeedR: ");  Serial.print(Feedback.speedR_meas);
      Serial.print(" V: ");  Serial.print(Feedback.batVoltage / 100);
      //            Serial.print(" T: ");  Serial.print(Feedback.boardTemp);
      //            Serial.print(" Led: ");  Serial.print(Feedback.cmdLed);
      Serial.println("");
    } else {
      Serial.println("Non-valid data skipped");
    }
    idx = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
  }

  // Update previous states
  incomingBytePrev = incomingByte;
}

int readPotAverage(int potPin, int numReadings) {
  long adc_sum = 0; // must be long to hold a large value

  for (int i = 0; i < numReadings; i++) {
    int adc = analogRead(potPin);
    adc_sum += adc;

    delay(0.1);
  }

  // Calculate the average
  int adc_average = adc_sum / numReadings;

  return adc_average;
}


// ########################## LOOP ##########################

void loop () {
  unsigned long timeNow = millis();

  Receive();

  if (sbus_rx.Read()) {
    /* Grab the received data */
    data = sbus_rx.data();
    /* Display the received data */


    SSR_toogle = data.ch[6];
    Overwrite = data.ch[8];

    if (SSR_toogle > 1000) {
      digitalWrite(SSR, HIGH);
    } else {
      digitalWrite(SSR, LOW);
    }

    //    Serial.println(SSR_toogle);
    //    Serial.println(Speed);
    //    Serial.print("\t");

    //    expected values: 372, 1090, 1810

    // Failsafe trigger
    if (Overwrite > 1200) {
      // input from steeringwheel
      averageGasValue = readPotAverage(GAS, 100);
      averageBreakValue = readPotAverage(BREAK, 100);

      if ((averageBreakValue > 400) and (averageGasValue > 500)) {
        Send(0, 1090); // send 0 speed (0 = 1090)
      } else if ((averageBreakValue > 400) and (averageGasValue <= 500)) {
        //        BreakMappedValue = map(averageBreakValue, 300, 3000, 1090, 372);
        //        BreakMappedValue = map(averageBreakValue, 300, 3000, 1090, 731); // map to half speed
        BreakMappedValue = map(averageBreakValue, 300, 3000, 1090, 1000); // map to minimal speed
        Send(0, BreakMappedValue);
      } else if ((averageBreakValue <= 400) and (averageGasValue > 500)) {
        //        GasMappedValue = map(averageGasValue, 500, 3800, 1090, 1810);
        //        GasMappedValue = map(averageGasValue, 500, 3800, 1090, 1450); // map to half speed
        GasMappedValue = map(averageGasValue, 500, 3800, 1090, 1200); // map to minimal speed

        Send(0, GasMappedValue);
      } else {
        Send(0, 1090); // send 0 speed (0 = 1090)
      }
    } else
    {
      Speed = data.ch[1];
      // input from remote
      if (Speed > 300) {
        Send(0, Speed);
      }
    }

    //  for (int i = 0; i < 10; ++i) {
    //    Serial.print("Content of data.ch[");
    //    Serial.print(i);
    //    Serial.print("]: ");
    //    Serial.println(data.ch[i]);
    //  }



    Serial.print("Gas remote: ");
    Serial.println(Speed);
    Serial.print("Gas: ");
    Serial.println(averageGasValue);
    Serial.print("Break: ");
    Serial.println(averageBreakValue);
    Serial.print("BreakMapped: ");
    Serial.println(BreakMappedValue);
    Serial.print("Overwrite: ");
    Serial.println(data.ch[8]);
    Serial.print("Speed: ");
    Serial.println(Speed);


    //    Serial.print(data.ch[1]);

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
