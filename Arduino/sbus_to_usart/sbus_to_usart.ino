// ============================================================
// CONFIG
// ============================================================

#define HOVER_SERIAL_BAUD   115200
#define SERIAL_BAUD         115200
#define START_FRAME         0xABCD

// Hoverboard UART (UART2)
#define RXD2 16
#define TXD2 17

// ADC inputs (ADC1 only)
#define GAS    33
#define BREAK  32

// Neutral / limits
#define NEUTRAL_VALUE 1000
#define MIN_OUTPUT    0
#define MAX_OUTPUT    2000

// #define DEBUG_RX   // uncomment to print raw incoming bytes/frame words

// ============================================================
// GLOBALS
// ============================================================

uint8_t idx = 0;
uint16_t bufStartFrame;
byte *p;
byte incomingByte;
byte incomingBytePrev;

int Speed = 0;

int averageGasValue   = 0;
int averageBreakValue = 0;
int GasMappedValue    = NEUTRAL_VALUE;
int BreakMappedValue  = NEUTRAL_VALUE;

// ============================================================
// STRUCTS
// ============================================================

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

// ============================================================
// FUNCTIONS
// ============================================================

void Send(int16_t uSteer, int16_t uSpeed) {
  Command.start    = START_FRAME;
  Command.steer    = uSteer;
  Command.speed    = uSpeed;
  Command.checksum = Command.start ^ Command.steer ^ Command.speed;
  Serial1.write((uint8_t *)&Command, sizeof(Command));
}

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

int readPotAverage(int pin, int numReadings) {
  long sum = 0;
  for (int i = 0; i < numReadings; i++) {
    sum += analogRead(pin);
    delay(1);
  }
  return (int)(sum / numReadings);
}

// Inverse-log curve: slow near neutral, stronger near end
float mapInverseLog(int input, int inMin, int inMax, int outMin, int outMax) {
  float norm = (float)(input - inMin) / (inMax - inMin);
  norm = constrain(norm, 0.0f, 1.0f);
  float curved = 1.0f - sqrt(1.0f - norm);
  return outMin + curved * (outMax - outMin);
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial1.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, RXD2, TXD2);
}

// ============================================================
// LOOP
// ============================================================

void loop () {
  unsigned long timeNow = millis();

  // Read feedback from hoverboard (prints status when a valid frame is received)
  Receive();

  // ---------- FAILSAFE DEFAULT ----------
  int16_t sendValue = NEUTRAL_VALUE;

  // ---------- READ INPUTS ----------
  averageGasValue   = readPotAverage(GAS, 100);
  averageBreakValue = readPotAverage(BREAK, 100);

  // ---------- ADC SANITY CHECK ----------
  if (averageGasValue < 0 || averageGasValue > 4095 ||
      averageBreakValue < 0 || averageBreakValue > 4095) {
    Send(0, NEUTRAL_VALUE);
    return;
  }

  // ---------- YOUR ORIGINAL DECISION LOGIC ----------
  if ((averageBreakValue > 400) && (averageGasValue > 500)) {

    sendValue = NEUTRAL_VALUE;

  }
  else if ((averageBreakValue > 400) && (averageGasValue <= 500)) {

    BreakMappedValue = (int)mapInverseLog(
      averageBreakValue,
      209,     // brake min
      2997,    // brake max
      1000,    // neutral
      0        // full brake
    );

    sendValue = BreakMappedValue;

  }
  else if ((averageBreakValue <= 400) && (averageGasValue > 500)) {

    GasMappedValue = (int)mapInverseLog(
      averageGasValue,
      444,     // gas min
      3665,    // gas max
      1000,    // neutral
      2000     // full gas
    );

    sendValue = GasMappedValue;

  }
  else {

    sendValue = NEUTRAL_VALUE;
  }

  // ---------- HARD OUTPUT FAILSAFE ----------
  if (sendValue < MIN_OUTPUT || sendValue > MAX_OUTPUT) {
    sendValue = NEUTRAL_VALUE;
  }

  // ---------- SEND ----------
  Send(0, sendValue);

  // ---------- DEBUG PRINTS ----------
  Serial.print("Gas remote: ");
  Serial.println(Speed);
  Serial.print("Gas: ");
  Serial.println(averageGasValue);
  Serial.print("Break: ");
  Serial.println(averageBreakValue);
  Serial.print("BreakMapped: ");
  Serial.println(BreakMappedValue);
  Serial.print("GasMapped: ");
  Serial.println(GasMappedValue);
  Serial.print("Speed: ");
  Serial.println(Speed);
}
