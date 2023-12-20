// Potentiometer is connected to GPIO 34 (Analog ADC1_CH6)
const int potPin = 34;

// variable for storing the potentiometer value
int potValue = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
}

void loop() {
  int adc = 0;
  long adc_sum = 0; // must be long to hold a large value

  for ( int i = 0; i < 100; i++ )
  {
    adc = analogRead( potPin );
    adc_sum = adc_sum + adc;
  }

  // divide by the number of readings to get the average
  adc = adc_sum / 100;
  // Reading potentiometer value
  //  potValue = analogRead(potPin);
  Serial.println(adc);
  delay(50);
}
