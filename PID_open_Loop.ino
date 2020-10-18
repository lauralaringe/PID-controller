/* Code to test the open loop system and write a value to the LED. */

// Definition of the LDR measurmente (ADC A0) and the PWM digital output pin (9)
byte LEDpin = 9;
byte LDRpin = A0;

// Definition of auxiliar variables
String lecture;
double Input, Output;

// Definition of time auxiliar variables
unsigned long Time = 0, Time_ant = 0;

// Function to read data from the keyboard
void newConfigParameter() {

  if (Serial.available() > 0) {
    lecture = Serial.readString();
    Input = lecture.toFloat();
  }
}

// Setup function to initialize the serial communications and the output pin
void setup() {
  Serial.begin(9600);
  pinMode(LEDpin, OUTPUT);
}

// Loop function
void loop() 
{
  // Read the current time in miliseconds
  Time = millis();
    
  // Read data from the keyboard
  if (Serial.available() > 0) {
    lecture = Serial.readString();
    Input = lecture.toFloat();
    analogWrite(LEDpin, Input);
  }

  // Each second, read the LDP measurment and print
  if (Time - Time_ant >= 1000)
  { 
      // Measurement of the LDR sensor, and scale the value from 1023 to 255
      // In the PWM 5V are equal to 255
      Output = map(analogRead(LDRpin), 0, 1023, 0.0, 255.0);
      
      // Print data
      //Serial.print("LED input: ");
      Serial.print(Input);
      Serial.print("; ");
      //Serial.print("LDR lecture: ");
      Serial.println(Output);

      // Update time
      Time_ant = Time;
  }
}
