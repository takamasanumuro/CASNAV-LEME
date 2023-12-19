#include <Arduino.h>
//Program to read an analog input of a rudder  via a potentiometer and map it to angle

constexpr int rudderPin = A0; //Analog input pin for rudder potentiometer
constexpr int zeroRudderADCValue = 488; // ADC value when rudder is at zero degrees
constexpr float deltaRudderADCValue = 3.178f; // Change in ADC per degree of rudder angle

//Linear mapping of ADC value to the corresponding physical measurement being sensed
float linearMap(int analogPin, int zeroADCValue, float deltaADCValue) {
	return (analogRead(analogPin) - zeroADCValue) / deltaADCValue;
}

void setup() {
	Serial.begin(115200);
	pinMode(rudderPin, INPUT); pinMode(LED_BUILTIN, OUTPUT);
	Serial.println("Initializing rudder angle sensor...");
}

void loop() {
	static unsigned long previousTime = millis();
	
	// The following buffers are used to format the output string for the UART
	char buffer[64]; // General purpose buffer
	char stringBuffer[8]; // Buffer to convert float values to string since Arduino doesn't support %f in sprintf

	if (millis() - previousTime >= 1000) {
		previousTime = millis();
		digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
		float angle = linearMap(rudderPin, zeroRudderADCValue, deltaRudderADCValue);
		dtostrf(angle, 4, 2, stringBuffer);
		sprintf(buffer, "Rudder angle: %s°", stringBuffer);
		Serial.println(buffer);
	}
}

