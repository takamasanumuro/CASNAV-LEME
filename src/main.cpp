#include <Arduino.h>
#include <EEPROM.h> // Access to permanent memory on the microcontroller
#include "Debug.h"
//Program to read an analog input of a rudder  via a potentiometer and map it to angle.

/* ATTENTION
Make sure to power the Arduino via the Vin pin or the power jack with a stable power supply, otherwise
the ADC readings will be unstable and the rudder angle will be inaccurate, as the conversion
uses the power supply voltage as a reference by default.
A precision voltage reference can be used on the ANAREF pin to improve the accuracy of the ADC readings
regardless of the stability of the power supply voltage.
*/

static constexpr int rudderPin = A0; //Analog input pin for rudder potentiometer
static int zeroRudderADCValue; // ADC value when rudder is at zero degrees
static float deltaRudderADCValue; // Change in ADC per degree of rudder angle

//Linear mapping of ADC value to the corresponding physical measurement being sensed
float linearMap(int analogPin, int zeroADCValue, float deltaADCValue) {

	int adcValue = analogRead(analogPin);
	DEBUG_PRINT("ADC value: "); DEBUG_PRINTLN(adcValue);
	return (adcValue - zeroADCValue) / deltaADCValue;
}

void SaveFlashInteger(int address, int value) {
	EEPROM.update(address, value & 0xFF);
	EEPROM.update(address + 1, (value >> 8) & 0xFF);
}

int ReadFlashInteger(int address) {
	return (EEPROM.read(address) | (EEPROM.read(address +1) << 8));
}

void ResetFlashMemory() {
	noInterrupts();
	for (int i = 0; i < EEPROM.length(); i++) {
		EEPROM.update(i, 0xFF);
	}
	Serial.println("RESET");
	delay(500);
	interrupts();
}

// Configure the flash memory reset button
void ConfigureFlashReset(uint8_t inputPin, uint8_t outputPin) {
	pinMode(inputPin, INPUT_PULLUP);

	if (outputPin != -1) {
		pinMode(outputPin, OUTPUT); digitalWrite(outputPin, LOW);
	}

	// Attach an interrupt to the input pin to detect when the user presses the button
	attachInterrupt(digitalPinToInterrupt(inputPin), &ResetFlashMemory, FALLING);
}

void CalibrateReading(int measurePin, int* zeroADCValue, float* deltaADCValue) {

	constexpr int zeroADCValueAddress = 0;
	constexpr int deltaADCValueAddress = 2;

	int zeroADC = ReadFlashInteger(zeroADCValueAddress);
	float deltaADC = ReadFlashInteger(deltaADCValueAddress);

	Serial.print("TST_zeroADC: "); Serial.println(zeroADC);

	//Check if the values already exist
	if (zeroADC != 0xFFFF && deltaADC != 0xFFFF) {
		Serial.println("Existing values detected for calibration settings");
		Serial.print("Stored Zero ADC value: "); Serial.println(zeroADC);
		Serial.print("Stored Delta ADC value: "); Serial.println(deltaADC);
		*zeroADCValue = zeroADC;
		*deltaADCValue = deltaADC;
		return;
	} 

	// The following buffers are used to format the output string for the UART
	char buffer[64]; // General purpose buffer
	char stringBuffer[8]; // Buffer to convert float values to string since Arduino doesn't support %f in sprintf
	char alternateStringBuffer[8]; // Buffer to convert float values to string since Arduino doesn't support %f in sprintf
	
	Serial.println("Move the rudder to the center and press any key to continue...");
	while (!Serial.available());
	while (Serial.available()) Serial.read();
	int centerADCValue = analogRead(measurePin);
	sprintf(buffer, "Center ADC value: %d", centerADCValue);
	Serial.println(buffer);
	
	Serial.println("Move the rudder to the 45 degrees left and press any key to continue...");
	while (!Serial.available());
	while (Serial.available()) Serial.read();
	int referenceAngleADCValue = analogRead(measurePin);
	sprintf(buffer, "Reference angle ADC value: %d", referenceAngleADCValue);
	Serial.println(buffer);

	// Calculate the delta ADC value per degree of rudder angle
	*zeroADCValue = centerADCValue;
	*deltaADCValue = (referenceAngleADCValue - centerADCValue) / 45.0f;

	//Store in flash memory
	SaveFlashInteger(zeroADCValueAddress, *zeroADCValue);
	SaveFlashInteger(deltaADCValueAddress, *deltaADCValue);

	dtostrf(*deltaADCValue, 1, 2, stringBuffer);
	sprintf(buffer, "Calibration complete. Zero ADC value: %d, Delta ADC value: %s", zeroADCValue, stringBuffer);
	Serial.println(buffer);

	// Wait for the user to press a key before continuing
	Serial.println("Press any key to continue...");
	while (!Serial.available());
	while(Serial.available()) Serial.read();
}

void setup() {
	Serial.begin(115200);
	pinMode(rudderPin, INPUT); pinMode(LED_BUILTIN, OUTPUT);
	Serial.println("Initializing rudder angle sensor...");

	ConfigureFlashReset(/*InputPin*/ 21, /*OutputPin*/ 22);
	CalibrateReading(rudderPin, &zeroRudderADCValue, &deltaRudderADCValue);

	Serial.println("Initialization complete");
	Serial.print("Zero ADC value: "); Serial.println(zeroRudderADCValue);
	Serial.print("Delta ADC value: "); Serial.println(deltaRudderADCValue);
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
		dtostrf(angle, 1, 0, stringBuffer);
		sprintf(buffer, "Rudder angle: %s°", stringBuffer);
		Serial.println(buffer);
	}
}

