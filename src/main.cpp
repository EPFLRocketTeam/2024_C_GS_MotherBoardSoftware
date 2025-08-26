#include <Arduino.h>
#include <capsule.h>  
#include <Adafruit_NeoPixel.h>

#include "ERT_RF_Protocol_Interface/Protocol.h"
#include "config.h"

void handleRF_UPLINK(uint8_t packetId, uint8_t *dataIn, uint32_t len); 
void handleRF_AV_DOWNLINK(uint8_t packetId, uint8_t *dataIn, uint32_t len); 
void handleRF_GSE_DOWNLINK(uint8_t packetId, uint8_t *dataIn, uint32_t len); 
void handleUi(uint8_t packetId, uint8_t *dataIn, uint32_t len);
void handleCommandInput(uint8_t packetId, uint8_t *dataIn, uint32_t len);

Adafruit_NeoPixel ledA(1, NEOPIXEL_A_PIN, NEO_GRB + NEO_KHZ800); // 1 led
Adafruit_NeoPixel ledB(1, NEOPIXEL_B_PIN, NEO_GRB + NEO_KHZ800); // 1 led

// RF communications with the pad
CapsuleStatic RF_UPLINK(handleRF_UPLINK);
CapsuleStatic RF_AV_DOWNLINK(handleRF_AV_DOWNLINK);
CapsuleStatic RF_GSE_DOWNLINK(handleRF_GSE_DOWNLINK);

// Data commnuications with ground control's operation computer
CapsuleStatic Ui(handleUi);

// Data for the antenna and camera rotators
CapsuleStatic CommandInput(handleCommandInput);

uint32_t colors[] = {
	0x32A8A0, // Cyan
	0x0000FF, // Blue
	0xFFEA00, // Yellow
	0x00FF00, // Green
	0xFF0000, // Red
	0xCF067C, // Purple
	0xFF0800  // Orange
}; 


void setup() {
	pinMode(LED_BUILTIN, OUTPUT);

	// Initialize all motherboard ports
	RF_UPLINK_PORT.begin(RF_UPLINK_BAUD);
	RF_AV_DOWNLINK_PORT.begin(RF_AV_DOWNLINK_BAUD);
	RF_GSE_DOWNLINK_PORT.begin(RF_GSE_DOWNLINK_BAUD);

	UI_PORT.begin(115200);
	COMMAND_INPUT_PORT.begin(COMMAND_INPUT_BAUD);
	
	ledA.begin();
	ledA.fill(0x00FF00);
	ledA.show();

	ledB.begin();
	ledB.fill(0x00FF00);
	ledB.show();
}

void loop() {
	// Data for each port is decoded and processed

	while (RF_UPLINK_PORT.available()) {
		RF_UPLINK.decode(RF_UPLINK_PORT.read());
	}

	while (RF_AV_DOWNLINK_PORT.available()) {
		RF_AV_DOWNLINK.decode(RF_AV_DOWNLINK_PORT.read());
	}

	while (RF_GSE_DOWNLINK_PORT.available()) {
		RF_GSE_DOWNLINK.decode(RF_GSE_DOWNLINK_PORT.read());
	}

	while (COMMAND_INPUT_PORT.available()) {
		CommandInput.decode(COMMAND_INPUT_PORT.read());
	}

	while (UI_PORT.available()) {
		Ui.decode(UI_PORT.read());
	} 

}


void handleRF_AV_DOWNLINK(uint8_t packetId, uint8_t *dataIn, uint32_t len) {
	uint8_t* packetToSend;
	switch (packetId) {
	case GSC_INTERNAL_VEHICLE_DOWNLINK:
	case AV_TELEMETRY: // idem as HOPPER_TELEMETRY
		// Feedback to the UI is provided
		packetToSend = Ui.encode(packetId,dataIn,len);
		UI_PORT.write(packetToSend, Ui.getCodedLen(len));
		delete[] packetToSend;
		break;
	default:
		break;
	}
	uint32_t ledColor = colors[random(0,7)];
	ledA.fill(ledColor);
	ledA.show();
}

// Data from the GSE is routed to the UI to provide feedback
void handleRF_GSE_DOWNLINK(uint8_t packetId, uint8_t *dataIn, uint32_t len) {	
	uint8_t* packetToSend;
	switch (packetId) {
	case GSC_INTERNAL_GSE_DOWNLINK:
	case GSE_TELEMETRY:
		packetToSend = Ui.encode(packetId, dataIn, len);
		UI_PORT.write(packetToSend, Ui.getCodedLen(len));
		delete[] packetToSend;
		break;
	default:
		break;
	}
	uint32_t ledColor = colors[random(0,7)];
	ledA.fill(ledColor);
	ledA.show();
}

// Commands from the UI are routed through to the uplink
void handleUi(uint8_t packetId, uint8_t *dataIn, uint32_t len) {
	uint8_t* packetToSend = RF_UPLINK.encode(packetId,dataIn,len);
	if (packetId == GSC_INTERNAL) {
		RF_AV_DOWNLINK_PORT.write(packetToSend, RF_UPLINK.getCodedLen(len));
		RF_GSE_DOWNLINK_PORT.write(packetToSend, RF_UPLINK.getCodedLen(len));
	}

	RF_UPLINK_PORT.write(packetToSend, RF_UPLINK.getCodedLen(len));
	delete[] packetToSend;
	
	uint32_t ledColor = colors[random(0,7)];
	ledB.fill(ledColor);
	ledB.show();
}


void handleCommandInput(uint8_t packetId, uint8_t *dataIn, uint32_t len) {
	uint8_t* packetToSend;
	switch (packetId) {
	case CAPSULE_ID::ABORT_BOARD:
		packetToSend = Ui.encode(packetId,dataIn,len);
		UI_PORT.write(packetToSend,Ui.getCodedLen(len));
		delete[] packetToSend;
		break;
	default:
		break;
	}
	uint32_t ledColor = colors[random(0,7)];
	ledA.fill(ledColor);
	ledA.show();
}

void handleRF_UPLINK(uint8_t packetId, uint8_t *dataIn, uint32_t len) {
	uint8_t* packetToSend;
	switch (packetId) {
	case GSC_INTERNAL_UPLINK:
		packetToSend = Ui.encode(packetId, dataIn, len);
		UI_PORT.write(packetToSend, Ui.getCodedLen(len));
		delete[] packetToSend;
		break;
	default:
		break;
	}
	uint32_t ledColor = colors[random(0,7)];
	ledA.fill(ledColor);
	ledA.show();
}