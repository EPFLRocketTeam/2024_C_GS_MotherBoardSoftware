#include <Arduino.h>
#include <capsule.h>  
#include <Adafruit_NeoPixel.h>
#include <rotator.h>
#include <TinyGPSPlus.h>
#include <gps.h>

#include "ERT_RF_Protocol_Interface/Protocol.h"
#include "config.h"

void handleRF_UPLINK(uint8_t packetId, uint8_t *dataIn, uint32_t len); 
void handleRF_AV_DOWNLINK(uint8_t packetId, uint8_t *dataIn, uint32_t len); 
void handleRF_GSE_DOWNLINK(uint8_t packetId, uint8_t *dataIn, uint32_t len); 
void handleUi(uint8_t packetId, uint8_t *dataIn, uint32_t len);
void handleAntennaRotator(uint8_t packetId, uint8_t *dataIn, uint32_t len);
void handleCameraRotator(uint8_t packetId, uint8_t *dataIn, uint32_t len);
// void handleBinoculars(uint8_t packetId, uint8_t *dataIn, uint32_t len);
// void handleCommandInput(uint8_t packetId, uint8_t *dataIn, uint32_t len);
// void sendRotatorCmd(PacketTrackerCmd cmd);

Adafruit_NeoPixel ledA(1, NEOPIXEL_A_PIN, NEO_GRB + NEO_KHZ800); // 1 led
Adafruit_NeoPixel ledB(1, NEOPIXEL_B_PIN, NEO_GRB + NEO_KHZ800); // 1 led

TinyGPSPlus gps;
av_downlink_t lastPacket;
// PacketTrackerCmd cmdToSend;
// PacketBinocGlobalStatus binocGlobalStatus;

// RF communications with the pad
CapsuleStatic RF_UPLINK(handleRF_UPLINK);
CapsuleStatic RF_AV_DOWNLINK(handleRF_AV_DOWNLINK);
CapsuleStatic RF_GSE_DOWNLINK(handleRF_GSE_DOWNLINK);

// Data commnuications with ground control's operation computer
CapsuleStatic Ui(handleUi);

// Data for the antenna and camera rotators
CapsuleStatic AntennaRotator(handleAntennaRotator);
CapsuleStatic CameraRotator(handleCameraRotator);
// CapsuleStatic Binoculars(handleBinoculars);
// CapsuleStatic CommandInput(handleCommandInput);

static rotClass rotator;

float yawBinocOffset = 0;
float pitchBinocOffset = 0;

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
	pinMode(LED_BUILTIN,OUTPUT);
	// put your setup code here, to run once:

	// Initialize all motherboard ports
	RF_UPLINK_PORT.begin(RF_UPLINK_BAUD);
	RF_AV_DOWNLINK_PORT.begin(RF_AV_DOWNLINK_BAUD);
	RF_GSE_DOWNLINK_PORT.begin(RF_GSE_DOWNLINK_BAUD);

	UI_PORT.begin(115200);
	ANTENNA_ROTATOR_PORT.begin(ANTENNA_ROTATOR_BAUD);
	CAMERA_ROTATOR_PORT.begin(CAMERA_ROTATOR_BAUD);
	BINOCULARS_PORT.begin(BINOCULARS_BAUD);
	COMMAND_INPUT_PORT.begin(COMMAND_INPUT_BAUD);

	ledA.begin();
	ledA.fill(0x00FF00);
	ledA.show();

	ledB.begin();
	ledB.fill(0x00FF00);
	ledB.show();
	
	//
	rotator.begin();

	position groundPosition;
	groundPosition.lat = 39.4826182;
	groundPosition.lon = -8.3369708;
	groundPosition.alt = 100.0;
	rotator.setGroundPosition(groundPosition);

	rotator.setNominalPositionRate(1.0);
	rotator.setNominalPointerRate(20.0);

	GPS_PORT.begin(GPS_BAUD);
	gpsSetup(GPS_BAUD, 1, 4, 1, 0);
}

void loop() {

	while (GPS_PORT.available()) {
		gps.encode(GPS_PORT.read());
	}
	
	// If the GPS has received a new valid location, we update the rotator's ground position
	if (gps.location.isUpdated() && gps.location.isValid()) {
		position groundPosition;
		groundPosition.lat = gps.location.lat();
		groundPosition.lon = gps.location.lng();

		#if FLIGHT
		groundPosition.alt = gps.altitude.meters();
		#elif
		groundPosition.alt = lastPacket.gnss_alt;
		#endif

		rotator.setGroundPosition(groundPosition);

		// Debugging
		#ifdef DEBUG
		Serial.print("Ground position updated: ");
		Serial.print(groundPosition.lat,7);
		Serial.print(" ");
		Serial.print(groundPosition.lon,7);
		Serial.print(" ");
		Serial.print(groundPosition.alt,1);
		Serial.print(" ");
		Serial.println(gps.satellites.value());
		#endif
	}

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

	// while (BINOCULARS_PORT.available()) {
	// 	Binoculars.decode(BINOCULARS_PORT.read());
	// }

	// while (COMMAND_INPUT_PORT.available()) {
	// 	CommandInput.decode(COMMAND_INPUT_PORT.read());
	// }

	while (UI_PORT.available()) {
		Ui.decode(UI_PORT.read());
	} 

	// Upon updating the rotator's data, its internal FSM is updated and a new command is computed
	if (rotator.isUpdated()) {
		static TARGET_MODE lastMode = TARGET_MODE::TARGET_NONE;

		rotator.setMode(rotator.computeMode());

		// If the rotator's mode changes, a new estimate is computed
		if (rotator.getMode() != lastMode) {
			Serial.print("Rotator mode : ");
			Serial.println(rotator.getMode());

			if (lastMode == TARGET_MODE::TARGET_NONE and rotator.getMode() == TARGET_MODE::TARGET_POSITION) {
				position lastPosition = rotator.getPosition();
				rotator.latEstimator.reset(lastPosition.lat);
				rotator.lonEstimator.reset(lastPosition.lon);
				rotator.altEstimator.reset(lastPosition.alt);
			}

			lastMode = rotator.getMode();
		}

		// UI_PORT.print("Rotator mode : ");
		// UI_PORT.println(rotator.getMode());		
	}
}


void handleRF_AV_DOWNLINK(uint8_t packetId, uint8_t *dataIn, uint32_t len) {
	switch (packetId) {
		#ifdef RF_PROTOCOL_FIREHORN
		case CAPSULE_ID::AV_TELEMETRY:
		#else
		case CAPSULE_ID::HOPPER_DOWNLINK:
		#endif 
		{
			// UI_PORT.println("Packet with ID 00 from RF_AV_DOWN received : ");
			uint32_t ledColor = colors[random(sizeof(colors)/sizeof(uint32_t))];
			ledB.fill(ledColor);
			ledB.show();

			memcpy(&lastPacket, dataIn, av_downlink_size);
		
			
			// Feedback to the UI is provided
			uint8_t* packetToSend = Ui.encode(packetId,dataIn,len);
			UI_PORT.write(packetToSend,Ui.getCodedLen(len));
			delete[] packetToSend;
			break;
		}
		default:
			break;

	}
	uint32_t ledColor = colors[random(0,7)];
	ledA.fill(ledColor);
	ledA.show();
}

// Data from the GSE is routed to the UI to provide feedback
void handleRF_GSE_DOWNLINK(uint8_t packetId, uint8_t *dataIn, uint32_t len) {	
	switch (packetId) {
		case CAPSULE_ID::GSE_TELEMETRY:
		{
			uint8_t* packetToSend = Ui.encode(packetId,dataIn,len);
			// TODO
			UI_PORT.write(packetToSend,Ui.getCodedLen(len));
			delete[] packetToSend;
			break;
		}
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
	RF_UPLINK_PORT.write(packetToSend,RF_UPLINK.getCodedLen(len));
	delete[] packetToSend;
	
	uint32_t ledColor = colors[random(0,7)];
	ledB.fill(ledColor);
	ledB.show();
}



// These little guys don't have anything to handle since none are routing through the motherboard 

void handleCameraRotator(uint8_t packetId, uint8_t *dataIn, uint32_t len) {}

void handleAntennaRotator(uint8_t packetId, uint8_t *dataIn, uint32_t len) {}

void handleRF_UPLINK(uint8_t packetId, uint8_t *dataIn, uint32_t len) {}