#include <Arduino.h>
#include <config.h>

static unsigned char serial2bufferRead[1000];

void gpsSetup(int a, int b, int c, int d, int e);
void sendPacket(byte *packet, byte len);