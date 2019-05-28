
#include <Pals2.h>

Pals2 pals;
char buffer[100];

void setup() {
	Serial.begin(38400);
	pals.begin();
  pals.setAmbientLightADCGain(800,false);
  //dumpRegisterContents();

}
void loop() {
   sprintf(buffer,"Raw Ambient Light = %d \n", pals.getRawAmbientLightOnDemand());
   Serial.print(buffer);
   Serial.print("-----------------------\n");
   delay(1000);
}

void dumpRegisterContents(){

  Serial.print("------------------------------- \n");
  Serial.print("PALS2 Registers\n");
  Serial.print("------------------------------- \n");
  for(int a=0x80;a<=0x9C;a++){
    Serial.print("0x");
    Serial.print(a, HEX);
    Serial.print(" = ");
    Serial.print("0x");
    Serial.print(pals.dumpRegister(a),HEX);
    Serial.print(" \n");
  }
  Serial.print("------------------------------- \n");
}
