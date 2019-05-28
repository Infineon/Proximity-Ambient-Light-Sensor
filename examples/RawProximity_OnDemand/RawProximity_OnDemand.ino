
#include <Pals2.h>


#define IRED_CURRENT 10 //100 mA

Pals2 pals;
char buffer[100];

void setup() {
	Serial.begin(38400);
	pals.begin();
  pals.setProximityMeasurementRate(16);
  //dumpRegisterContents();

}
void loop() {

  for(int iredNr = IRED1; iredNr <= IRED3; iredNr++){
    pals.setProximityOutput(iredNr, IRED_CURRENT);
    sprintf(buffer,"Proximity LED %d = %d \n", iredNr+1 ,pals.getRawProximityOnDemand());
    Serial.print(buffer);
  }
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
