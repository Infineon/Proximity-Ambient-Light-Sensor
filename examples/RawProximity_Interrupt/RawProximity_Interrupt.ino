
#include <Pals2.h>


#define IRED_CURRENT 10 //100 mA

// Pins
#define PALS_INT    9  // Needs to be an interrupt pin

// Constants
#define PROX_INT_HIGH   50000 // Proximity level for interrupt
#define PROX_INT_LOW    0  // No far interrupt

Pals2 pals;
char buffer[100];
int isr_flag = 0;

void setup() {

delay(1000);
// Set PIN as interrupt
  pinMode(PALS_INT, INPUT);
  
  // Initialize Serial port
  Serial.begin(38400);
  Serial.println();
  Serial.println(F("---------------------------------------"));
  Serial.println(F("  INFINEON PALS  - ProximityInterrupt  "));
  Serial.println(F("---------------------------------------"));

  // Initialize interrupt service routine
  attachInterrupt(0, interruptRoutine, FALLING);
  
	pals.begin();
  pals.setProximityMeasurementRate(16);
  pals.setProximityOutput(IRED1, IRED_CURRENT);
  pals.setProximityInterrupt(PROX_INT_HIGH,ENABLE,PROX_INT_LOW, DISABLE);
  pals.setProximityPeriodicMeasurements(ENABLE);

  //Uncomment fucntion call to debug 
  //dumpRegisterContents();

}
void loop() {

  if ( isr_flag == 1 ) {
    sprintf(buffer,"Proximity = %d \n", pals.readProximity());
    Serial.print(buffer);
    isr_flag = 0; 
  }

}

void interruptRoutine() {
  isr_flag = 1;
  pals.clearProximityInterrupts();
  Serial.print("Interrupt! ");
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
