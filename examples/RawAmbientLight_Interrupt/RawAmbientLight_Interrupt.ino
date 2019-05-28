
#include <Pals2.h>


// Pins
#define PALS_INT    9  // Needs to be an interrupt pin

// Constants
#define ALS_INT_HIGH   50000 // Proximity level for interrupt
#define ALS_INT_LOW    1000  // No far interrupt

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
  Serial.println(F(" INFINEON PALS - AmbienTLightInterrupt "));
  Serial.println(F("---------------------------------------"));

  // Initialize interrupt service routine
  attachInterrupt(0, interruptRoutine, FALLING);
  
	pals.begin();
  pals.setAmbientLightMeasurementRate(4);
  pals.setAmbientLightADCGain(800,false);

  pals.setAmbientLightInterrupt(ALS_INT_HIGH,DISABLE,ALS_INT_LOW, ENABLE);
  pals.setAmbientLightPeriodicMeasurements(ENABLE);
  //Uncomment fucntion call to debug 
  //dumpRegisterContents();

}
void loop() {

  if ( isr_flag == 1 ) {
    sprintf(buffer,"Ambient Light = %d \n", pals.readAmbientLight());
    Serial.print(buffer);
    isr_flag = 0; 
  }

}

void interruptRoutine() {
  isr_flag = 1;
  pals.clearAmbientLightInterrupts();
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
