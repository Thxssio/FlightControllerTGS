#include <FlightControllerTGS.h>


FlightControllerTGSOutput myOut;


void setup() {
  myOut.begin(5, 7); 
  pinMode(2, OUTPUT);
  pinMode(2, HIGH);   
  myOut.write(1, 800.0);
  myOut.write(2, 1721);
  myOut.write(3, 912.12);
}

void loop() {
}
