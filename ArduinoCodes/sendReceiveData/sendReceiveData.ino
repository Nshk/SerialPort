/* 
 * Example for seceiving AND sending data from the serial port
 */


#define BAUD 9600
#define DELAY_TIME 100
#define led 13

String receivedString;

void setup() {
  Serial.begin(BAUD);
  pinMode(led, OUTPUT);
}

void loop() {
  if (Serial.available() > 0){
    receivedString = Serial.readStringUntil('\n');
  }

  if (receivedString.equals("ON")) {
    digitalWrite(led, HIGH);  
  } else if (receivedString.equals("OFF")) {
    digitalWrite(led, LOW);
  }
  if (!receivedString.equals(""))  {
    Serial.print("You sent: ");
    Serial.print(receivedString);
    Serial.print("\n");
    receivedString = "";
  }
}
