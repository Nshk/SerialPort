/* 
 * Example for seceiving AND sending data from the serial port
 */


#define BAUD 9600
#define DELAY_TIME 100
#define led 13

String receivedString;

void setup() {
    Serial.begin(BAUD);
}

void loop() {
    if (Serial.available() > 0){
        receivedString = Serial.readStringUntil('\n');
    }
    
    if (receivedString.equals("PING"))  {
        Serial.print("PONG\n");
        receivedString = "";
    }
}
