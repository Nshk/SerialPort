#include <iostream>
#include "SerialPort.hpp"
#include <stdio.h>
#include <string.h>

using namespace std;

//char* portName = "\\\\.\\COM20";

#define MAX_DATA_LENGTH 255
#define DEL 2
//Control signals for turning on and turning off the led
//Check arduino code
char ledON[] = "ON\n";
char ledOFF[] = "OFF\n";

void exampleReceiveData(SerialPort* arduino, char* incomingData)
{
    int readResult = arduino->readSerialPort(incomingData, MAX_DATA_LENGTH);
    cout << incomingData << "\n";
    #ifdef _WIN32
    Sleep(DEL);
    #else
    sleep(DEL);
    #endif
}

void exampleWriteData(SerialPort* arduino)
{
    arduino->writeSerialPort(ledON, MAX_DATA_LENGTH);
    #ifdef _WIN32
    Sleep(DEL);
    #else
    sleep(DEL);
    #endif
    arduino->writeSerialPort(ledOFF, MAX_DATA_LENGTH);
    #ifdef _WIN32
    Sleep(DEL);
    #else
    sleep(DEL);
    #endif
}

int main(int argc, const char* argv[])
{
    char* incomingData = (char*)malloc(MAX_DATA_LENGTH);
    if (!incomingData) {
        cerr << "Error: buffer not allocated\n";
        return 1;
    } 
    SerialPort ard(argv[1], 9600);
    SerialPort* arduino = &ard;
    if (!arduino) {
        std::cerr << "Error, object non created!\n";
    }
    while(arduino->isConnected()) {
        exampleWriteData(arduino);
        exampleReceiveData(arduino, incomingData);
    }
    free(incomingData);
    return 0;
}
