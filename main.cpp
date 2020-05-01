#include <iostream>
#include "SerialPort.hpp"
#include <stdio.h>
#include <string.h>

using namespace std;

//char* portName = "\\\\.\\COM20";

#define MAX_DATA_LENGTH 255
#define DEL 2

void slp(int s)
{
#ifdef _WIN32
        Sleep(s);
#else
        sleep(s);
#endif
}

void exampleReceiveData(SerialPort& ser, char* incomingData)
{
    int readResult = ser.readSerialPort(incomingData, MAX_DATA_LENGTH);
}

void exampleWriteData(SerialPort& ser)
{
    ser.writeSerialPort("PING\n", MAX_DATA_LENGTH);
}

int main(int argc, const char* argv[])
{
    if (argc != 2) {
        std::cout << "Usage: " << argv[0] <<  " serial_path_or_name\n";
        return 1;
    }
    char* incomingData = (char*)malloc(MAX_DATA_LENGTH);
    if (!incomingData) {
        cerr << "Error: buffer not allocated\n";
        return 1;
    } 
    SerialPort serial(argv[1], 9600);
    while(serial.isConnected()) {
        exampleWriteData(serial);
        cout << "Sent PING\n";
        slp(1);
        exampleReceiveData(serial, incomingData);
        cout << "Received " << incomingData;
        slp(1);
    }
    free(incomingData);
    return 0;
}
