/*
* Author: Manash Kumar Mandal
* Modified Library introduced in Arduino Playground which does not work
* This works perfectly
* LICENSE: MIT
*/

#pragma once

#define ARDUINO_WAIT_TIME 2000
#define MAX_DATA_LENGTH 255
#ifdef _WIN32
#include <windows.h>
#else
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <string.h>
#endif

#include <iostream>

class SerialPort
{
private:
    #ifdef _WIN32
    HANDLE handler;
    COMSTAT status;
    DWORD errors;
    #else
    int handler;
    #endif
    bool connected;
public:
    SerialPort(const char *portName, unsigned long int baud);
    ~SerialPort();

    int readSerialPort(char *buffer, unsigned int buf_size);
    bool writeSerialPort(char *buffer, unsigned int buf_size);
    bool isConnected();
    void closeSerial();
};
