/*
* Author: Manash Kumar Mandal
* Modified Library introduced in Arduino Playground which does not work
* This works perfectly
* LICENSE: MIT
*/

#include "SerialPort.hpp"

SerialPort::SerialPort(const char *portName, long unsigned int baud)
{
    connected = false;
    #ifdef _WIN32
    // Windows code
    handler = CreateFileA(static_cast<LPCSTR>(portName),
                                GENERIC_READ | GENERIC_WRITE,
                                0,
                                NULL,
                                OPEN_EXISTING,
                                FILE_ATTRIBUTE_NORMAL,
                                NULL);
    if (handler == INVALID_HANDLE_VALUE)
    {
        if (GetLastError() == ERROR_FILE_NOT_FOUND)
        {
            std::cerr << "ERROR: Handle was not attached.Reason : " << portName << " not available\n";
        }
        else
        {
            std::cerr << "ERROR!!!\n";
        }
    }
    else
    {
        DCB dcbSerialParameters = {0};

        if (!GetCommState(handler, &dcbSerialParameters))
        {
            std::cerr << "Failed to get current serial parameters\n";
        }
        else
        {
            switch(baud) {
            case 9600:
                dcbSerialParameters.BaudRate = CBR_9600;
                break;
            case 19200:
                dcbSerialParameters.BaudRate = CBR_19200;
                break;
            case 38400:
                dcbSerialParameters.BaudRate = CBR_38400;
                break;
            case 57600:
                dcbSerialParameters.BaudRate = CBR_57600;
                break;
            case 115200:
                dcbSerialParameters.BaudRate = CBR_115200;
                break;
            default:
                dcbSerialParameters.BaudRate = CBR_9600;
                break;
            }
                
            dcbSerialParameters.ByteSize = 8;
            dcbSerialParameters.StopBits = ONESTOPBIT;
            dcbSerialParameters.Parity = NOPARITY;
            dcbSerialParameters.fDtrControl = DTR_CONTROL_ENABLE;

            if (!SetCommState(handler, &dcbSerialParameters))
            {
                std::cout << "ALERT: could not set serial port parameters\n";
            }
            else
            {
                connected = true;
                PurgeComm(handler, PURGE_RXCLEAR | PURGE_TXCLEAR);
                Sleep(ARDUINO_WAIT_TIME);
            }
        }
    }
    #else
    // Unix code
    handler = open(portName, O_RDWR);
    if (handler < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
    } else {
        // New termios struc
        struct termios tty;
        memset(&tty, 0, sizeof tty);
        // Read existing settings
        if(tcgetattr(handler, &tty) != 0) {
            printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        }

        tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity
        tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit
        tty.c_cflag |= CS8; // 8 bits per byte
        tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control
        tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines
        tty.c_lflag &= ~ICANON; //Disable canonical mode
        tty.c_lflag &= ~ECHO; // Disable echo
        tty.c_lflag &= ~ECHOE; // Disable erasure
        tty.c_lflag &= ~ECHONL; // Disable new-line echo
        tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
        tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
        // Disable any special handling of received bytes
        tty.c_oflag &= ~OPOST;
        // Prevent special interpretation of output bytes (e.g. newline chars)
        tty.c_oflag &= ~ONLCR;
        // Prevent conversion of newline to carriage return/line feed
        // Wait for up to 1s (10 deciseconds),
        //returning as soon as any data is received.
        tty.c_cc[VTIME] = 10;    
        tty.c_cc[VMIN] = 0;

        // Set in/out baud rate
        switch(baud) {
        case 9600:
            cfsetispeed(&tty, B9600);
            cfsetospeed(&tty, B9600);
            break;
        case 19200:
            cfsetispeed(&tty, B19200);
            cfsetospeed(&tty, B19200);
            break;
        case 38400:
            cfsetispeed(&tty, B38400);
            cfsetospeed(&tty, B38400);
            break;
        case 57600:
            cfsetispeed(&tty, B57600);
            cfsetospeed(&tty, B57600);
            break;
        case 115200:
            cfsetispeed(&tty, B115200);
            cfsetospeed(&tty, B115200);
            break;
        default:
            cfsetispeed(&tty, B9600);
            cfsetospeed(&tty, B9600);
            break;
        }
        // Save tty settings, also checking for error
        if (tcsetattr(handler, TCSANOW, &tty) != 0) {
            printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        } else {
            connected = true;
        }
    }
    #endif
}

SerialPort::~SerialPort()
{
    closeSerial();
}

// Reading bytes from serial port to buffer;
// returns read bytes count, or if error occurs, returns 0
int SerialPort::readSerialPort(char *buffer, unsigned int buf_size)
{
    #ifdef _WIN32
    DWORD bytesRead{};
    unsigned int toRead = 0;

    ClearCommError(handler, &errors, &status);

    if (status.cbInQue > 0)
    {
        if (status.cbInQue > buf_size)
        {
            toRead = buf_size;
        }
        else
        {
            toRead = status.cbInQue;
        }
    }
    #endif

    memset(buffer, 0, buf_size);
    #ifdef __WIN32
    if (ReadFile(handler, buffer, toRead, &bytesRead, NULL))
    {
        return bytesRead;
    }
    #else
    int r = 0;
    r = read(handler, buffer, buf_size);
    if (r >= 0) {
        return r;
    } else {
        perror("read: ");
        connected = false;
    }
    #endif
    return -1;
}

// Sending provided buffer to serial port;
// returns true if succeed, false if not
bool SerialPort::writeSerialPort(char *buffer, unsigned int buf_size)
{
    #ifdef _WIN32
    DWORD bytesSend;

    if (!WriteFile(handler, (void*) buffer, buf_size, &bytesSend, 0))
    {
        ClearCommError(handler, &errors, &status);
        return false;
    }
    #else
    if (write(handler, buffer, buf_size) < buf_size) {
        perror("write: ");
        connected = false;
        return false;
    }
    #endif
    return true;
}

// Checking if serial port is connected
bool SerialPort::isConnected()
{
    #ifdef _WIN32
    if (!ClearCommError(handler, &errors, &status))
    {
        connected = false;
    }
    #endif
    return connected;
}

void SerialPort::closeSerial()
{
    std::cout << "Closing Serial\n";
    connected = false;
    #ifdef _WIN32
    CloseHandle(handler);
    #else
    close(handler);
    #endif
}
