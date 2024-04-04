// from https://github.com/Gmatarrubia/LibreriasTutoriales/tree/master
#ifndef SERIALCLASS_H_INCLUDED
#define SERIALCLASS_H_INCLUDED

#include <windows.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <stdio.h>
#include <vector>

#include <comdef.h>
#include <wbemidl.h>

#pragma comment(lib, "wbemuuid.lib")

#define ARDUINO_WAIT_TIME 2000
#define DEFAULT_READ_BUFFER_SIZE 255

class Serial
{
private:
	//Serial comm handler
	HANDLE hSerial;
	//Connection status
	bool connected;
	//Get various information about the connection
	COMSTAT status;
	//Keep track of last error
	DWORD errors;
	std::vector<char> m_vReadBuffer;
	unsigned char m_cReadBufferSize;

	std::string msgBuffer;

public:
	//Initialize Serial communication with the given COM port
	Serial(const char* portName);

	//Close the connection
	//NOTA: for some reason you can't connect again before exiting
	//the program and running it again
	~Serial();

	//Read data in a buffer, if nbChar is greater than the
	//maximum number of bytes available, it will return only the
	//bytes available. The function return -1 when nothing could
	//be read, the number of bytes actually read.
	int readData(std::string& readBuffer);

	//Writes data from a buffer through the Serial connection
	//return true on success.
	bool writeData(const std::string& writeBuffer);

	//Check if we are actually connected
	bool isConnected();

	std::vector<std::string> getCommandFromSerial();

	std::string GetUSBConnections();

};

#endif // SERIALCLASS_H_INCLUDED
