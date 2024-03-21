#include "SerialPort.h"

Serial::Serial(const char* portName) : m_cReadBufferSize(DEFAULT_READ_BUFFER_SIZE)
{
	//We're not yet connected
	this->connected = false;

	// get size of the wchar_t 
	int wchars_num = MultiByteToWideChar(CP_UTF8, 0, portName, -1, NULL, 0);
	wchar_t* wstr = new wchar_t[wchars_num]; // create a buffer
	MultiByteToWideChar(CP_UTF8, 0, portName, -1, wstr, wchars_num); // set the value from convertion

	//Try to connect to the given port throuh CreateFile
	this->hSerial = CreateFile(wstr,
		GENERIC_READ | GENERIC_WRITE,
		0,
		NULL,
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL,
		NULL);

	delete[] wstr; // delete the dynamic array of wchar

	//Check if the connection was successfull
	if (this->hSerial == INVALID_HANDLE_VALUE)
	{
		//If not success full display an Error
		if (GetLastError() == ERROR_FILE_NOT_FOUND)
		{
			//Print Error if neccessary
			printf("[ERROR] Handle was not attached. Reason: %s not available.\n", portName);
		}
		else
		{
			printf("[ERROR]?!");
		}
	}
	else
	{
		//If connected we try to set the comm parameters
		DCB dcbSerialParams = { 0 };

		//Try to get the current
		if (!GetCommState(this->hSerial, &dcbSerialParams))
		{
			//If impossible, show an error
			printf("[ERROR] Failed to get current serial parameters!");
		}
		else
		{
			//Define serial connection parameters for the arduino board
			dcbSerialParams.BaudRate = CBR_9600;
			dcbSerialParams.ByteSize = 8;
			dcbSerialParams.StopBits = ONESTOPBIT;
			dcbSerialParams.Parity = NOPARITY;

			//Set the parameters and check for their proper application
			if (!SetCommState(hSerial, &dcbSerialParams))
			{
				printf("[ERROR] Could not set Serial Port parameters");
			}
			else
			{
				//If everything went fine we're connected
				this->connected = true;
				std::cout << "connected\n";
				//We wait 2s as the arduino board will be reseting
				Sleep(ARDUINO_WAIT_TIME);
			}
		}
	}


	// reserve space for reading
	m_vReadBuffer.reserve(DEFAULT_READ_BUFFER_SIZE);
	m_vReadBuffer.push_back('f');
	m_vReadBuffer.push_back('f');

}

Serial::~Serial()
{
	//Check if we are connected before trying to disconnect
	if (this->connected)
	{
		//We're no longer connected
		this->connected = false;
		//Close the serial handler
		printf("[INFO] Closing Serial port\n");
		CloseHandle(this->hSerial);

	}
}


int Serial::readData(std::string& readBuffer)
{
	//Number of bytes we'll have read
	DWORD bytesRead;
	//Number of bytes we'll really ask to read
	unsigned int toRead;

	//Use the ClearCommError function to get status info on the Serial port
	ClearCommError(this->hSerial, &this->errors, &this->status);

	//Check if there is something to read
	if (this->status.cbInQue > 0)
	{
		//If there is we check if there is enough data to read the required number
		//of characters, if not we'll read only the available characters to prevent
		//locking of the application.
		if (this->status.cbInQue > m_cReadBufferSize)
		{
			toRead = m_cReadBufferSize;
		}
		else
		{
			toRead = this->status.cbInQue;
		}

		//Try to read the require number of chars, and return the number of read bytes on success
		if (ReadFile(this->hSerial, &m_vReadBuffer[0], toRead, &bytesRead, NULL) && bytesRead != 0)
		{
			readBuffer += std::string(&m_vReadBuffer[0], bytesRead);
			return bytesRead;
		}

	}

	//If nothing has been read, or that an error was detected return -1
	return -1;

}

bool Serial::writeData(const std::string& writeBuffer)
{
	DWORD bytesSend;

	//Try to write the buffer on the Serial port
	if (!WriteFile(this->hSerial, (void*)writeBuffer.c_str(), writeBuffer.size(), &bytesSend, 0))
	{
		//In case it don't work get comm error and return false
		ClearCommError(this->hSerial, &this->errors, &this->status);

		return false;
	}
	else
		return true;
}

bool Serial::isConnected()
{
	//Simply return the connection status
	return this->connected;
}

std::vector<std::string> Serial::getCommandFromSerial()
{
	std::vector<std::string> msgs;

	if (!connected) return msgs;
	

	std::string receivedStr;
	if (readData(receivedStr) != -1) {
		for (const char& c : receivedStr) {
			if (c == '\n')
			{
				msgs.push_back(msgBuffer);
				msgBuffer = std::string();
			}
			else
			{
				msgBuffer += c;
			}
		}
	}

	return msgs;
}
