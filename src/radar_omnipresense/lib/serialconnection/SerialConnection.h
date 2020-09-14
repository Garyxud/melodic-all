/* Copyright 2018 Ryan Cooper (RyanLoringCooper@gmail.com)
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
* The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
#ifndef SERIALCONNECTION_H
#define SERIALCONNECTION_H

#include <thread>
#include <chrono>
#include <cstring>
#include <string>
#include <mutex>
#include <condition_variable>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <cstdio>

#ifndef _MAX_DATA_LENGTH
    // size of the buffer that is filled when a read is preformed
    #define _MAX_DATA_LENGTH 4096
#endif

#ifndef _BUFFER_SIZE
    // size of the circular buffer that the user is served data from
    // 4194304 = 2^22 = 4MB
    #define _BUFFER_SIZE 4194304
#endif

class SerialConnection {
protected:

	// a circular buffer that holds the data read from a connection until the user requests it
    char *buffer;

    // indexes related to buffer. The readIndex cannot pass the writeIndex.
    // readIndex is the next index to be read, writeIndex is the next index to be written
    int readIndex, writeIndex;

    // the amount of time preformReads() will wait before trying to read again
    // if less than 0, it will block indefinitely
    // if equal to 0, it will never block and will continuously try to read data
    // if more than 0, it will wait that long in milliseconds
    int blockingTime;

    // flag to indicate if the connection is connected. This is not always useful, such as with serial devices
    volatile bool connected;

    // flag to indicate whether to program has been asked to stop reading and terminate
    volatile bool interruptRead;

    // flag that allows for waitForData() to be notified when there is new data
    volatile bool cvBool;

    // flag to indicate if this CommConnection is never going to read data from its connection
    bool noReads;

    // flags related to whether the reading thread is running
    bool begun, terminated;

    // flag to indicate whether debugging messages should be displayed
    bool debug;

    // does the reading from the connection by running performReads()
    std::thread *readThread;

    // the mutex that prevents waitForData() from prematurely telling the user there is data
    std::mutex dataMutex;

    // the condition variable that uses the above mutex
    std::condition_variable cv;

    /* calls getData(2) and fillBuffer(2)
     * is the function executed by readThread
     * sets cv when there is new data
     */
    void performReads();

    /* fills buffer with the data that is in buff and moves the writeIndex forward by bytesRead amount
     * @param buff
     *     an array of characters to put into buffer starting at the writeIndex
     * @param bytesRead
     *     the amount of bytes stored in buff
     */
    void fillBuffer(char *buff, const int &bytesRead);

    // attempts to stop readThread and destroy it
    void closeThread();

    /* Reads the bytes between start and end in buffer
     * If end is before start, it will wrap around the buffer
     * this will also update readIndex
     * @param start
     *      the location to start reading from
     * @param end
     *      the location of the last byte to be put into the return value
     * @retval a null terminated c string that contains the bytes in buffer between start and end
     */
    char *readRange(const int &start, const int &end); 

	int ser;
	int set_interface_attribs (const int &speed, const int &parity); 
	int set_blocking (const bool &should_block);

	// a child class may attempt to restart the connection with this function as it is called when getData failes
	void failedRead();
	/* the function that the child class implements to do the reading of the data from the connection
     * @param buff
     *      the buffer to be filled by the child
     * @param buffSize
     *      the size of buff
     * @retval is the number of bytes written to buff
     */
	int getData(char *buff, const int &buffSize);
	void exitGracefully();
    /* allows the child to implement how blocking is done for its connection
     * called by the constructor
     * @param blockingTime
     *      the amount of time in milliseconds to block for a read call if data is not available when fillBuffer(2) is called
     * @retval whether it was successful in setting how long to block for
     */
	bool setBlocking(const int &blockingTime = -1);
public:
	SerialConnection(const char *portName, const int &speed, const int &parity, const int &blockingTime = -1, const bool &debug = false, const bool &noReads = false);
	SerialConnection(const SerialConnection &other);
	SerialConnection &operator=(const SerialConnection &other);
	~SerialConnection();
	// starts the readThread
    bool begin();

    // returns how many bytes are available to be read from the buffer immediately
    long long available() const;

    /* blocks until there is a byte to be read from the buffer
     * @retval the number of bytes that are available
     */
    int waitForData();

	/* blocks until the delimiter is found
     * @param delim
     * 		the character to wait for the connection to send
     * @retval the number of bytes to be read until the delimitor is found
     */
    int waitForDelimitor(const char &delim);

    /* returns 1 byte from the buffer if one is available and moves readIndex up by 1
     * if no byte is available, then it returns 0
     * @retval the byte read from buffer, or 0 if no byte is available
     */
    char read();

    /* fills buff with bytesToRead number of bytes and moves readIndex up by bytesToRead amount
     * @param buff 
     *      must be allocated by the caller 
     *      is left untouched if no bytes are available to be read
     * @param bytesToRead
     *      the number of bytes to be copied into buff
     *      this number of bytes won't be read if there aren't that many available
     */
    void read(char *buff, const long long &bytesToRead);

    /* Reads until delim is read, until maxBytes is read, or there are no 
     * more bytes to read from the buffer
     * @param buff
     *      should be an unallocated reference to a char *
     *      it will be allocated on the heap by this function 
     *      and filled with the bytes read from buffer
     *      it will also be null terminated
     *      if buffer[readIndex] == delim, then buff will be a reference to NULL
     * @param delim
     *      the character to be read until
     * @param maxBytes
     *      if set to -1, this program will read until the delim is read or there are no more bytes to read
     * @param includeDelim
     *      determines whether the delim will be included in the buffer or not
     * @retval the number of bytes put into buff not including the null terminator
     */
    int readUntil(char **buff, const char &delim, const long long &maxBytes = -1, const bool &includeDelim = false);

    /* returns a string with bytesToRead number of characters if that many bytes can be read
     * if no argument is provided to this function, the string that is returned has all the bytes that are in buffer
     * it will move readIndex up by the number of bytes it put into the string
     * @param bytesToRead
     *      the number of bytes to read from buffer
     *      if zero, all available bytes will be read
     *      if bytesToRead is greater than the number of available bytes, then this will return an empty string
     * @retval a string that holds all the bytes read from buffer      
     */
    std::string readString(const long long &bytesToRead = 0);

    // returns connected
    bool isConnected() const;

    // sets readIndex = writeIndex
    void clearBuffer();	

    /* shuts down the connection, and attempts to terminate the read thread
     * calls exitGracefully()
     * called by the destructor
     */
    void terminate();

    // calls write(2)
    bool write(const std::string &buff);
	// returns false and sets errno upon error
	bool write(const char *buff, const int &buffSize);
};

#endif // SERIALPORT_H
