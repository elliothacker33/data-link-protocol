// Application layer protocol header.
// NOTE: This file must not be changed.

#ifndef _APPLICATION_LAYER_H_
#define _APPLICATION_LAYER_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// Application layer main function.
// Arguments:
//   serialPort: Serial port name (e.g., /dev/ttyS0).
//   role: Application role {"tx", "rx"}.
//   baudrate: Baudrate of the serial port.
//   nTries: Maximum number of frame retries.
//   timeout: Frame timeout.
//   filename: Name of the file to send / receive.
void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename);

// Get the size of a file
// Arguments:
//  fptr: File pointer
long getFileSize(FILE* fptr);

// Function that returns a control packet
// Arguments:
//  fileSize: File size
//  filename: Name of the file
//  sizePacket: Number of bytes of the control packet
//  control: Number that specifies the control packet
unsigned char* buildControlPacket(long fileSize, const char* filename, int* sizePacket, unsigned char control);

// Function that returns a data packet
// Arguments:
//  fptr: File pointer
//  payload: number of bytes of data
//  s: sequence number of the packet
//  sizeDataPacket: Number of bytes of the packet (number of packet + sequence number + number of bytes payload + data)
unsigned char* buildDataPacket(FILE* fptr, int payload, int s, int* sizeDataPacket);


#endif // _APPLICATION_LAYER_H_
