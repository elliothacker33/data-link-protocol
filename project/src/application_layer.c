// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// Definitions

// Results
#define OK 0
#define ERROR -1
// Booleans
#define FALSE 0
#define TRUE 1
// Control Field
#define START 1
#define DATA 2
#define END 3
// T value
#define FILE_SIZE 0
#define FILE_NAME 1

size_t getFileSize(FILE* fptr){
    int result;
    result = fseek(fptr, 0, SEEK_END);
    if (result == ERROR) {
        perror("Error: Unable to seek to end of file");
        exit(ERROR);
    }

    int fileSize = ftell(fptr);
    if (fileSize == ERROR){
        perror("Error: Unable to obtain file size");
        exit(ERROR);
    }
    rewind(fptr);
    return (size_t) fileSize;
}

unsigned char* buildControlPacket(int fileSize, const char* filename, size_t* sizePacket, int control){

    size_t l1 = (fileSize > 0) ? (int)(log(fileSize) / log(256)) + 1 : 1;
    size_t l2 = strlen(filename) + 1;
    (*sizePacket) = 5 + l1 + l2;

    unsigned char* controlPacket = (unsigned char*)malloc((*sizePacket) * sizeof(unsigned char));
    if (controlPacket == NULL){
        perror("Error: Unable to allocate memory");
        exit(ERROR);
    }

    controlPacket[0] = control; // C
    controlPacket[1] = FILE_SIZE; // T1
    controlPacket[2] = l1; // L1

    //V1
    for (int i = 0; i < l1; i++){
        controlPacket[3 + i] = (fileSize >> (8 * i)) & 0xFF;
    }

    controlPacket[3 + l1] = FILE_NAME; //T2
    controlPacket[4 + l1] = l2; //L2

    //V2
    for (int i = 0; i < l2; i++){
        controlPacket[5 + l1 + i] = filename[i];
    }
    return controlPacket;
}

unsigned char* buildDataPacket(FILE* fptr, int payload, size_t s, size_t* sizeDataPacket){

    (*sizeDataPacket) = payload + 4;
    unsigned char* dataPacket = (unsigned char*)malloc((*sizeDataPacket) * sizeof(unsigned char));

    if (dataPacket == NULL){
        perror("Error: Unable to allocate memory");
        exit(ERROR);
    }

    dataPacket[0] = DATA;
    dataPacket[1] = (unsigned char) s;
    dataPacket[2] = (unsigned char) (payload / 256);
    dataPacket[3] = (unsigned char) (payload % 256);

    for (int i = 0; i < payload; i++){
        int byte = fgetc(fptr);

        if (byte == EOF){
            perror("Error: Unexpected end of file");
            exit(ERROR);
        }
    
        dataPacket[4 + i] = (unsigned char) byte;
    }
    return dataPacket;
}


void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{   
    // Ask for statistics
    /*
    int showStats;
    printf("0 - hide statistics 1 - show statistics: ");
    scanf("%d", &showStats);
    */

    // Create Link Layer settings
    LinkLayer openConnection;
    strcpy(openConnection.serialPort, serialPort); // serial port
    openConnection.baudRate = baudRate; // baud rate
    openConnection.nRetransmissions = nTries; // number of retries
    openConnection.timeout = timeout; // timeout 
    int fd;


    if ((*role) == 't'){

        // Open connection
        openConnection.role = LlTx;
        fd = llopen(openConnection);
        if (fd == ERROR){
            perror("ERROR: Error opening connection");
            exit(ERROR);
        }

        // Open file 
        FILE* fPtr = fopen(filename, "rb");
        if (fPtr == NULL){
            perror("ERROR: Error opening file");
            exit(ERROR);
        }

        // File size
        size_t fileSize = getFileSize(fPtr);

        // Start packet
        size_t sizeStartPacket;
        unsigned char* startPacket = buildControlPacket(fileSize, filename, &sizeStartPacket,START);
        if (llwrite(startPacket, sizeStartPacket) == ERROR){
            perror("ERROR: Error sending start packet");
            exit(ERROR);
        }
        free(startPacket);

        // Data packets
        size_t s = 0; // Sequence number
        size_t usePayload;
        size_t sendPayload = fileSize;
        size_t sizeDataPacket;

        while (sendPayload > 0) {
            if (sendPayload > MAX_PAYLOAD_SIZE){
                usePayload = MAX_PAYLOAD_SIZE;
            }
            else{
                usePayload = sendPayload;
            }

            // Data packet
            unsigned char* dataPacket = buildDataPacket(fPtr, usePayload, s, &sizeDataPacket);
            if (llwrite(dataPacket, sizeDataPacket) == ERROR){
                perror("ERROR: Error sending data packet");
                exit(ERROR);
            }
            free(dataPacket);

            s++;
            if (s > 255){
                s = 0;
            }
            sendPayload -= usePayload;
        }

        // End packet
        size_t sizeEndPacket;
        unsigned char* endPacket = buildControlPacket(fileSize, filename, &sizeEndPacket,END);
        if (llwrite(endPacket, sizeEndPacket) == ERROR){
            perror("ERROR: Error sending start packet");
            exit(ERROR);
        }
        free(endPacket);

        // Close file
        int result = fclose(fPtr);
        if (result == ERROR){
            perror("ERROR: Error closing file");
            exit(ERROR);
        }


        // Close connection
        int closeConnection;
        closeConnection = llclose(0);
        if (closeConnection == ERROR){
            perror("ERROR: Error opening connection");
            exit(ERROR);
        }

    }
    else {
        openConnection.role = LlRx;
        fd = llopen(openConnection);

        if (fd == -1){
            perror("Error: Error opening connection");
            exit(-1);
        }

        fd = llclose(0);
        if (fd == -1){
            perror("Error: Error opening connection");
            exit(-1);
        }


    }

}
