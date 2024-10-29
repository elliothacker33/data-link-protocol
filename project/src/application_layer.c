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
#define START 0x01
#define DATA 0x02
#define END 0x03
// T value
#define FILE_SIZE 0x00
#define FILE_NAME 0x01

long getFileSize(FILE* fptr){

    if ((fseek(fptr, 0, SEEK_END)) == ERROR) {
        perror("ERROR: Unable to seek to end of file\n");
        exit(ERROR);
    }

    long fileSize = ftell(fptr);
    if (fileSize == ERROR){
        perror("ERROR: Unable to obtain file size\n");
        exit(ERROR);
    }
    rewind(fptr);
    return fileSize;
}

unsigned char* buildControlPacket(long fileSize, const char* filename, int* sizePacket, unsigned char control){

    int l1 = (fileSize > 0) ? (int)(log2(fileSize) / log2(256)) + 1 : 1;
    int l2 = strlen(filename) + 1;
    (*sizePacket) = 5 + l1 + l2;

    unsigned char* controlPacket = (unsigned char*)malloc((*sizePacket) * sizeof(unsigned char));
    if (controlPacket == NULL){
        perror("ERROR: Unable to allocate memory\n");
        exit(ERROR);
    }

    controlPacket[0] = control; // C
    controlPacket[1] = FILE_SIZE; // T1
    controlPacket[2] = (unsigned char) l1; // L1

    //V1
    for (int i = 0 ; i < l1; i++){
        controlPacket[3 + i] = (fileSize >> (8 * i)) & 0xFF;
    }

    controlPacket[3 + l1] = FILE_NAME; //T2
    controlPacket[4 + l1] = (unsigned char) l2; //L2

    //V2
    for (int i = 0; i < l2; i++){
        controlPacket[5 + l1 + i] = filename[i];
    }

    return controlPacket;
}

unsigned char* buildDataPacket(FILE* fptr, int payload, int s, int* sizeDataPacket){

    (*sizeDataPacket) = payload + 4;
    unsigned char* dataPacket = (unsigned char*)malloc((*sizeDataPacket) * sizeof(unsigned char));

    if (dataPacket == NULL){
        perror("ERROR: Unable to allocate memory\n");
        exit(ERROR);
    }

    dataPacket[0] = DATA;
    dataPacket[1] = (unsigned char) s;
    dataPacket[2] = (unsigned char) (payload >> 8) & 0xFF;
    dataPacket[3] = (unsigned char) payload & 0xFF;

    for (int i = 0; i < payload; i++){
        int byte = fgetc(fptr);

        if (byte == EOF){
            perror("ERROR: Unexpected end of file\n");
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
    
    // int showStats;
    // printf("0 - hide statistics 1 - show statistics: ");
    // scanf("%d", &showStats);
   

    // Create Link Layer settings
    LinkLayer openConnection;
    strcpy(openConnection.serialPort, serialPort); // serial port
    openConnection.baudRate = baudRate; // baud rate
    openConnection.nRetransmissions = nTries; // number of retries
    openConnection.timeout = timeout; // timeout 
    int fd;


    if (strcmp(role, "tx") == 0){

        // Open connection
        openConnection.role = LlTx;
        fd = llopen(openConnection);
        if (fd == ERROR){
            perror("ERROR: Error opening connection\n");
            exit(ERROR);
        }

        // Open file 
        FILE* fPtr = fopen(filename, "rb");
        if (fPtr == NULL){
            perror("ERROR: Error opening file\n");
            exit(ERROR);
        }

        // File size
        long fileSize = getFileSize(fPtr);

        // Start packet
        int sizeStartPacket = 0;
        unsigned char* startPacket = buildControlPacket(fileSize, filename, &sizeStartPacket,START);
        if (llwrite(startPacket, sizeStartPacket) == ERROR){
            perror("ERROR: Error sending start packet\n");
            exit(ERROR);
        }
        free(startPacket);

        // Data packets
        int s = 0; // Sequence number
        int usePayload;
        int sendPayload = fileSize;
        int sizeDataPacket;

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
                perror("ERROR: Error sending data packet\n");
                exit(ERROR);
            }

            free(dataPacket);

            s++;
            if (s > 99){
                s = 0;
            }
            sendPayload -= usePayload;
        }

        // End packet
        int sizeEndPacket = 0;
        unsigned char* endPacket = buildControlPacket(fileSize, filename, &sizeEndPacket,END);
        if (llwrite(endPacket, sizeEndPacket) == ERROR){
            perror("ERROR: Error sending start packet\n");
            exit(ERROR);
        }
        free(endPacket);

        // Close file
        int result = fclose(fPtr);
        if (result == ERROR){
            perror("ERROR: Error closing file\n");
            exit(ERROR);
        }


        // Close connection
        int closeConnection;
        closeConnection = llclose(1);
        if (closeConnection == ERROR){
            perror("ERROR: Error closing connection\n");
            exit(ERROR);
        }

    }
    else if (strcmp(role, "rx") == 0) {

        openConnection.role = LlRx;
        fd = llopen(openConnection);

        if (fd == ERROR) {
            perror("Error: Error opening connection");
            exit(ERROR);
        }

        unsigned char* packet = (unsigned char*) malloc((MAX_PAYLOAD_SIZE + 4)*sizeof(unsigned char));
        if (packet == NULL) {
            perror("ERROR: Unable to allocate memory");
            exit(ERROR);
        }

        while (TRUE){
            int size = llread(packet);
            if (size > 0){
                if (packet[0] != START){
                    perror("Unexpected START packet");
                    free(packet);
                    exit(-1);
                }
                break;
            }
        }

        FILE* file = fopen(filename, "wb");

        if (file == NULL){
            perror("ERROR: Error opening file\n");
            exit(ERROR);
        }

        while (TRUE) {
            int size = llread(packet);

            if (packet[0] == END){
                break;
            }
            if (size > 0){
                fwrite(packet+4, sizeof(unsigned char), size-4, file);
            }
        }

        fclose(file);

        fd = llclose(1);
        if (fd == -1){
            perror("Error: Error clsosing connection");
            exit(-1);
        }
        
    }

}
