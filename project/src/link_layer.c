// Link layer protocol implementation
#include "link_layer.h"
#include "serial_port.h"
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

LinkLayerRole role;
int timeout;
int nRetransmissions;
int alarmCount = 0;
int alarmEnabled = FALSE;

// Definitions

// Results
#define OK 0
#define ERROR -1
// Booleans
#define FALSE 0
#define TRUE 1

// Flag
#define FLAG 0x7E // Initial and Final delimiter flag

// Address
#define A0 0x03 // Sender
#define A1 0x01 // Receiver

// Control
#define SET 0x03
#define UA 0x07
#define RR0 0xAA
#define RR1 0xAB
#define REJ0 0x54
#define REJ1 0x55
#define DISC 0x0B

// Byte stuffing
#define ESC 0x7D


typedef enum {
    SUPERVISION,
    INFO
} FrameType;

typedef struct {
    FrameType type;
    unsigned char control;
    unsigned char address;
    int bytes;
} Frame;

enum State {
    START_STATE,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    STOP_STATE
};

// Alarm function handler
void alarmHandler(int signal) {
    alarmEnabled = FALSE;
    alarmCount++;
    printf("Alarm #%d\n", alarmCount);
}

void supervisionStateMachine(unsigned char byte, int* state, unsigned char* frame, const Frame* frameParameters){
    printf("Supervision mode\n");
    switch (*state){
        case START_STATE:
            if (byte == FLAG){
                frame[0] = byte;
                (*state) = FLAG_RCV;
            }
            break;
        case FLAG_RCV:
            if (byte == frameParameters->control){
                frame[1] = byte;
                (*state) = A_RCV;
            }
            else if (byte != FLAG){
                (*state) = START_STATE;
            }
            break;
        case A_RCV:
            if (byte == frameParameters->address){
                frame[2] = byte;
                (*state) = C_RCV;
            }
            else if (byte == FLAG){
                (*state) = FLAG_RCV;
            }
            else{
                (*state) = START_STATE;
            }
            break;
        case C_RCV:
            if (byte == (frame[1]^frame[2])){
                frame[3] = byte;
                (*state) = BCC_OK;
            }
            else if (byte == FLAG){
                (*state) = FLAG_RCV;
            }
            else{
                (*state) = START_STATE;
            }
            break;
        case BCC_OK:
            if (byte == FLAG){
                frame[4] = byte;
                (*state) = STOP_STATE;
            }
            else{
                (*state) = START_STATE;
            }
            break;
        default:
            (*state) = START_STATE;
    }
}


// TODO: informationStateMachine

void processByte(unsigned char byte, int* state, unsigned char* frame, const Frame* frameParameters){

    if (frameParameters->type == SUPERVISION){
        // Process Supervision frame
        supervisionStateMachine(byte, state, frame, frameParameters);
    }
    else if (frameParameters->type == INFO){
        // Process INFO frame
        // TODO:
    }
}

void buildFrameSu(unsigned char* frame, const Frame* frameParameters){
    frame[0] = FLAG;
    frame[1] = frameParameters->address;
    frame[2] = frameParameters->control;
    frame[3] = frameParameters->address ^ frameParameters->control;
    frame[4] = FLAG;
}



int buildFrame(unsigned char* frame, const Frame* frameParameters){
    // Checking type of frame
    if (frameParameters->type != INFO){
        // Supervision / Unnumbered frames
        buildFrameSu(frame, frameParameters);
    }
    else{
        buildFrameInfo(frame,frameParameters);
    }
    return 0;
}




////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    if (openSerialPort(connectionParameters.serialPort,connectionParameters.baudRate) < 0){
        return -1;
    }



    return 1;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // TODO

    return 0;
}


int llclose(int showStatistics){

    (void)signal(SIGALRM, alarmHandler);
    int result;
    int state;
    Frame supervisionFrameSend;
    Frame supervisionFrameReceive;

    // Allocate memory for the frame buffer
    unsigned char* frameBufferSend = (unsigned char*)malloc(5 * sizeof(unsigned char));
    if (frameBufferSend == NULL) {
        perror("Error allocating memory for frameBuffer\n");
        exit(ERROR);
    }
    unsigned char* frameBufferReceive = (unsigned char*)malloc(5 * sizeof(unsigned char));
    if (frameBufferReceive == NULL) {
        perror("Error allocating memory for frameBuffer\n");
        exit(ERROR);
    }


    if (role == LlTx) {
        // Frame Parameters
        // Send DISC
        supervisionFrameSend.type = SUPERVISION;
        supervisionFrameSend.control = DISC;
        supervisionFrameSend.address = A0;
        supervisionFrameSend.bytes = 5;

        // Receive DISC
        supervisionFrameReceive.type = SUPERVISION;
        supervisionFrameReceive.control = DISC;
        supervisionFrameReceive.address = A1;
        supervisionFrameReceive.bytes = 5;

        // Build the send DISC frame
        result = buildFrame(frameBufferSend, &supervisionFrameSend);
        if (result == ERROR) {
            perror("Error during frame building\n");
            exit(ERROR);
        }

        unsigned char buffer_read = 0;
        int byte = 0;
        state = 0;

        while (state != STOP_STATE && alarmCount < nRetransmissions) {

            // Sending disc frame
            if (!alarmEnabled){
                // Setting alarm
                alarmEnabled = TRUE;
                alarm(timeout);

                printf("%dº alarm\n", alarmCount);
                result = writeBytesSerialPort(frameBufferSend, supervisionFrameSend.bytes);
                if (result == ERROR) {
                    perror("Error writing bytes to serial port\n");
                    exit(ERROR);
                }
                else{
                    printf("DISC frame sent\n");
                }
            }

            // Reading disc frame
            byte = readByteSerialPort(&buffer_read);
            if (byte == ERROR){
                perror("Error reading byte from serial port\n");
                exit(ERROR);
            }

            if (byte > 0) {
                // Process answer
                printf("Byte received: 0x%02X\n", buffer_read);
                processByte(buffer_read, &state, frameBufferReceive, &supervisionFrameReceive);
            }
            else{
                printf("No byte received\n");
            }
        }

        // Create UA frame
        supervisionFrameSend.control = UA;
        buildFrame(frameBufferSend,&supervisionFrameSend);

        // Send UA frame
        result = writeBytesSerialPort(frameBufferSend, supervisionFrameSend.bytes);
        if (result == ERROR) {
            perror("Error writing bytes to serial port\n");
            exit(ERROR);
        }

    }
    else if (role == LlRx){

        // Frame parameters
        supervisionFrameReceive.type = SUPERVISION;
        supervisionFrameReceive.control = DISC;
        supervisionFrameReceive.address = A0;
        supervisionFrameReceive.bytes = 5;

        unsigned char buffer_read = 0;
        int byte = 0;
        state = 0;

        while (state != STOP_STATE){
            // Reading disc frame
            byte = readByteSerialPort(&buffer_read);
            if (byte == ERROR){
                perror("Error reading byte from serial port\n");
                exit(ERROR);
            }

            if (byte > 0) {
                // Process answer
                printf("Byte received: 0x%02X\n", buffer_read);
                processByte(buffer_read, &state, frameBufferReceive, &supervisionFrameReceive);
            }
            else{
                printf("No byte received\n");
            }
        }

        // Frame parameters
        supervisionFrameSend.type = SUPERVISION;
        supervisionFrameSend.control = DISC;
        supervisionFrameSend.address = A1;
        supervisionFrameSend.bytes = 5;

        supervisionFrameReceive.type = SUPERVISION;
        supervisionFrameReceive.control = UA;
        supervisionFrameReceive.address = A1;
        supervisionFrameReceive.bytes = 5;

        state = 0;

        while (state != STOP_STATE && alarmCount < nRetransmissions) {

            // Sending disc frame
            if (!alarmEnabled){
                // Setting alarm
                alarmEnabled = TRUE;
                alarm(timeout);

                printf("%dº alarm\n", alarmCount);
                result = writeBytesSerialPort(frameBufferSend, supervisionFrameSend.bytes);
                if (result == ERROR) {
                    perror("Error writing bytes to serial port\n");
                    exit(ERROR);
                }
                else{
                    printf("DISC frame sent\n");
                }
            }

            // Reading UA frame
            byte = readByteSerialPort(&buffer_read);
            if (byte == ERROR){
                perror("Error reading byte from serial port\n");
                exit(ERROR);
            }

            if (byte > 0) {
                // Process answer
                printf("Byte received: 0x%02X\n", buffer_read);
                processByte(buffer_read, &state, frameBufferReceive, &supervisionFrameReceive);
            }
            else{
                printf("No byte received\n");
            }
        }
    }

    free(frameBufferSend);
    free(frameBufferReceive);
    int clstat = closeSerialPort();
    return clstat;
}






