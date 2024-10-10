// Link layer protocol implementation
#include "link_layer.h"
#include "serial_port.h"
#include <stdio.h>
#include <stdlib.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

LinkLayerRole role;
int timeout;
int nRetransmissions;
int alarmCount = 0;
int alarmEnabled = FALSE;

// Definitions

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
    enum FrameType type;
    unsigned char control;
    unsigned char address;
    int bytes;
} Frame;

// Alarm function handler
void alarmHandler(int signal) {
    alarmEnabled = FALSE;
    alarmCount++;
    printf("Alarm #%d\n", alarmCount);
}

/**
 * @brief Build supervision frames;
 * @param frame - pointer to frame
 * @param frameParameters - frame parameters (control,type,address,bytes).
 */
void buildFrameSu(unsigned char* frame, const Frame* frameParameters){
    frame[0] = FLAG;
    frame[1] = frameParameters.address;
    frame[2] = frameParameters.control;
    frame[3] = frameParameters.address ^ frameParameters.control;
    frame[4] = FLAG;
}
/**
 * @brief Build Information frames
 * @param frame
 */

/**
 * @brief Build frame structure by type
 * @param frame
 * @param frameParameters
 */
void buildFrame(unsigned char* frame, const Frame* frameParameters){
    // Checking type of frame
    if (frameParameters.type != INFO){
        // Supervision / Unnumbered frames
        buildFrameSu(frame, frameParameters);
    }
    else{

    }
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

    // Setting alarmHandler
    (void)signal(SIGALRM,alarmHandler);
    int response;
    Frame frameParameters;
    unsigned char* frame;

    if (role == LlTX){
        // Build Frame
        frame = (unsigned char*)malloc(5 * sizeof(unsigned char));
        if (frame == NULL){
            perror("Error allocating memory for frame");
            exit(-1);
        }

        // Build  Frame
        frameParameters.type = SUPERVISION;
        frameParameters.control = DISC;
        frameParameters.bytes = 5;
        response = buildFrame(frame, &frameParameters);
        if (response == -1){
            perror("Error during frame building");
            exit(-1);
        }

        // Sending Disc Frame
        while (response != OK && alarmCount < nRetransmissions) {
            if (!alarmEnabled){
                alarmEnabled = TRUE;
                alarm(timeout);
                response = writeBytesSerialPort(frame, frameParameters.bytes);
                if (response == -1){
                    perror("Error during writing bytes to Serial Port");
                    exit(-1);
                }
            }
        }



    }
    else if (role == LlRx){
        // Recetor

    }

    free(frame);
    int clstat = closeSerialPort();
    return clstat;
}






