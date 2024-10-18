// Link layer protocol implementation
#include "link_layer.h"
#include "serial_port.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

LinkLayerRole role;
int timeout;
int nRetransmissions;
int alarmRinging = FALSE;
int alarmCount = 0;

// Definitions

// Results
#define OK 0
#define ERROR -1
// Booleans
#define FALSE 0
#define TRUE 1
// Sizes
#define FRAME_SIZE_S 5
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

typedef enum
{
    START_STATE,
    FLAG_RCV,
    ADDR_RCV,
    CTRL_RCV,
    BCC1_RCV,
    DATA_RCV,
    BCC2_RCV,
    ESC_OCT_RCV,
    STOP_STATE
} State;

void alarmHandler(int signal)
{
    // Alarm enabled means that the alarm got triggered (time set expired)
    alarmRinging = FALSE;
    alarmCount++;
    printf("Alarm #%d\n", alarmCount);
}

void supervisionStateMachine(unsigned char byte, int* state, unsigned char* frame, const Frame* frameParameters){
    switch (*state){
        case START_STATE:
            if (byte == FLAG){
                frame[0] = byte;
                (*state) = FLAG_RCV;
            }
            break;
        case FLAG_RCV:
            if (byte == frameParameters->address){
                frame[1] = byte;
                (*state) = ADDR_RCV;
            }
            else if (byte != FLAG){
                (*state) = START_STATE;
            }
            break;
        case ADDR_RCV:
            if (byte == frameParameters->control){
                frame[2] = byte;
                (*state) = CTRL_RCV;
            }
            else if (byte == FLAG){
                (*state) = FLAG_RCV;
            }
            else{
                (*state) = START_STATE;
            }
            break;
        case CTRL_RCV:
            if (byte == (frame[1]^frame[2])){
                frame[3] = byte;
                (*state) = BCC1_RCV;
            }
            else if (byte == FLAG){
                (*state) = FLAG_RCV;
            }
            else{
                (*state) = START_STATE;
            }
            break;
        case BCC1_RCV:
            if (byte == FLAG){
                frame[4] = byte;
                (*state) = STOP_STATE;
                alarm(0);
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

void buildFrameInfo(unsigned char* frame, unsigned char* data, const Frame* frameParameters){}







////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    int fd = openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate);
    if (fd == ERROR)
        return fd;

    // Define parameters
    role = connectionParameters.role;
    timeout = connectionParameters.timeout;
    nRetransmissions = connectionParameters.nRetransmissions;


    State state = START_STATE;
    unsigned char byte_read = 0;

    struct sigaction sa;
    sa.sa_handler = alarmHandler;
    sa.sa_flags = 0;

    if (sigaction(SIGALRM, &sa, 0) == -1){
        perror("ERROR: Setting signal handler\n");
        exit(ERROR);
    }

    if (role == LlTx)
    {

        while (state != STOP_STATE && alarmCount < nRetransmissions)
        {
            if (alarmRinging == FALSE)
            {
                alarm(timeout); // Set alarm for 3 seconds
                alarmRinging = TRUE;

                // Send the SET message
                unsigned char buf[FRAME_SIZE_S] = {FLAG, A0, SET, A0 ^ SET, FLAG};
                if (writeBytesSerialPort(buf, FRAME_SIZE_S) == -1)
                {
                    perror("Failed to write SET packet");
                    exit(ERROR);
                }
            }

            if (readByteSerialPort(&byte_read) > 0)
            {
                switch (state)
                {
                case START_STATE:
                    if (byte_read == FLAG)
                    {
                        state = FLAG_RCV;
                    }
                    break;
                case FLAG_RCV:
                    if (byte_read == A0)
                        state = ADDR_RCV;
                    else if (byte_read != FLAG)
                        state = START_STATE;
                    break;
                case ADDR_RCV:
                    if (byte_read == UA)
                        state = CTRL_RCV;
                    else if (byte_read == FLAG)
                        state = FLAG_RCV;
                    else
                        state = START_STATE;
                    break;
                case CTRL_RCV:
                    if (byte_read == (A0 ^ UA))
                        state = BCC1_RCV;
                    else if (byte_read == FLAG)
                        state = FLAG_RCV;
                    else
                    {
                        state = START_STATE;
                    }
                    break;
                case BCC1_RCV:
                    if (byte_read == FLAG)
                    {
                        alarm(0);
                        state = STOP_STATE;
                    }
                    else
                        state = START_STATE;
                    break;
                default:
                    state = START_STATE;
                }
            }
        }
        if (state != STOP_STATE){
            perror("Failed to establish connection");
            exit(ERROR);
        }
    }
    else if (connectionParameters.role == LlRx)
    {
        while (state != STOP_STATE)
        {
            if (readByteSerialPort(&byte_read) > 0)
            {
                switch (state)
                {
                case START_STATE:
                    if (byte_read == FLAG)
                    {
                        state = FLAG_RCV;
                    }
                    break;
                case FLAG_RCV:
                    if (byte_read == A0)
                        state = ADDR_RCV;
                    else if (byte_read != FLAG)
                        state = START_STATE;
                    break;
                case ADDR_RCV:
                    if (byte_read == SET)
                        state = CTRL_RCV;
                    else if (byte_read == FLAG)
                        state = FLAG_RCV;
                    else
                        state = START_STATE;
                    break;
                case CTRL_RCV:
                    if (byte_read == (A0 ^ SET))
                        state = BCC1_RCV;
                    else if (byte_read == FLAG)
                        state = FLAG_RCV;
                    else
                    {
                        state = START_STATE;
                    }
                    break;
                case BCC1_RCV:
                    if (byte_read == FLAG)
                    {
                        unsigned char buf[FRAME_SIZE_S] = {FLAG, A0, UA, A0 ^ UA, FLAG};
                        if (writeBytesSerialPort(buf, FRAME_SIZE_S) == -1)
                        {
                            perror("Failed to write UA packet");
                            exit(ERROR);
                        }
                        state = STOP_STATE;
                    }
                    else
                        state = START_STATE;
                    break;
                default:
                    state = START_STATE;
                }
            }
        }
    }
    return fd;
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

//TODO: STATISTICS,change makefile
int llclose(int showStatistics){

     // Signal handling
    struct sigaction sa;
    sa.sa_handler = alarmHandler;
    sa.sa_flags = 0;
    if (sigaction(SIGALRM, &sa, 0) == -1){
        perror("ERROR: Setting signal handler\n");
        exit(ERROR);
    }

    int result;
    int state;
    Frame supervisionFrameSend;
    Frame supervisionFrameReceive;
    alarmCount = 0;
    alarmRinging = FALSE;

    // Allocate memory for the frame buffer
    unsigned char* frameBufferSend = (unsigned char*)malloc(FRAME_SIZE_S * sizeof(unsigned char));
    if (frameBufferSend == NULL) {
        perror("ERROR: Allocating memory for frameBufferSend\n");
        exit(ERROR);
    }
    unsigned char* frameBufferReceive = (unsigned char*)malloc(FRAME_SIZE_S * sizeof(unsigned char));
    if (frameBufferReceive == NULL) {
        perror("ERROR: Allocating memory for frameBufferReceive\n");
        exit(ERROR);
    }


    if (role == LlTx) {

        // Frame Parameters
        // Send DISC
        supervisionFrameSend.type = SUPERVISION;
        supervisionFrameSend.control = DISC;
        supervisionFrameSend.address = A0;
        supervisionFrameSend.bytes = FRAME_SIZE_S;

        // Receive DISC
        supervisionFrameReceive.type = SUPERVISION;
        supervisionFrameReceive.control = DISC;
        supervisionFrameReceive.address = A1;
        supervisionFrameReceive.bytes = FRAME_SIZE_S;

        // Build the send DISC frame
        buildFrameSu(frameBufferSend, &supervisionFrameSend);

        unsigned char buffer_read = 0;
        int byte;
        state = 0;
    

        while (state != STOP_STATE && alarmCount < nRetransmissions) {
            // Sending disc frame
            if (alarmRinging == FALSE){
                // Setting alarm
                alarmRinging = TRUE;
                alarm(timeout);

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

            if (byte > 0) {
                // Process answer
                printf("Byte received: 0x%02X\n", buffer_read);
                processByte(buffer_read, &state, frameBufferReceive, &supervisionFrameReceive);
            }
        }

        if (state != STOP_STATE){
            perror("ERROR:Timeout during receiving DISC\n"); 
            exit(ERROR);
        }

        // Create UA frame
        supervisionFrameSend.control = UA;
        supervisionFrameSend.address = A1;
        buildFrameSu(frameBufferSend,&supervisionFrameSend);

         // Send UA frame
        result = writeBytesSerialPort(frameBufferSend, supervisionFrameSend.bytes);
        if (result == ERROR) {
            perror("Error writing bytes to serial port\n");
            exit(ERROR);
        }
        else{
            printf("UA frame sent\n");
        } 
    }

    else if (role == LlRx){

        // Frame parameters
        supervisionFrameReceive.type = SUPERVISION;
        supervisionFrameReceive.control = DISC;
        supervisionFrameReceive.address = A0;
        supervisionFrameReceive.bytes = FRAME_SIZE_S;

        unsigned char buffer_read = 0;
        int byte;
        state = 0;

        while (state != STOP_STATE){
            // Reading disc frame
            byte = readByteSerialPort(&buffer_read);

            if (byte > 0) {
                // Process answer
                printf("Byte received: 0x%02X\n", buffer_read);
                processByte(buffer_read, &state, frameBufferReceive, &supervisionFrameReceive);
                if (state == STOP_STATE) {
                    break;
                }
            }
            else{
                printf("No byte received\n");
            }
        }

        // Frame parameters
        supervisionFrameSend.type = SUPERVISION;
        supervisionFrameSend.control = DISC;
        supervisionFrameSend.address = A1;
        supervisionFrameSend.bytes = FRAME_SIZE_S;
        buildFrameSu(frameBufferSend, &supervisionFrameSend);


        supervisionFrameReceive.type = SUPERVISION;
        supervisionFrameReceive.control = UA;
        supervisionFrameReceive.address = A1;
        supervisionFrameReceive.bytes = FRAME_SIZE_S;

        state = 0;

        while (state != STOP_STATE && alarmCount < nRetransmissions) {

            // Sending disc frame
            if (!alarmRinging){
                // Setting alarm
                alarmRinging = TRUE;
                alarm(timeout);

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

            if (byte > 0) {
                // Process answer
                printf("Byte received: 0x%02X\n", buffer_read);
                processByte(buffer_read, &state, frameBufferReceive, &supervisionFrameReceive);
            }

        }

        if (state != STOP_STATE){
            perror("ERROR: Timeout during receiving UA\n"); 
            exit(ERROR);
        }

    }

    free(frameBufferSend);
    free(frameBufferReceive);
    int clstat = closeSerialPort();
    return clstat;
}






