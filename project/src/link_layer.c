#define _POSIX_SOURCE 1 // POSIX compliant source
// Link layer protocol implementation
#include "link_layer.h"
#include "serial_port.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>

// MISC

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

#define C_IF(Nr) (Nr << 6)

// Control
#define SET 0x03
#define UA 0x07
#define RR(Nr) (0xAA | Nr)
#define REJ(Nr) (0x54 | Nr)
#define DISC 0x0B

// Byte stuffing
#define ESC 0x7D


typedef enum FrameType {
    SUPERVISION,
    INFO
} FrameType;

typedef struct Frame {
    FrameType type;
    unsigned char control;
    unsigned char address;
    int bytes;
} Frame;

typedef enum State
{
    WAITING_FLAG,
    WAITING_ADDR,
    WAITING_CTRL,
    WAITING_BCC1,
    WAITING_FLAG2,
    WAITING_DATA,
    WAITING_BCC2,
    ESC_OCT_RCV,
    STOP_STATE
} State;

unsigned char Ns = 1;

void alarmHandler(int signal)
{
    // Alarm enabled means that the alarm got triggered (time set expired)
    alarmRinging = FALSE;
    alarmCount++;
    printf("Alarm #%d\n", alarmCount);
}

void supervisionStateMachine(unsigned char byte, int* state, unsigned char* frame, const Frame* frameParameters){
    switch (*state){
        case WAITING_FLAG:
            if (byte == FLAG){
                frame[0] = byte;
                (*state) = WAITING_ADDR;
            }
            break;
        case WAITING_ADDR:
            if (byte == frameParameters->address){
                frame[1] = byte;
                (*state) = WAITING_CTRL;
            }
            else if (byte != FLAG){
                (*state) = WAITING_FLAG;
            }
            break;
        case WAITING_CTRL:
            if (byte == frameParameters->control){
                frame[2] = byte;
                (*state) = WAITING_BCC1;
            }
            else if (byte == FLAG){
                (*state) = WAITING_ADDR;
            }
            else{
                (*state) = WAITING_FLAG;
            }
            break;
        case WAITING_BCC1:
            if (byte == (frame[1]^frame[2])){
                frame[3] = byte;
                (*state) = WAITING_FLAG2;
            }
            else if (byte == FLAG){
                (*state) = WAITING_ADDR;
            }
            else{
                (*state) = WAITING_FLAG;
            }
            break;
        case WAITING_FLAG2:
            if (byte == FLAG){
                frame[4] = byte;
                (*state) = STOP_STATE;
                alarm(0);
            }
            else{
                (*state) = WAITING_FLAG;
            }
            break;
        default:
            (*state) = WAITING_FLAG;
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


    State state = WAITING_FLAG;
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
                case WAITING_FLAG:
                    if (byte_read == FLAG)
                    {
                        state = WAITING_ADDR;
                    }
                    break;
                case WAITING_ADDR:
                    if (byte_read == A0)
                        state = WAITING_CTRL;
                    else if (byte_read != FLAG)
                        state = WAITING_FLAG;
                    break;
                case WAITING_CTRL:
                    if (byte_read == UA)
                        state = WAITING_BCC1;
                    else if (byte_read == FLAG)
                        state = WAITING_ADDR;
                    else
                        state = WAITING_FLAG;
                    break;
                case WAITING_BCC1:
                    if (byte_read == (A0 ^ UA))
                        state = WAITING_FLAG2;
                    else if (byte_read == FLAG)
                        state = WAITING_ADDR;
                    else
                    {
                        state = WAITING_FLAG;
                    }
                    break;
                case WAITING_FLAG2:
                    if (byte_read == FLAG)
                    {
                        alarm(0);
                        state = STOP_STATE;
                    }
                    else
                        state = WAITING_FLAG;
                    break;
                default:
                    state = WAITING_FLAG;
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
                case WAITING_FLAG:
                    if (byte_read == FLAG)
                    {
                        state = WAITING_ADDR;
                    }
                    break;
                case WAITING_ADDR:
                    if (byte_read == A0)
                        state = WAITING_CTRL;
                    else if (byte_read != FLAG)
                        state = WAITING_FLAG;
                    break;
                case WAITING_CTRL:
                    if (byte_read == SET)
                        state = WAITING_BCC1;
                    else if (byte_read == FLAG)
                        state = WAITING_ADDR;
                    else
                        state = WAITING_FLAG;
                    break;
                case WAITING_BCC1:
                    if (byte_read == (A0 ^ SET))
                        state = WAITING_FLAG2;
                    else if (byte_read == FLAG)
                        state = WAITING_ADDR;
                    else
                    {
                        state = WAITING_FLAG;
                    }
                    break;
                case WAITING_FLAG2:
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
                        state = WAITING_FLAG;
                    break;
                default:
                    state = WAITING_FLAG;
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

    // State machine
    State state = WAITING_FLAG;
    unsigned char *byte_read = 0;
    unsigned char received_IF = 0;
    int byte_nr = 0;
    while (state != STOP_STATE) {
        if (readByteSerialPort(byte_read) > 0) {
            switch (state) {
                case WAITING_FLAG:
                    if (*byte_read == FLAG) {
                        state = WAITING_ADDR;
                    }
                    break;
                case WAITING_ADDR:
                    if (*byte_read == A0) {
                        state = WAITING_CTRL;
                    } else if (*byte_read == FLAG) {
                        state = WAITING_ADDR;
                    } else {
                        state = WAITING_FLAG;
                    }
                    break;
                case WAITING_CTRL:
                    // DUVIDA: Se eu receber o frame nao esperado, é um duplicado? E transmitter guarda o frame anterior ao enviado,
                    // e tem capacidade para enviá-lo caso receba um rej?. i.e o q fazer se no receiver eu receber um Information frame number 
                    // diferente do esperado?? 
                    if (*byte_read == C_IF(0) || *byte_read == C_IF(1)) {
                        received_IF = *byte_read;
                        state = WAITING_BCC1;
                    } else if (*byte_read == FLAG) {
                        state = WAITING_ADDR;
                    } else {
                        state = WAITING_FLAG;
                    }
                    break;
                case WAITING_BCC1:
                    if (*byte_read == (A0 ^ C_IF(0)) || *byte_read == (A0 ^ C_IF(1))) {
                        state = WAITING_DATA;
                    } else if (*byte_read == FLAG) {
                        state = WAITING_ADDR;
                    } else {
                        state = WAITING_FLAG;
                    }
                    break;
                case WAITING_DATA:
                    if (*byte_read == ESC) {
                        state = ESC_OCT_RCV;
                    }
                    else if (*byte_read == FLAG) {

                        unsigned char bcc2_rcv = packet[byte_nr - 1];
                        unsigned char bcc2_actual = 0;

                        packet[byte_nr - 1] = '\0';

                        for (size_t i = 0; i < byte_nr; i++)
                        {
                            bcc2_actual = bcc2_actual ^ packet[i];
                        }

                        if (bcc2_actual == bcc2_rcv)
                        {
                            // Frame received, ready to receive next frame
                            if (C_IF(Ns) == received_IF) {
                                state = STOP_STATE;
                                Ns ^= 1;

                                unsigned char* frame = NULL;
                                Frame supervisionFrameReceive = {SUPERVISION, RR(Ns), A0, FRAME_SIZE_S};
                                buildFrameSu(frame, &supervisionFrameReceive);
                                if (writeBytesSerialPort(frame, FRAME_SIZE_S) == -1)
                                {
                                    perror("Failed to write RR packet");
                                    exit(ERROR);
                                }
                                return byte_nr;
                            }

                            // Duplicate frame, discarding
                            else
                            {
                                state = STOP_STATE;

                                unsigned char* frame = NULL;
                                Frame supervisionFrameReceive = {SUPERVISION, RR(Ns), A0, FRAME_SIZE_S};
                                buildFrameSu(frame, &supervisionFrameReceive);
                                if (writeBytesSerialPort(frame, FRAME_SIZE_S) == -1)
                                {
                                    perror("Failed to write RR packet");
                                    exit(ERROR);
                                }
                                // ignore packet received till now
                                return 0;
                            }
                        }
                        // BCC2 error, rejecting frame
                        else
                        {
                            unsigned char* frame = NULL;
                            Frame supervisionFrameReceive = {SUPERVISION, received_IF >> 6, A0, FRAME_SIZE_S};
                            buildFrameSu(frame, &supervisionFrameReceive);
                            if (writeBytesSerialPort(frame, FRAME_SIZE_S) == -1)
                            {
                                perror("Failed to write REJ packet\n");
                                exit(ERROR);
                            }
                            return -1;
                        }
                    }

                    // Reading data
                    else {
                        packet[byte_nr] = *byte_read;
                        byte_nr++;
                    }
                    break;
                case ESC_OCT_RCV:
                    packet[byte_nr] = *byte_read^0x20;
                    byte_nr++;
                    state = WAITING_DATA;
                    break;
                default:   
            }
        }
    }
    return -1;
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






