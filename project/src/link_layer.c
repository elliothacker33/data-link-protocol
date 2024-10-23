#define _POSIX_SOURCE 1 // POSIX compliant source
// Link layer protocol implementation
#include "link_layer.h"
#include "serial_port.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>

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

#define C_IF(Nr) (Nr << 7)

// Control
#define SET 0x03
#define UA 0x07
#define RR(Nr) (0xAA | Nr)
#define REJ(Nr) (0x54 | Nr)
#define DISC 0x0B

// Byte stuffing
#define ESC 0x7D


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

unsigned char Ns;

void alarmHandler(int signal)
{
    // Alarm enabled means that the alarm got triggered (time set expired)
    if (signal == SIGALRM) {
        alarmRinging = FALSE;
        alarmCount++;
        printf("Alarm #%d\n", alarmCount);
    }
    else{
        printf("Unexpected signal %d\n", signal);
    }
}

void buildFrameSupervision(unsigned char* frame, const unsigned char address, const unsigned char control){
    frame[0] = FLAG;
    frame[1] = address;
    frame[2] = control;
    frame[3] = address ^ control;
    frame[4] = FLAG;
}

void buildFrameInformation(unsigned char* frame, const unsigned char* data, const unsigned char address, const unsigned char control, const size_t bytes){
    frame[0] = FLAG;
    frame[1] = address;
    frame[2] = control;
    frame[3] = address ^ control;

    unsigned char bcc2 = 0;
    for (int i = 0; i < bytes; i++){
        frame[4 + i] = data[i];
        bcc2 ^= data[i];
    }
    frame[4 + bytes] = bcc2;
    frame[5 + bytes] = FLAG;
}


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
                    if (byte_read == FLAG){
                        alarm(0);
                        state = STOP_STATE;
                    }
                    else {
                        state = WAITING_FLAG;
                    }
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
int llwrite(const unsigned char *buf, int bufSize) {

    struct sigaction sa;
    sa.sa_handler = alarmHandler;
    sa.sa_flags = 0;
    if (sigaction(SIGALRM, &sa, 0) == ERROR) {
        perror("ERROR: Setting signal handler\n");
        return ERROR;
    }

    if (buf == NULL) {
        perror("ERROR: buffer is null");
        return ERROR;
    }

    int frameBytes = bufSize + 6;
    unsigned char* frameBufferSend = (unsigned char*)malloc(frameBytes * sizeof(unsigned char));
    if (frameBufferSend == NULL) {
        perror("ERROR: Allocating memory for frameBufferSend\n");
        return ERROR;
    }
    buildFrameInformation(frameBufferSend, buf, A0, C_IF(Ns), bufSize);

    // Byte stuffing (+1 for bcc2 stuffing)
    for (size_t i = 0; i < bufSize + 1; i++) {
        if (frameBufferSend[4 + i] == FLAG || frameBufferSend[4 + i] == ESC) {
            frameBytes++;
            frameBufferSend = realloc(frameBufferSend, frameBytes);
            if (frameBufferSend == NULL) {
                perror("ERROR: Reallocating memory for frameBufferSend\n");
                return ERROR;
            }
            memmove(&frameBufferSend[4 + i + 2], &frameBufferSend[4 + i + 1], frameBytes - (4 + i + 2));

            if (frameBufferSend[4 + i] == FLAG) {
                frameBufferSend[4 + i] = ESC;
                frameBufferSend[4 + i + 1] = FLAG^0x20;
            } else if (frameBufferSend[4 + i] == ESC) {
                frameBufferSend[4 + i] = ESC;
                frameBufferSend[4 + i + 1] = ESC^0x20;
            }
            i++;
        }
    }

    // Supervision frame received
    unsigned char* frameBufferReceive = (unsigned char*)malloc(FRAME_SIZE_S * sizeof(unsigned char));
    if (frameBufferReceive == NULL) {
        perror("ERROR: Allocating memory for frameBufferReceive\n");
        free(frameBufferSend);
        return ERROR;
    }

    State state = WAITING_FLAG;
    int byte;
    unsigned char buffer_read = 0;
    alarmCount = 0;
    alarmRinging = FALSE;

    while (alarmCount < nRetransmissions) {
        
        // Sending information frame
        if (alarmRinging == FALSE) {
            alarmRinging = TRUE;
            alarm(timeout);
            
            byte = writeBytesSerialPort(frameBufferSend, frameBytes);
            if (byte == ERROR) {
                perror("Error writing bytes to serial port\n");
                free(frameBufferSend);
                free(frameBufferReceive);
                return ERROR;
            } else {
                printf("I%d sent, bytes written = %d \n",Ns,byte);
            }
        }

        byte = readByteSerialPort(&buffer_read);
        if (byte > 0) {
            switch (state) {
                case WAITING_FLAG:
                    if (buffer_read == FLAG) {
                        frameBufferReceive[0] = buffer_read;
                        state = WAITING_ADDR;
                    }
                    break;
                case WAITING_ADDR:
                    if (buffer_read == A0) {
                        frameBufferReceive[1] = buffer_read;
                        state = WAITING_CTRL;
                    } else if (buffer_read != FLAG) {
                        state = WAITING_FLAG;
                    }
                    break;
                case WAITING_CTRL:
                    if (buffer_read == REJ(0) || buffer_read == REJ(1) || buffer_read == RR(0) || buffer_read == RR(1)) {
                        frameBufferReceive[2] = buffer_read;
                        state = WAITING_BCC1;
                    } else if (buffer_read == FLAG) {
                        state = WAITING_ADDR;
                    } else {
                        state = WAITING_FLAG;
                    }
                    break;
                case WAITING_BCC1:
                    if (buffer_read == (frameBufferReceive[1] ^ frameBufferReceive[2])) {
                        frameBufferReceive[3] = buffer_read;
                        state = WAITING_FLAG2;
                    } else if (buffer_read == FLAG) {
                        state = WAITING_ADDR;
                    } else {
                        state = WAITING_FLAG;
                    }
                    break;
                case WAITING_FLAG2:
                    if (buffer_read == FLAG) {
                        frameBufferReceive[4] = buffer_read;
                        state = STOP_STATE;
                        alarm(0);
                    } else {
                        state = WAITING_FLAG;
                    }
                    break;
                default:
                    state = WAITING_FLAG;
            }
        }

        // Actions after receiving frame
        if (state == STOP_STATE) {

            if (frameBufferReceive[2] == REJ(0)) {
                printf("REJ(0) received\n");
                printf("Frames sent with errors\n");
            }
            else if (frameBufferReceive[2] == REJ(1)) {
                printf("REJ(1) received\n");
                printf("Frames sent with errors\n");
            }
            else if (frameBufferReceive[2] == RR(0) && Ns == 0){
                Ns^=1;
                printf("RR(0) received\n");
                printf("Duplicate frames\n");
                free(frameBufferSend);
                free(frameBufferReceive);
                return 0;

            }
            else if (frameBufferReceive[2] == RR(1) && Ns == 1){
                Ns^=1;
                printf("RR(1) received\n");
                printf("Duplicate frames\n");
                free(frameBufferSend);
                free(frameBufferReceive);
                return 0;
            }
            else if (frameBufferReceive[2] == RR(1) && Ns == 0) {
                Ns^=1;
                printf("RR(1) received\n");
                printf("Frames sent and received successfully\n");
                free(frameBufferSend);
                free(frameBufferReceive);
                return frameBytes;
            }
            else if (frameBufferReceive[2] == RR(0) && Ns == 1) {
                Ns^=1;
                printf("RR(0) received\n");
                printf("Frames sent and received successfully\n");
                free(frameBufferSend);
                free(frameBufferReceive);
                return frameBytes;
            }
        }
    }

    free(frameBufferSend);
    free(frameBufferReceive);
    return ERROR;
}



////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{

    // State machine
    State state = WAITING_FLAG;
    unsigned char* byte_read = 0;
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
                                buildFrameSupervision(frame, A0, RR(Ns));
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
                                buildFrameSupervision(frame, RR(Ns),A0);
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
                            if (C_IF(Ns) == received_IF) 
                            {
                                unsigned char* frame = NULL;
                                buildFrameSupervision(frame, A0,REJ(received_IF >> 7));
                                if (writeBytesSerialPort(frame, FRAME_SIZE_S) == -1)
                                {
                                    perror("Failed to write REJ packet\n");
                                    exit(ERROR);
                                }
                                return -1;
                            }
                            else
                            {
                                state = STOP_STATE;
                                unsigned char* frame = NULL;
                                buildFrameSupervision(frame, RR(Ns),A0);
                                if (writeBytesSerialPort(frame, FRAME_SIZE_S) == -1)
                                {
                                    perror("Failed to write RR packet");
                                    exit(ERROR);
                                }
                                // ignore packet received till now
                                return 0;
                            }
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
                    state = WAITING_FLAG;
            }
        }
    }
    return -1;
}

int llclose(int showStatistics) {

    // Signal handling
    struct sigaction sa;
    sa.sa_handler = alarmHandler;
    sa.sa_flags = 0;
    if (sigaction(SIGALRM, &sa, 0) == ERROR) {
        perror("ERROR: Setting signal handler\n");
        return ERROR;
    }

    State state;
    alarmCount = 0;
    alarmRinging = FALSE;

    // Allocate memory for the frame buffers
    unsigned char* frameBufferSend = malloc(FRAME_SIZE_S * sizeof(unsigned char));
    if (frameBufferSend == NULL) {
        perror("ERROR: Allocating memory for frameBufferSend\n");
        return ERROR;
    }
    unsigned char* frameBufferReceive = malloc(FRAME_SIZE_S * sizeof(unsigned char));
    if (frameBufferReceive == NULL) {
        perror("ERROR: Allocating memory for frameBufferReceive\n");
        return ERROR;
    }

    if (role == LlTx) {
        printf("LLCLOSE: Lltx\n");
        // Build supervision frame
        buildFrameSupervision(frameBufferSend, A0, DISC);

        unsigned char buffer_read = 0;
        int byte;
        state = WAITING_FLAG;

        while (state != STOP_STATE && alarmCount < nRetransmissions) {
            // Sending DISC frame
            if (!alarmRinging) {
                alarmRinging = TRUE;
                alarm(timeout);

                byte = writeBytesSerialPort(frameBufferSend, FRAME_SIZE_S);
                if (byte == ERROR) {
                    perror("Error writing bytes to serial port\n");
                    free(frameBufferSend);
                    free(frameBufferReceive);
                    exit(ERROR);
                } else {
                    printf("DISC frame sent, bytes written = %d \n", byte);
                }
            }

            // Reading DISC frame
            byte = readByteSerialPort(&buffer_read);

            if (byte > 0) {
                switch (state) {
                    case WAITING_FLAG:
                        if (buffer_read == FLAG) {
                            frameBufferReceive[0] = buffer_read;
                            state = WAITING_ADDR;
                        }
                        break;
                    case WAITING_ADDR:
                        if (buffer_read == A1) {
                            frameBufferReceive[1] = buffer_read;
                            state = WAITING_CTRL;
                        } else if (buffer_read != FLAG) {
                            state = WAITING_FLAG;
                        }
                        break;
                    case WAITING_CTRL:
                        if (buffer_read == DISC) {
                            frameBufferReceive[2] = buffer_read;
                            state = WAITING_BCC1;
                        } else if (buffer_read == FLAG) {
                            state = WAITING_ADDR;
                        } else {
                            state = WAITING_FLAG;
                        }
                        break;
                    case WAITING_BCC1:
                        if (buffer_read == (frameBufferReceive[1] ^ frameBufferReceive[2])) {
                            frameBufferReceive[3] = buffer_read;
                            state = WAITING_FLAG2;
                        } else if (buffer_read == FLAG) {
                            state = WAITING_ADDR;
                        } else {
                            state = WAITING_FLAG;
                        }
                        break;
                    case WAITING_FLAG2:
                        if (buffer_read == FLAG) {
                            frameBufferReceive[4] = buffer_read;
                            state = STOP_STATE;
                            alarm(0);
                        } else {
                            state = WAITING_FLAG;
                        }
                        break;
                }
            }
        }

        if (state != STOP_STATE) {
            perror("ERROR: Timeout during receiving DISC\n");
            free(frameBufferSend);
            free(frameBufferReceive);
            return ERROR;
        }
        else{
            printf("DISC frame received\n");
        }


        // Create and send UA frame
        buildFrameSupervision(frameBufferSend, A1, UA);

        byte = writeBytesSerialPort(frameBufferSend, FRAME_SIZE_S);
        if (byte == ERROR) {
            perror("Error writing bytes to serial port\n");
            free(frameBufferSend);
            free(frameBufferReceive);
            return ERROR;
        } else {
            printf("UA frame sent, bytes written = %d\n",byte);
        }

    } else if (role == LlRx) {
        printf("LLCLOSE: LlRx\n");
        unsigned char buffer_read = 0;
        int byte;
        state = WAITING_FLAG;

        // Receiver logic
        while (state != STOP_STATE) {
            byte = readByteSerialPort(&buffer_read);
            if (byte > 0) {
                switch (state) {
                    case WAITING_FLAG:
                        if (buffer_read == FLAG) {
                            frameBufferReceive[0] = buffer_read;
                            state = WAITING_ADDR;
                        }
                        break;
                    case WAITING_ADDR:
                        if (buffer_read == A0) {
                            frameBufferReceive[1] = buffer_read;
                            state = WAITING_CTRL;
                        } else if (buffer_read != FLAG) {
                            state = WAITING_FLAG;
                        }
                        break;
                    case WAITING_CTRL:
                        if (buffer_read == DISC) {
                            frameBufferReceive[2] = buffer_read;
                            state = WAITING_BCC1;
                        } else if (buffer_read == FLAG) {
                            state = WAITING_ADDR;
                        } else {
                            state = WAITING_FLAG;
                        }
                        break;
                    case WAITING_BCC1:
                        if (buffer_read == (frameBufferReceive[1] ^ frameBufferReceive[2])) {
                            frameBufferReceive[3] = buffer_read;
                            state = WAITING_FLAG2;
                        } else if (buffer_read == FLAG) {
                            state = WAITING_ADDR;
                        } else {
                            state = WAITING_FLAG;
                        }
                        break;
                    case WAITING_FLAG2:
                        if (buffer_read == FLAG) {
                            frameBufferReceive[4] = buffer_read;
                            state = STOP_STATE;
                            alarm(0);
                        } else {
                            state = WAITING_FLAG;
                        }
                        break;
                }
            }
        }

        // Create DISC frame
        buildFrameSupervision(frameBufferSend, A1, DISC);
        state = WAITING_FLAG;
        alarmCount = 0;

        while (state != STOP_STATE && alarmCount < nRetransmissions) {
            
            // Sending DISC frame
            if (!alarmRinging) {
                alarmRinging = TRUE;
                alarm(timeout);

                byte = writeBytesSerialPort(frameBufferSend, FRAME_SIZE_S);
                if (byte == ERROR) {
                    perror("Error writing bytes to serial port\n");
                    free(frameBufferSend);
                    free(frameBufferReceive);
                    return ERROR;
                } else {
                    printf("DISC frame sent, bytes written = %d\n",byte);
                }
            }

            // Reading UA frame
            byte = readByteSerialPort(&buffer_read);
            if (byte > 0) {
                switch (state) {
                    case WAITING_FLAG:
                        if (buffer_read == FLAG) {
                            frameBufferReceive[0] = buffer_read;
                            state = WAITING_ADDR;
                        }
                        break;
                    case WAITING_ADDR:
                        if (buffer_read == A1) {
                            frameBufferReceive[1] = buffer_read;
                            state = WAITING_CTRL;
                        } else if (buffer_read != FLAG) {
                            state = WAITING_FLAG;
                        }
                        break;
                    case WAITING_CTRL:
                        if (buffer_read == UA) {
                            frameBufferReceive[2] = buffer_read;
                            state = WAITING_BCC1;
                        } else if (buffer_read == FLAG) {
                            state = WAITING_ADDR;
                        } else {
                            state = WAITING_FLAG;
                        }
                        break;
                    case WAITING_BCC1:
                        if (buffer_read == (frameBufferReceive[1] ^ frameBufferReceive[2])) {
                            frameBufferReceive[3] = buffer_read;
                            state = WAITING_FLAG2;
                        } else if (buffer_read == FLAG) {
                            state = WAITING_ADDR;
                        } else {
                            state = WAITING_FLAG;
                        }
                        break;
                    case WAITING_FLAG2:
                        if (buffer_read == FLAG) {
                            frameBufferReceive[4] = buffer_read;
                            state = STOP_STATE;
                            alarm(0);
                        } else {
                            state = WAITING_FLAG;
                        }
                        break;
                }
            }
        }

        if (state != STOP_STATE) {
            perror("ERROR: Timeout during receiving UA\n");
            free(frameBufferSend);
            free(frameBufferReceive);
            return ERROR;
        }
        else {
            printf("UA frame received\n");
        }
    }

    // Free allocated memory
    free(frameBufferSend);
    free(frameBufferReceive);

    // Close the serial port
    int clstat = closeSerialPort();
    return clstat;
}






