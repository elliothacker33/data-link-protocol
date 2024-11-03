#define _POSIX_SOURCE 1 // POSIX compliant source
// Link layer protocol implementation
#include "link_layer.h"
#include "serial_port.h"


// MISC

LinkLayerRole role;
int timeout;
int nRetransmissions;
int alarmRinging = FALSE;
int alarmCount = 0;
int TotalAlarmCount = 0;
int alarmInterrupted = 0;
int baudRate;

// Definitions
// Booleans
#define FALSE 0
#define TRUE 1
// Sizes
#define FRAME_SIZE_S 5
// Flag
#define FLAG 0x7E 

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

// Statistics
struct timeval  tv1, tv2;

typedef enum SupervisionState
{
    S_WAITING_FLAG,
    S_WAITING_ADDR,
    S_WAITING_CTRL,
    S_WAITING_BCC1,
    S_WAITING_FLAG2,
    S_STOP_STATE
} SupervisionState;
typedef enum InformationState
{
    I_WAITING_FLAG,
    I_WAITING_ADDR,
    I_WAITING_CTRL,
    I_WAITING_BCC1,
    I_WAITING_DATA,
    I_WAITING_BCC2,
    I_ESC_OCT_RCV,
    I_STOP_STATE
} InformationState;

unsigned char Ns; // expected number for frame

void alarmHandler(int signal)
{
    // Alarm enabled means that the alarm got triggered (time set expired)
    if (signal == SIGALRM) {
        alarmRinging = FALSE;
        alarmCount++;
        printf("Alarm #%d expired\n", alarmCount);
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

void buildFrameInformation(unsigned char* frame, const unsigned char* data, const unsigned char control, const int bytes){
    frame[0] = FLAG;
    frame[1] = A0;
    frame[2] = control;
    frame[3] = A0 ^ control;

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
    if (fd == -1)
        return fd;
    printf("Serial port opened\n");

    // Define parameters
    role = connectionParameters.role;
    timeout = connectionParameters.timeout;
    nRetransmissions = connectionParameters.nRetransmissions;
    baudRate = connectionParameters.baudRate;


    SupervisionState state = S_WAITING_FLAG;
    unsigned char byte_read = 0;

    struct sigaction sa;
    sa.sa_handler = alarmHandler;
    sa.sa_flags = 0;

    if (sigaction(SIGALRM, &sa, 0) == -1){
        perror("ERROR: Setting signal handler\n");
        return -1;
    }

    if (role == LlTx)
    {
        int lastAlarmCount = 0;

        while (state != S_STOP_STATE && alarmCount < nRetransmissions)
        {
            if (alarmRinging == FALSE)
            {   
                lastAlarmCount = alarmCount;
                alarm(timeout); 
                alarmRinging = TRUE;
                state = S_WAITING_FLAG;

                // Send the SET message
                unsigned char buf[FRAME_SIZE_S] = {FLAG, A0, SET, A0 ^ SET, FLAG};
                if (writeBytesSerialPort(buf, FRAME_SIZE_S) != FRAME_SIZE_S)
                {   
                    perror("Error writing SET frame to serial port\n");
                    alarm(0);
                    if (alarmCount == lastAlarmCount){
                        alarmRinging = FALSE;
                        alarmCount++;
                        alarmInterrupted++;
                        printf("Alarm %d interrupted\n", alarmCount);
                    }
                    continue;
                }
            }

            if (readByteSerialPort(&byte_read) > 0)
            {
                switch (state)
                {
                case S_WAITING_FLAG:
                    if (byte_read == FLAG)
                    {
                        state = S_WAITING_ADDR;
                    }
                    break;
                case S_WAITING_ADDR:
                    if (byte_read == A0)
                        state = S_WAITING_CTRL;
                    else if (byte_read != FLAG)
                        state = S_WAITING_FLAG;
                    break;
                case S_WAITING_CTRL:
                    if (byte_read == UA)
                        state = S_WAITING_BCC1;
                    else if (byte_read == FLAG)
                        state = S_WAITING_ADDR;
                    else
                        state = S_WAITING_FLAG;
                    break;
                case S_WAITING_BCC1:
                    if (byte_read == (A0 ^ UA))
                        state = S_WAITING_FLAG2;
                    else if (byte_read == FLAG)
                        state = S_WAITING_ADDR;
                    else
                    {
                        state = S_WAITING_FLAG;
                    }
                    break;
                case S_WAITING_FLAG2:
                    if (byte_read == FLAG){
                        state = S_STOP_STATE;
                        alarm(0);
                        if (lastAlarmCount == alarmCount){
                            alarmCount++;
                            alarmInterrupted++;
                            printf("Alarm %d interrupted\n", alarmCount);
                        }
                        printf("Connection established\n");
                        gettimeofday(&tv1, NULL);
                        printf("Starting counting now\n");
                    }
                    else {
                        state = S_WAITING_FLAG;
                        break;
                    }
                    case S_STOP_STATE:
                        break;
                }
            }
        }

        TotalAlarmCount += alarmCount;
        if (state != S_STOP_STATE){
            perror("Failed to establish connection\n");
            return -1;
        }

    }
    else if (connectionParameters.role == LlRx)
    {
        while (state != S_STOP_STATE)
        {
            if (readByteSerialPort(&byte_read) > 0)
            {
                switch (state)
                {
                case S_WAITING_FLAG:
                    if (byte_read == FLAG)
                    {
                        state = S_WAITING_ADDR;
                    }
                    break;
                case S_WAITING_ADDR:
                    if (byte_read == A0)
                        state = S_WAITING_CTRL;
                    else if (byte_read != FLAG)
                        state = S_WAITING_FLAG;
                    break;
                case S_WAITING_CTRL:
                    if (byte_read == SET)
                        state = S_WAITING_BCC1;
                    else if (byte_read == FLAG)
                        state = S_WAITING_ADDR;
                    else
                        state = S_WAITING_FLAG;
                    break;
                case S_WAITING_BCC1:
                    if (byte_read == (A0 ^ SET))
                        state = S_WAITING_FLAG2;
                    else if (byte_read == FLAG)
                        state = S_WAITING_ADDR;
                    else
                    {
                        state = S_WAITING_FLAG;
                    }
                    break;
                case S_WAITING_FLAG2:
                    if (byte_read == FLAG)
                    {
                        unsigned char buf[FRAME_SIZE_S] = {FLAG, A0, UA, A0 ^ UA, FLAG};
                        if (writeBytesSerialPort(buf, FRAME_SIZE_S) != FRAME_SIZE_S)
                        {
                            perror("Failed to write UA frame\n");
                            state = S_WAITING_FLAG;
                            break;
                        }
                        else
                        {
                            state = S_STOP_STATE;
                        }
                    }
                    else {
                        state = S_WAITING_FLAG;
                        break;
                    }
                    case S_STOP_STATE:
                        break;
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
    if (sigaction(SIGALRM, &sa, 0) == -1) {
        perror("ERROR: Setting signal handler\n");
        return -1;
    }

    if (buf == NULL) {
        perror("ERROR: buffer is null\n");
        return -1;
    }

    int frameBytes = bufSize + 6;

    unsigned char* frameBufferSend = (unsigned char*)malloc(frameBytes * sizeof(unsigned char));
    if (frameBufferSend == NULL) {
        perror("ERROR: Allocating memory for frameBufferSend\n");
        return -1;
    }
    buildFrameInformation(frameBufferSend, buf, C_IF(Ns), bufSize);

    int extraBytes = 0;

    // Byte stuffing (+1 for bcc2 stuffing)
    for (int i = 0; i < bufSize + 1; i++) {
        if (frameBufferSend[4 + i] == FLAG || frameBufferSend[4 + i] == ESC){
            extraBytes++;
        }
    }

    
    if (extraBytes > 0) {
       
        frameBytes += extraBytes;
        frameBufferSend = realloc(frameBufferSend, frameBytes);
        if (frameBufferSend == NULL) {
            perror("ERROR: Reallocating memory for frameBufferSend\n");
            free(frameBufferSend);
            return -1;
        }
        
       
        for (int i = 0; i < bufSize + extraBytes + 1; i++) {
            if (frameBufferSend[4 + i] == FLAG || frameBufferSend[4 + i] == ESC) {
            
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
       
    }
    
    unsigned char* frameBufferReceive = (unsigned char*)malloc(FRAME_SIZE_S * sizeof(unsigned char));
    if (frameBufferReceive == NULL) {
        perror("ERROR: Allocating memory for frameBufferReceive\n");
        free(frameBufferSend);
        return -1;
    }

    SupervisionState state = S_WAITING_FLAG;
    int byte;
    unsigned char buffer_read = 0;
    alarmCount = 0;
    int lastAlarmCount = 0;
    int rejected_frame = FALSE;
    alarmRinging = FALSE;

    while (alarmCount < nRetransmissions) {

        if (alarmRinging == FALSE || rejected_frame == TRUE) {
            
            lastAlarmCount = alarmCount;
            state = S_WAITING_FLAG;
            alarmRinging = TRUE;
            rejected_frame = FALSE;
            alarm(timeout);
            
            byte = writeBytesSerialPort(frameBufferSend, frameBytes);
            if (byte != frameBytes) {
                perror("ERROR: writing bytes to serial port\n");
                alarm(0);
                if (alarmCount == lastAlarmCount){
                    alarmRinging = FALSE;
                    alarmCount++;
                    alarmInterrupted++;
                    printf("Alarm %d interrupted\n", alarmCount);
                }
                continue;
            } else {
                printf("I(%d) sent, bytes written = %d \n",Ns,byte);
            }
        }

        
        byte = readByteSerialPort(&buffer_read);

        if (byte > 0) {
            switch (state) {
                case S_WAITING_FLAG:
                    if (buffer_read == FLAG) {
                        frameBufferReceive[0] = buffer_read;
                        state = S_WAITING_ADDR;
                    }
                    break;
                case S_WAITING_ADDR:
                    if (buffer_read == A0) {
                        frameBufferReceive[1] = buffer_read;
                        state = S_WAITING_CTRL;
                    } else if (buffer_read != FLAG) {
                        state = S_WAITING_FLAG;
                    }
                    break;
                case S_WAITING_CTRL:
                    if (buffer_read == REJ(0) || buffer_read == REJ(1) || buffer_read == RR(0) || buffer_read == RR(1)) {
                        frameBufferReceive[2] = buffer_read;
                        state = S_WAITING_BCC1;
                    } else if (buffer_read == FLAG) {
                        state = S_WAITING_ADDR;
                    } else {
                        state = S_WAITING_FLAG;
                    }
                    break;
                case S_WAITING_BCC1:
                    if (buffer_read == (frameBufferReceive[1] ^ frameBufferReceive[2])) {
                        frameBufferReceive[3] = buffer_read;
                        state = S_WAITING_FLAG2;
                    } else if (buffer_read == FLAG) {
                        state = S_WAITING_ADDR;
                    } else {
                        state = S_WAITING_FLAG;
                    }
                    break;
                case S_WAITING_FLAG2:
                    if (buffer_read == FLAG) {
                        frameBufferReceive[4] = buffer_read;
                        state = S_STOP_STATE;
                        alarm(0);
                        if (alarmCount == lastAlarmCount){
                            alarmCount++;
                            alarmInterrupted++;
                            printf("Alarm %d interrupted\n", alarmCount);
                        }
                    } else {
                        state = S_WAITING_FLAG;
                    }
                    break;
                default:
                    state = S_WAITING_FLAG;
            }
        }

        if (state == S_STOP_STATE && byte > 0) {
            if (frameBufferReceive[2] == REJ(0) || frameBufferReceive[2] == REJ(1)) {
                printf("REJ(%d) received\n", frameBufferReceive[2] == REJ(0) ? 0 : 1);
                printf("Frames sent with errors\n");
                rejected_frame = TRUE;
            } 
            else if ((frameBufferReceive[2] == RR(0) && Ns == 1) || (frameBufferReceive[2] == RR(1) && Ns == 0)) {
                Ns ^= 1;
                printf("RR(%d) received\n", frameBufferReceive[2] == RR(0) ? 0 : 1);
                printf("Frames sent and received successfully\n");
                free(frameBufferSend);
                free(frameBufferReceive);
                TotalAlarmCount += alarmCount;
                return frameBytes;
            }
        }
    }

    free(frameBufferSend);
    free(frameBufferReceive);
    return -1;
}



////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{

    InformationState state = I_WAITING_FLAG;
    unsigned char byte_read = 0;
    unsigned char received_IF = 0;
    int byte_nr = 0;
    while (TRUE) {
        if (readByteSerialPort(&byte_read) > 0) {
            switch (state) {
                case I_WAITING_FLAG:
                    if (byte_read == FLAG) {
                        state = I_WAITING_ADDR;
                    }
                    break;
                case I_WAITING_ADDR:
                    if (byte_read == A0) {
                        state = I_WAITING_CTRL;
                    } else if (byte_read == FLAG) {
                        state = I_WAITING_ADDR;
                    } else {
                        state = I_WAITING_FLAG;
                    }
                    break;
                case I_WAITING_CTRL:

                    if (byte_read == C_IF(0) || byte_read == C_IF(1)) {
                        received_IF = byte_read;
                        state = I_WAITING_BCC1;
                    } else if (byte_read == FLAG) {
                        state = I_WAITING_ADDR;
                    } else {
                        state = I_WAITING_FLAG;
                    }
                    break;
                case I_WAITING_BCC1:
                    if (byte_read == (A0 ^ C_IF(0)) || byte_read == (A0 ^ C_IF(1))) {
                        state = I_WAITING_DATA;
                    } else if (byte_read == FLAG) {
                        state = I_WAITING_ADDR;
                    } else {
                        state = I_WAITING_FLAG;
                    }
                    break;
                case I_WAITING_DATA:
                    if (byte_read == ESC) {
                        state = I_ESC_OCT_RCV;
                    }
                    else if (byte_read == FLAG) {
                        
                
                        unsigned char bcc2_rcv = packet[byte_nr - 1];
                        byte_nr--;
                        unsigned char bcc2_actual = 0;


                        for (size_t i = 0; i < byte_nr; i++)
                        {
                            bcc2_actual = bcc2_actual ^ packet[i];
                        }

                        if (bcc2_actual == bcc2_rcv)
                        {
                            // Frame received, ready to receive next frame
                            if (C_IF(Ns) == received_IF) {
                                Ns ^= 1;

                                unsigned char* frame = (unsigned char*) malloc (FRAME_SIZE_S*sizeof(unsigned char));
                                buildFrameSupervision(frame, A0, RR(Ns));
                                if (writeBytesSerialPort(frame, FRAME_SIZE_S) != FRAME_SIZE_S) 
                                {
                                    perror("Failed to write RR frame");
                                    return -1;
                                }
                                printf("Writing RR(%d) frame\n", Ns);
                                return byte_nr;

                            }

                            // Duplicate frame, discarding
                            else
                            {

                                unsigned char* frame = (unsigned char*) malloc (FRAME_SIZE_S*sizeof(unsigned char));
                                buildFrameSupervision(frame, RR(Ns),A0);
                                if (writeBytesSerialPort(frame, FRAME_SIZE_S) != FRAME_SIZE_S) 
                                {
                                    perror("Failed to write RR packet");
                                    return -1;
                                }

                                printf("Duplicate frame, writing RR(%d) frame\n", Ns);
                                // ignore packet received till now
                                return 0;
                            }
                        }
                        // BCC2 error, rejecting frame
                        else
                        {
                            if (C_IF(Ns) == received_IF) 
                            {
                                unsigned char* frame = (unsigned char*) malloc (FRAME_SIZE_S*sizeof(unsigned char));
                                buildFrameSupervision(frame, A0,REJ(received_IF >> 7));
                                if (writeBytesSerialPort(frame, FRAME_SIZE_S) != FRAME_SIZE_S)
                                {
                                    perror("Failed to write REJ frame\n");
                                    return -1;
                                }
    
                                printf("Writing REJ(%d) frame\n", received_IF >> 7);
                                return -1;
                            
                            }
                            else
                            {
                                unsigned char* frame = (unsigned char*) malloc (FRAME_SIZE_S*sizeof(unsigned char));
                                buildFrameSupervision(frame, RR(Ns),A0);
                                if (writeBytesSerialPort(frame, FRAME_SIZE_S) != FRAME_SIZE_S)
                                {
                                    perror("Failed to write RR frame\n");
                                    return -1;
                                }
                                // ignore packet received till now
                                return 0;
                            }
                        }
                    }

                    // Reading data
                    else {
                        packet[byte_nr] = byte_read;
                        byte_nr++;
                    }
                    break;
                case I_ESC_OCT_RCV:
                    packet[byte_nr] = byte_read^0x20;
                    byte_nr++;
                    state = I_WAITING_DATA;
                    break;
                default:
                    state = I_WAITING_FLAG;
            }
        }
    }
}

int llclose(int showStatistics) {

    struct sigaction sa;
    sa.sa_handler = alarmHandler;
    sa.sa_flags = 0;
    if (sigaction(SIGALRM, &sa, 0) == -1) {
        perror("ERROR: Setting signal handler\n");
        return -1;
    }

    SupervisionState state;
    alarmCount = 0;
    alarmRinging = FALSE;
    int lastAlarmCount = 0;

    unsigned char* frameBufferSend = malloc(FRAME_SIZE_S * sizeof(unsigned char));
    if (frameBufferSend == NULL) {
        perror("ERROR: Allocating memory for frameBufferSend\n");
        return -1;
    }
    unsigned char* frameBufferReceive = malloc(FRAME_SIZE_S * sizeof(unsigned char));
    if (frameBufferReceive == NULL) {
        perror("ERROR: Allocating memory for frameBufferReceive\n");
        return -1;
    }   

    if (role == LlTx) {
        printf("LLCLOSE: Lltx\n");
        buildFrameSupervision(frameBufferSend, A0, DISC);

        unsigned char buffer_read = 0;
        int byte;
        state = S_WAITING_FLAG;

        while (state != S_STOP_STATE && alarmCount < nRetransmissions) {
            // Sending DISC frame
            if (!alarmRinging) {
                lastAlarmCount = alarmCount;
                alarmRinging = TRUE;
                alarm(timeout);
                state = S_WAITING_FLAG;

                byte = writeBytesSerialPort(frameBufferSend, FRAME_SIZE_S);
                if (byte != FRAME_SIZE_S) {
                    perror("Error writing DISC frame to serial port\n");
                    alarm(0);
                    if (alarmCount == lastAlarmCount){
                        alarmRinging = FALSE;
                        alarmCount++;
                        alarmInterrupted++;
                        printf("Alarm %d interrupted\n", alarmCount);
                    }
                    continue;
                } else {
                    printf("DISC frame sent, bytes written = %d \n", byte);
                }
            }

            // Reading DISC frame
            byte = readByteSerialPort(&buffer_read);

            if (byte > 0) {
                switch (state) {
                    case S_WAITING_FLAG:
                        if (buffer_read == FLAG) {
                            frameBufferReceive[0] = buffer_read;
                            state = S_WAITING_ADDR;
                        }
                        break;
                    case S_WAITING_ADDR:
                        if (buffer_read == A1) {
                            frameBufferReceive[1] = buffer_read;
                            state = S_WAITING_CTRL;
                        } else if (buffer_read != FLAG) {
                            state = S_WAITING_FLAG;
                        }
                        break;
                    case S_WAITING_CTRL:
                        if (buffer_read == DISC) {
                            frameBufferReceive[2] = buffer_read;
                            state = S_WAITING_BCC1;
                        } else if (buffer_read == FLAG) {
                            state = S_WAITING_ADDR;
                        } else {
                            state = S_WAITING_FLAG;
                        }
                        break;
                    case S_WAITING_BCC1:
                        if (buffer_read == (frameBufferReceive[1] ^ frameBufferReceive[2])) {
                            frameBufferReceive[3] = buffer_read;
                            state = S_WAITING_FLAG2;
                        } else if (buffer_read == FLAG) {
                            state = S_WAITING_ADDR;
                        } else {
                            state = S_WAITING_FLAG;
                        }
                        break;
                    case S_WAITING_FLAG2:
                        if (buffer_read == FLAG) {
                            frameBufferReceive[4] = buffer_read;
                            state = S_STOP_STATE;
                            alarm(0);
                            if (alarmCount == lastAlarmCount){
                                alarmCount++;
                                alarmInterrupted++;
                                printf("Alarm %d interrupted\n", alarmCount);
                            }
                        } else {
                            state = S_WAITING_FLAG;
                            break;
                        }
                    case S_STOP_STATE:
                        break;
                }
            }
        }

        TotalAlarmCount += alarmCount;

        if (state != S_STOP_STATE) {
            perror("ERROR: Timeout during receiving DISC\n");
            free(frameBufferSend);
            free(frameBufferReceive);
            return -1;
        }
        else{
            printf("DISC frame received\n");
        }


        // Create and send UA frame
        buildFrameSupervision(frameBufferSend, A1, UA);

        byte = writeBytesSerialPort(frameBufferSend, FRAME_SIZE_S);

        if (byte != FRAME_SIZE_S) {
            perror("Error writing bytes to serial port\n");
            free(frameBufferSend);
            free(frameBufferReceive);
            return -1;
        } else {
            printf("UA frame sent, bytes written = %d\n",byte);
        }

        gettimeofday(&tv2, NULL);
        printf("Stopping counting now\n");

    } else if (role == LlRx) {
        printf("LLCLOSE: LlRx\n");
        unsigned char buffer_read = 0;
        int byte;
        state = S_WAITING_FLAG;

        while (state != S_STOP_STATE) {
            byte = readByteSerialPort(&buffer_read);
            if (byte > 0) {
                switch (state) {
                    case S_WAITING_FLAG:
                        if (buffer_read == FLAG) {
                            frameBufferReceive[0] = buffer_read;
                            state = S_WAITING_ADDR;
                        }
                        break;
                    case S_WAITING_ADDR:
                        if (buffer_read == A0) {
                            frameBufferReceive[1] = buffer_read;
                            state = S_WAITING_CTRL;
                        } else if (buffer_read != FLAG) {
                            state = S_WAITING_FLAG;
                        }
                        break;
                    case S_WAITING_CTRL:
                        if (buffer_read == DISC) {
                            frameBufferReceive[2] = buffer_read;
                            state = S_WAITING_BCC1;
                        } else if (buffer_read == FLAG) {
                            state = S_WAITING_ADDR;
                        } else {
                            state = S_WAITING_FLAG;
                        }
                        break;
                    case S_WAITING_BCC1:
                        if (buffer_read == (frameBufferReceive[1] ^ frameBufferReceive[2])) {
                            frameBufferReceive[3] = buffer_read;
                            state = S_WAITING_FLAG2;
                        } else if (buffer_read == FLAG) {
                            state = S_WAITING_ADDR;
                        } else {
                            state = S_WAITING_FLAG;
                        }
                        break;
                    case S_WAITING_FLAG2:
                        if (buffer_read == FLAG) {
                            frameBufferReceive[4] = buffer_read;
                            state = S_STOP_STATE;
                        } else {
                            state = S_WAITING_FLAG;
                            break;
                        }

                    case S_STOP_STATE:
                        break;
                }
            }
        }

        // Create DISC frame
        buildFrameSupervision(frameBufferSend, A1, DISC);
        state = S_WAITING_FLAG;
        alarmCount = 0;

        while (state != S_STOP_STATE && alarmCount < nRetransmissions) {
            
            // Sending DISC frame
            if (!alarmRinging) {
                lastAlarmCount = alarmCount;
                alarmRinging = TRUE;
                alarm(timeout);
                state = S_WAITING_FLAG;

                byte = writeBytesSerialPort(frameBufferSend, FRAME_SIZE_S);
                if (byte != FRAME_SIZE_S) {
                    perror("Error writing DISC frame to serial port\n");
                    alarm(0);
                    if (alarmCount == lastAlarmCount){
                        alarmRinging = FALSE;
                        alarmCount++;
                        alarmInterrupted++;
                        printf("Alarm %d interrupted\n", alarmCount);
                    }
                    continue;
                } else {
                    printf("DISC frame sent, bytes written = %d\n",byte);
                }
            }

            // Reading UA frame
            byte = readByteSerialPort(&buffer_read);

            if (byte > 0) {
                switch (state) {
                    case S_WAITING_FLAG:
                        if (buffer_read == FLAG) {
                            frameBufferReceive[0] = buffer_read;
                            state = S_WAITING_ADDR;
                        }
                        break;
                    case S_WAITING_ADDR:
                        if (buffer_read == A1) {
                            frameBufferReceive[1] = buffer_read;
                            state = S_WAITING_CTRL;
                        } else if (buffer_read != FLAG) {
                            state = S_WAITING_FLAG;
                        }
                        break;
                    case S_WAITING_CTRL:
                        if (buffer_read == UA) {
                            frameBufferReceive[2] = buffer_read;
                            state = S_WAITING_BCC1;
                        } else if (buffer_read == FLAG) {
                            state = S_WAITING_ADDR;
                        } else {
                            state = S_WAITING_FLAG;
                        }
                        break;
                    case S_WAITING_BCC1:
                        if (buffer_read == (frameBufferReceive[1] ^ frameBufferReceive[2])) {
                            frameBufferReceive[3] = buffer_read;
                            state = S_WAITING_FLAG2;
                        } else if (buffer_read == FLAG) {
                            state = S_WAITING_ADDR;
                        } else {
                            state = S_WAITING_FLAG;
                        }
                        break;
                    case S_WAITING_FLAG2:
                        if (buffer_read == FLAG) {
                            frameBufferReceive[4] = buffer_read;
                            state = S_STOP_STATE;
                            alarm(0);
                            if (alarmCount == lastAlarmCount){
                                alarmCount++;
                                alarmInterrupted++;
                                printf("Alarm %d interrupted\n", alarmCount);
                            }
                        } else {
                            state = S_WAITING_FLAG;
                            break;
                        }

                    case S_STOP_STATE:
                        break;
                }
            }
        }
        TotalAlarmCount += alarmCount;
        if (state != S_STOP_STATE) {
            perror("ERROR: Timeout during receiving UA frame\n");
            free(frameBufferSend);
            free(frameBufferReceive);
            return -1;
        }
        else {
            printf("UA frame received\n");
        }
    }

    free(frameBufferSend);
    free(frameBufferReceive);

    int clstat = closeSerialPort();

    if (clstat < 0) {
        perror("Error closing serial port\n");
        return clstat;
    }
    
    if (showStatistics == TRUE){
        if (role == LlTx){
            printf("Stats - Tx:\n");
        }
        else{
            printf("Stats - Rx:\n");
        }

        printf("Total alarms triggered: %d\n", TotalAlarmCount);
        printf("Alarms interrupted: %d\n", alarmInterrupted);
        printf("Alarms completed without interruption: %d\n", TotalAlarmCount - alarmInterrupted);

        printf ("Total time = %f seconds\n",
        (double) (tv2.tv_usec - tv1.tv_usec) / 1000000 +
        (double) (tv2.tv_sec - tv1.tv_sec));
    }
    
    return clstat;
}





