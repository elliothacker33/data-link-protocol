// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include <signal.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

#define FLAG 0x7E
#define TR_ADDR 0x03
#define RC_ADDR 0x01
#define C_SET 0x03
#define C_UA 0x07

#define C_REJ 0x01

#define C_DISC 0x0B

#define BUF_SIZE 256
#define PACKET_SIZE 5

alarmRinging = FALSE;
alarmCount = 0;

typedef enum State
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

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    int fd = openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate);
    if (fd < 0)
        return -1;

    State state = START_STATE;
    unsigned char byte_read = 0;

    if (connectionParameters.role == LlTx)
    {
        (void)signal(SIGALRM, alarmHandler); // Set alarm handler

        while (state != STOP_STATE && alarmCount < connectionParameters.nRetransmissions)
        {
            if (alarmRinging == TRUE)
            {
                alarm(connectionParameters.timeout); // Set alarm for 3 seconds
                alarmRinging = FALSE;

                // Send the SET message
                unsigned char buf[PACKET_SIZE] = {FLAG, TR_ADDR, C_SET, TR_ADDR ^ C_SET, FLAG};
                if (writeBytesSerialPort(buf, PACKET_SIZE) == -1)
                {
                    perror("Failed to write SET packet");
                    exit(-1);
                }
            }

            if (readByteSerialPort(byte_read) > 0)
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
                    if (byte_read == TR_ADDR)
                        state = ADDR_RCV;
                    else if (byte_read != FLAG)
                        state = START_STATE;
                    break;
                case ADDR_RCV:
                    if (byte_read == C_UA)
                        state = CTRL_RCV;
                    else if (byte_read == FLAG)
                        state = FLAG_RCV;
                    else
                        state = START_STATE;
                    break;
                case CTRL_RCV:
                    if (byte_read == (TR_ADDR ^ C_UA))
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
    }
    else if (connectionParameters.role == LlRx)
    {
        while (state = STOP_STATE)
        {
            if (readByteSerialPort(byte_read) > 0)
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
                    if (byte_read == TR_ADDR)
                        state = ADDR_RCV;
                    else if (byte_read != FLAG)
                        state = START_STATE;
                    break;
                case ADDR_RCV:
                    if (byte_read == C_SET)
                        state = CTRL_RCV;
                    else if (byte_read == FLAG)
                        state = FLAG_RCV;
                    else
                        state = START_STATE;
                    break;
                case CTRL_RCV:
                    if (byte_read == (TR_ADDR ^ C_SET))
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
                        unsigned char buf[PACKET_SIZE] = {FLAG, TR_ADDR, C_UA, TR_ADDR ^ C_UA, FLAG};
                        if (writeBytesSerialPort(buf, PACKET_SIZE) == -1)
                        {
                            perror("Failed to write UA packet");
                            exit(-1);
                        }
                        state = STOP_STATE;
                    }
                    else
                        state = START_STATE;
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

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{

    int clstat = closeSerialPort();
    return clstat;
}
