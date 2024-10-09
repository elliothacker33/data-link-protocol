// Write to serial port in non-canonical mode
//
// Modified by: Eduardo Nuno Almeida [enalmeida@fe.up.pt]

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>

// Baudrate settings are defined in <asm/termbits.h>, which is
// included by <termios.h>
#define BAUDRATE B38400
#define _POSIX_SOURCE 1 // POSIX compliant source

#define FALSE 0
#define TRUE 1

#define BUF_SIZE 1

#define PACKET_SIZE 5

#define FLAG 0x7E
#define ADDRESS_SENDER 0x03
#define ADDRESS_RECEIVER 0x01
#define CONTROL_SET 0x03
#define CONTROL_UA 0x07

int alarmEnabled = FALSE;
int alarmCount = 0;

volatile int STOP = FALSE;

typedef enum State
{
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
} State;

// Alarm function handler
void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;

    printf("Alarm #%d\n", alarmCount);
}

int main(int argc, char *argv[])
{
    // Program usage: Uses either COM1 or COM2
    const char *serialPortName = argv[1];

    if (argc < 2)
    {
        printf("Incorrect program usage\n"
               "Usage: %s <SerialPort>\n"
               "Example: %s /dev/ttyS1\n",
               argv[0],
               argv[0]);
        exit(1);
    }

    // Open serial port device for reading and writing, and not as controlling tty
    // because we don't want to get killed if linenoise sends CTRL-C.
    int fd = open(serialPortName, O_RDWR | O_NOCTTY);

    if (fd < 0)
    {
        perror(serialPortName);
        exit(-1);
    }

    struct termios oldtio;
    struct termios newtio;

    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1)
    {
        perror("tcgetattr");
        exit(-1);
    }

    // Clear struct for new port settings
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // Set input mode (non-canonical, no echo,...)
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0; // Inter-character timer unused
    newtio.c_cc[VMIN] = 0;  // Blocking read until 5 chars received

    // VTIME e VMIN should be changed in order to protect with a
    // timeout the reception of the following character(s)

    // Now clean the line and activate the settings for the port
    // tcflush() discards data written to the object referred to
    // by fd but not transmitted, or data received but not read,
    // depending on the value of queue_selector:
    //   TCIFLUSH - flushes data received but not read.
    tcflush(fd, TCIOFLUSH);

    // Set new port settings
    if (tcsetattr(fd, TCSANOW, &newtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set\n");

    // Create string to send
    unsigned char initial_set_packet[PACKET_SIZE] = {
        FLAG,
        ADDRESS_SENDER,
        CONTROL_SET,
        (ADDRESS_SENDER ^ CONTROL_SET),
        FLAG};

    // Set alarm function handler
    (void)signal(SIGALRM, alarmHandler);

    while (alarmCount < 3)
    {
        if (alarmEnabled == FALSE)
        {
            alarm(3); // Set alarm to be triggered in 3s
            alarmEnabled = TRUE;
        }
    }
    int bytes = write(fd, initial_set_packet, PACKET_SIZE);
    printf("Packet SET (%d bytes) written\n", bytes);

    // Wait until all bytes have been written to the serial port
    sleep(1);

    // Loop for input
    unsigned char buf[BUF_SIZE] = {0}; // +1: Save space for the final '\0' char
    unsigned char packet[PACKET_SIZE] = {0};
    State state = START;

    while (STOP == FALSE)
    {
        // Returns after 5 chars have been input
        int bytes = read(fd, buf, BUF_SIZE);
        switch (state)
        {
        case START:
            if (buf[0] == FLAG)
            {
                packet[0] = buf[0];
                state = FLAG_RCV;
            }
            break;
        case FLAG_RCV:
            if (buf[0] == ADDRESS_SENDER)
            {
                state = A_RCV;
                packet[1] = buf[0];
                break;
            }
            else if (buf[0] != FLAG)
                state = START;
            break;
        case A_RCV:
            if (buf[0] == CONTROL_UA)
            {
                state = C_RCV;
                packet[2] = buf[0];
                break;
            }
            else if (buf[0] != FLAG)
            {
                state = START;
            }
            break;
        case C_RCV:
            if (buf[0] == (ADDRESS_SENDER ^ CONTROL_UA))
            {
                state = BCC_OK;
                packet[3] = buf[0];
                break;
            }
            else if (buf[0] != FLAG)
            {
                state = START;
            }
            break;
        case BCC_OK:
            if (buf[0] == FLAG)
            {
                STOP = TRUE;
                packet[4] = buf[0];
                break;
            }
            else
            {
                state = START;
            }
            break;
        }
    }

    // print packet
    printf("Packet received: \n");
    for (int i = 0; i < PACKET_SIZE; i++)
    {
        printf("Byte: 0x%02X\n", packet[i]);
    }

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);

    return 0;
}
