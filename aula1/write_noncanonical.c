#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

#define BAUDRATE B38400
#define _POSIX_SOURCE 1 // POSIX compliant source

#define FALSE 0
#define TRUE 1

#define SET 0x03
#define UA  0x07
#define SENDER_ADDR 0x03
#define RECEIVER_ADDR 0x01
#define FLAG 0x7E

#define BUF_SIZE 256
#define PACKET_SIZE 5

volatile int STOP = FALSE;

enum State {
    START_STATE,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    STOP_STATE
};



int alarmEnabled = FALSE;
int alarmCount = 0;

// Alarm function handler
void alarmHandler(int signal) {
    alarmEnabled = FALSE;
    alarmCount++;
    printf("Alarm #%d\n", alarmCount);
}

int main(int argc, char *argv[]) {
    // Program usage: Uses either COM1 or COM2
    const char *serialPortName = argv[1];

    if (argc < 2) {
        printf("Incorrect program usage\n"
               "Usage: %s <SerialPort>\n"
               "Example: %s /dev/ttyS1\n",
               argv[0],
               argv[0]);
        exit(1);
    }

    // Open serial port device for reading and writing, and not as controlling tty
    int fd = open(serialPortName, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        perror(serialPortName);
        exit(-1);
    }

    struct termios oldtio, newtio;
    if (tcgetattr(fd, &oldtio) == -1) {
        perror("tcgetattr");
        exit(-1);
    }

    memset(&newtio, 0, sizeof(newtio));
    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;
    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd, TCSANOW, &newtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set\n");

       
     // 1º - First test this. Result will be abcde.
    
    // Create string to send
    unsigned char buf[5] = {0};
    for (int i = 0; i < 5; i++){
        buf[i] = 'a' + i % 26;
    }

    // In canonical mode, strings must end with '\n'
    buf[5] = '\n';
    
    int bytes = write(fd, buf, 5);
    printf("Bytes written = %d\n", bytes);

    // 2º - Second test. Build a SET message.

    unsigned char set_packet[PACKET_SIZE] = {FLAG, SENDER_ADDR, SET, SENDER_ADDR ^ SET, FLAG};

    (void)signal(SIGALRM, alarmHandler); // Set alarm handler

    unsigned char ua_packet[PACKET_SIZE] = {0}; 
    unsigned char buf_read = 0;
    int state = 0;

    // Start communication process
    while (state != STOP_STATE && alarmCount < 3) {
        if (!alarmEnabled) {
            alarm(3);  // Set alarm for 3 seconds
            alarmEnabled = TRUE;

            // Send the SET message
            bytes = write(fd, set_packet, PACKET_SIZE);
            if (bytes < 0) {
                perror("Failed to write SET packet");
                exit(-1);
            }
        }
        // Receive UA message
        bytes = read(fd, &buf_read, 1);

        if (bytes > 0) {

            switch (state) {
                case START_STATE:
                    if (buf_read == FLAG) {
                        ua_packet[0] = buf_read;
                        state = FLAG_RCV;
                    }
                    break;
                case FLAG_RCV:
                    if (buf_read == SENDER_ADDR) {
                        ua_packet[1] = buf_read;
                        state = A_RCV;
                    } else if (buf_read != FLAG) {
                        state = START_STATE;
                    }
                    break;
                case A_RCV:
                    if (buf_read == UA) {
                        ua_packet[2] = buf_read;
                        state = C_RCV;
                    } else if (buf_read == FLAG) {
                        state = FLAG_RCV;
                    } else {
                        state = START_STATE;
                    }
                    break;
                case C_RCV:
                    if (buf_read == (ua_packet[1] ^ ua_packet[2])) {
                        ua_packet[3] = buf_read;
                        state = BCC_OK;
                    } else if (buf_read == FLAG) {
                        state = FLAG_RCV;
                    } else {
                        state = START_STATE;
                    }
                    break;
                case BCC_OK:
                    if (buf_read == FLAG) {
                        ua_packet[4] = buf_read;
                        state = STOP_STATE;
                        alarm(0);
                        alarmEnabled = FALSE;
                    } else {
                        state = START_STATE;
                    }
                    break;
                default:
                    state = START_STATE;
            }
        }
    }

    if (alarmCount >= 3){
        printf("Communication failed after three alarms\n");
    }
    else{
        // Print received UA packet
        printf("Received UA packet:\n");
        printf("flag = %02X\n", ua_packet[0]);
        printf("sender_addr = %02X\n", ua_packet[1]);
        printf("control = %02X\n", ua_packet[2]);
        printf("bcc1 = %02X\n", ua_packet[3]);
        printf("flag = %02X\n", ua_packet[4]);
    }

    // Wait until all bytes have been written to the serial port
    sleep(1);

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);
    return 0;
}

