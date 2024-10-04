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

// Baudrate settings are defined in <asm/termbits.h>, which is
// included by <termios.h>
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

enum{
    START_STATE,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    STOP_STATE
};

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
    
     // 1º - First test this. Result will be abcde.
    
    // Create string to send
    unsigned char buf[5] = {0};

    for (int i = 0; i < 5; i++)
    {
        buf[i] = 'a' + i % 26;
    }

    // In canonical mode, strings must end with '\n'
    buf[5] = '\n';
    

    int bytes = write(fd, buf, 5);
    
    printf("Bytes written = %d\n", bytes);
    
    
    

    // 2º - Second test. Build a SET message.

    unsigned char set_packet[PACKET_SIZE]; // 5 bytes message.

    set_packet[0] = FLAG;
    set_packet[1] = SENDER_ADDR; // Sender address
    set_packet[2] = SET;  // Control
    set_packet[3] = set_packet[1]^set_packet[2]; // BCC1
    set_packet[4] = FLAG;

    bytes = write(fd, set_packet, PACKET_SIZE); // Bytes written.

    if (bytes < 0){
        perror("Write packet failed\n");
        exit(-1);
    }

    printf("SET message sent\n");

    unsigned char ua_packet[PACKET_SIZE] = {0}; 
    unsigned char buf_read = 0;

    int state = 0;

    while (state != STOP_STATE){
        bytes = read(fd, &buf_read, 1); // Bytes read.

        if (bytes > 0){
            switch (state){
                case START_STATE:
                    if (buf_read == 0x7E){
                        ua_packet[0] = buf_read;
                        state = FLAG_RCV;
                    }
                    break;
                case FLAG_RCV:
                    if (buf_read == 0x03){
                        ua_packet[1] = buf_read;
                        state = A_RCV;
                    }
                    else if (buf_read != 0x7E){
                        state = START_STATE;
                    }
                    break;
                case A_RCV:
                    if (buf_read == 0x07){
                        ua_packet[2] = buf_read;
                        state = C_RCV;
                    }
                    else if (buf_read == 0x7E){
                        state = FLAG_RCV;
                    }
                    else{
                        state = START_STATE;
                    }
                    break;
                case C_RCV:
                    if (buf_read == (ua_packet[1]^ua_packet[2])){
                        ua_packet[3] = buf_read;
                        state = BCC_OK;
                    }
                    else if (buf_read == 0x7E){
                        state = FLAG_RCV;
                    }
                    else{
                        state = START_STATE;
                    }
                    break;
                case BCC_OK:
                    if (buf_read == 0x7E){
                        ua_packet[4] = buf_read;
                        state = STOP_STATE;
                    }
                    else{
                        state = START_STATE;
                    }
                    break;
                default:
                    state = START_STATE;
            }
        }
    
    }

    // Print ua packet

    printf("Receiveing UA packet \n");
    printf("flag = %02X\n", ua_packet[0]);
    printf("sender_addr = %02X\n", ua_packet[1]);
    printf("control = %02X\n", ua_packet[2]);
    printf("bcc1 = %02X\n", ua_packet[3]);
    printf("flag = %02X\n", ua_packet[4]);

    // Wait until all bytes have been written to the serial port
    sleep(1);

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);

    return 0;
}
