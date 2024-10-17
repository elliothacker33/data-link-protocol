// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    // Create Link Layer settings
    LinkLayer openConnection;
    strcpy(openConnection.serialPort, serialPort); // serial port
    openConnection.baudRate = baudRate; // baud rate
    openConnection.nRetransmissions = nTries; // number of retries
    openConnection.timeout = timeout; // timeout 
    int fd;

    if ((*role) == 't'){
        openConnection.role = LlTx;
        fd = llopen(openConnection);
        if (fd == -1){
            perror("Error: Error opening connection");
            exit(-1);
        }

        fd = llclose(0);
        if (fd == -1){
            perror("Error: Error opening connection");
            exit(-1);
        }


    }
    else {
        openConnection.role = LlRx;
        fd = llopen(openConnection);

        if (fd == -1){
            perror("Error: Error opening connection");
            exit(-1);
        }

        fd = llclose(0);
        if (fd == -1){
            perror("Error: Error opening connection");
            exit(-1);
        }


    }




}
