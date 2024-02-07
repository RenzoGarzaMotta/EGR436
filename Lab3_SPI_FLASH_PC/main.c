#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

// #define SERIAL_PORT "/dev/tty.usbmodem14103"
#define SERIAL_PORT "/dev/tty.SLAB_USBtoUART"

void getCmdInput(void);
void printMenu(void);

char command[20];
char cmdArg[50];
uint8_t itemsRead;

int main(){
    char command[10];
    char cmdArg[50];
    uint8_t itemsRead;

    int fd;
    fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NDELAY);

    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;

    options.c_cflag |= (CLOCAL | CREAD);
    tcsetattr(fd, TCSANOW, &options);

    while(1){
        // getCmdInput();

        char send = '1';
        // printf("\nSend: ");
        scanf("%c", &send);
        fflush(stdin);
        int bytes_written = write(fd, &send, 1);
        usleep(50000); //50000 is visible blinkning.

        char read_buffer;
        int bytes_read = read(fd, &read_buffer, 1);
        if(bytes_read > 0){
            printf("%c", read_buffer);
        }
        usleep(50000);  //50000 is visible blinkning.
    }
}

void getCmdInput(void){
    char inputCmd[100];

    // printf("Enter a line of input (multiple words separated by a space are optional): ");
    printMenu();
    fgets(inputCmd, sizeof(inputCmd), stdin);

    itemsRead = sscanf(inputCmd, "%s %s", command, cmdArg);

    // if (items_read < 2) {
    //     printf("You entered 1: %s\n", command);
    // } else {
    //     printf("You entered 2: %s %s\n", command, cmdArg);
    // }
}

void printMenu(void){
    printf("Available Commands:\n");
    printf("STORE <filename>.<extension>\n");
    printf("DIR\n");
    printf("MEM\n");
    printf("DELETE <number>\n");
    printf("READ <number>\n");
    printf("CLEAR\n");
    printf("\n");
    // printf("HELP <command>\n");
}

void manageCommands(void){
    if(itemsRead){  //Check if cmd input was STORE, DELETE, or READ
        if(!strcmp(command, "STORE")){//STORE selected
            //Open file from cmdArg
            //Read file
            //Send STORE cmd '1'
            //Send file through serial port
        }else if(!strcmp(command, "DELETE")){//DELETE selected
            //Send DELETE cmd '2'
            //Send file number from cmdArg through serial port
        }else if(!strcmp(command, "READ")){//READ selected
            //Send READ cmd '3'
            //Send file number from cmdArg through serial port
            //Wait for file data to be sent via serial port
            //Print file to the terminal.
        }
    }else{
        if(!strcmp(command, "DIR")){//DIR selected
            //Send DIR cmd '4'
            //Wait for data to come into the buffer
            //Display Directory File to the terminal
        }else if(!strcmp(command, "MEM")){//MEM selected
            //Send MEM cmd '5'
            //Wait for data to come into the buffer
            //Display memory available to the terminal
        }else if(!strcmp(command, "CLEAR")){//CLEAR selected
            //Send CLEAR cmd '6'
        }
    }
}

void readCommands(void){

}

void storeFile(void){

}

void printDirectory(void){

}

void dispMemory(void){

}

void deleteFile(void){

}

void readFile(void){

}

void clearChip(void){

}