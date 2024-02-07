#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

// #define SERIAL_PORT "/dev/tty.usbmodem14103"
#define SERIAL_PORT "/dev/tty.SLAB_USBtoUART"

int main(){
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
        char send = '1';
        printf("\nSend: ");
        scanf("%c", &send);
        fflush(stdin);
        int bytes_written = write(fd, &send, 1);
        usleep(50000); //50000 is visible blinkning.

        char read_buffer;
        int bytes_read = read(fd, &read_buffer, 1);
        if(bytes_read > 0){
            printf("Received: %c", read_buffer);
        }
        usleep(50000);  //50000 is visible blinkning.
    }
}