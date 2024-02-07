#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#define SERIAL_PORT "/dev/tty.usbmodem14103"
// #define SERIAL_PORT "/dev/tty.SLAB_USBtoUART"

int set_uart_parameters(int fd, speed_t baud_rate, int data_bits, int stop_bits, int parity) {
    struct termios options;
    tcgetattr(fd, &options);

    // // Turn off any options that might interfere with our ability to send and
    // // receive raw binary bytes.
    // options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
    // options.c_oflag &= ~(ONLCR | OCRNL);
    // options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    
    // // Set up timeouts: Calls to read() will return as soon as there is
    // // at least one byte available or when 100 ms has passed.
    // options.c_cc[VTIME] = 1;
    // options.c_cc[VMIN] = 0;

    cfsetispeed(&options, baud_rate);
    cfsetospeed(&options, baud_rate);
    options.c_cflag &= ~CSIZE;
    switch (data_bits) {
        case 5:
            options.c_cflag |= CS5;
            break;
        case 6:
            options.c_cflag |= CS6;
            break;
        case 7:
            options.c_cflag |= CS7;
            break;
        case 8:
            options.c_cflag |= CS8;
            break;
        default:
            return -1;
    }
    switch (parity) {
        case 0:
            options.c_cflag &= ~PARENB;
            break;
        case 1:
            options.c_cflag |= PARENB;
            options.c_cflag &= ~PARODD;
            break;
        case 2:
            options.c_cflag |= PARENB;
            options.c_cflag |= PARODD;
            break;
        default:
            return -1;
    }
    if (stop_bits == 1) {
        options.c_cflag &= ~CSTOPB;
    } else if (stop_bits == 2) {
        options.c_cflag |= CSTOPB;
    } else {
        return -1;
    }
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &options);
    return 0;
}

int main() {
    int printSend = 0, printRec = 0;
    char send = '1';

     int fd;

    while (1) {
        fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd != -1) {
        break;
        }
        usleep(500000);
    }

    if (set_uart_parameters(fd, B115200, 8, 1, 0) == -1) {
        perror("Error setting UART parameters");
    }

    while (1) {
        char read_buffer;
        int bytes_read = read(fd, &read_buffer, 1);
        
        if((bytes_read != -1)){
            printf("Received data: %c\n", read_buffer);
            printRec =  1;
            printSend =  0;
        }

        usleep(500000);

        int bytes_written = write(fd, &send, 1);
        tcflush(fd, TCOFLUSH);
        if (bytes_written == -1) {
            perror("Error writing to the serial port");
            usleep(500000);
        }
        else if (bytes_written < 1) {
            perror("Not all data was written, retrying");
            usleep(500000);
        }
        else {
            printf("Data sent successfully\n");
            printSend = 1;
            printRec = 0;
        }
    }
}