#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>

#define SERIAL_PORT "/dev/tty.SLAB_USBtoUART"

void printMenu(void);

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

    // Set the standard input and output to non-blocking mode
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    flags = fcntl(STDOUT_FILENO, F_GETFL, 0);
    fcntl(STDOUT_FILENO, F_SETFL, flags | O_NONBLOCK);

    char input_buffer[20];
    int input_len = 0;

    while(1){
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);
        FD_SET(fd, &readfds);

        // Wait for input from the standard input or data ready to be sent on the serial port
        int maxfd = (STDIN_FILENO > fd) ? STDIN_FILENO : fd;
        int activity = select(maxfd + 1, &readfds, NULL, NULL, NULL);

        if (activity == -1) {
            perror("select");
            exit(EXIT_FAILURE);
        }

        // If input is available from the standard input, read it and send it
        if (FD_ISSET(STDIN_FILENO, &readfds)) {
            char input_char;
            int bytes_read = read(STDIN_FILENO, &input_char, 1);
            if (bytes_read > 0) {
                if (input_len < sizeof(input_buffer) - 1) {
                    input_buffer[input_len++] = input_char;
                    usleep(50000);
                }
            }
        }

        // If data is available on the serial port, read it and print it
        if (FD_ISSET(fd, &readfds)) {
            char read_buffer;
            int bytes_read = read(fd, &read_buffer, 1);
            if (bytes_read > 0) {
                printf("%c", read_buffer);
                fflush(stdout);
            }
        }

        // Process the accumulated input if a complete command is received
        if (input_len > 0 && input_buffer[input_len - 1] == '\n') {
            input_buffer[input_len - 1] = '\0';
            printf("%s\n", input_buffer);
            // Do something with the command
            input_len = 0;
        }
    }
}