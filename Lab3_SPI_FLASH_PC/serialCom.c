// #include <fcntl.h>
// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>
// #include <termios.h>
// #include <unistd.h>

// // #define SERIAL_PORT "/dev/tty.usbmodem14103"
// #define SERIAL_PORT "/dev/tty.SLAB_USBtoUART"

// int main(){
//     uint8_t readToggle = 1;
//     int fd;
//     int bytes_read;
//     fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
//     // fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NONBLOCK);

//     struct termios options;
//     tcgetattr(fd, &options);
//     cfsetispeed(&options, B115200);
//     cfsetospeed(&options, B115200);

//     options.c_cflag |= CS8;
//     options.c_cflag &= ~PARENB;
//     options.c_cflag &= ~CSTOPB;

//     options.c_cflag |= (CLOCAL | CREAD);
//     options.c_cc[VTIME] = 1; // set timeout to 100ms
//     tcsetattr(fd, TCSANOW, &options);
    
//     while(1){
//         char cmd[50], cmdArg[50];
//         char inputCmd[100];
//         fflush(stdin);
//         fgets(inputCmd, sizeof(inputCmd), stdin);
//         int itemsRead = sscanf(inputCmd, "%s %s", cmd, cmdArg);
//         if (itemsRead == 1){
//             write(fd, cmd, strlen(cmd));
//             write(fd, "\r\n", 2);
//             // readToggle = 0;
//         }else if(itemsRead == 2){
//             write(fd, cmd, strlen(cmd));
//             write(fd, " ", 1);
//             write(fd, cmdArg, strlen(cmdArg));
//             write(fd, "\r\n", 2);
//             // readToggle = 0;
//         }
//         // usleep(10000); //50000 is visible blinkning.
//         uint8_t i;
//         char read_buffer;
//         do{
//             bytes_read = read(fd, &read_buffer, 1);
//             // if(bytes_read > 0){
//                 printf("%c", read_buffer);
//                 fflush(stdin);
//             // }
//         }while(bytes_read > 0);
//         // readToggle = 1;
//         // usleep(10000); //50000 is visible blinkning.
//     }
// }






#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#define SERIAL_PORT "/dev/tty.SLAB_USBtoUART"

void disable_raw_mode(void);
void enable_raw_mode(void);

struct termios orig_termios;

char cmd[50], cmdArg[50];
char inputCmd[100];
FILE *fp;
char ch;

int main() {
    int fd;
    int bytes_read;
    fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NDELAY);

    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cc[VTIME] = 1; // set timeout to 100ms
    tcsetattr(fd, TCSANOW, &options);

    enable_raw_mode();
    int flag = 0;
    while(1){
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(STDIN_FILENO, &rfds);
        FD_SET(fd, &rfds);
        struct timeval tv = {0, 0}; // No timeout
        fflush(stdin);


        if (select(fd+1, &rfds, NULL, NULL, &tv) > 0) {
            // There is data waiting to be read
            if (FD_ISSET(STDIN_FILENO, &rfds)) {
                // Data is waiting to be read from the terminal
                fgets(inputCmd, sizeof(inputCmd), stdin);
                int itemsRead = sscanf(inputCmd, "%s %s", cmd, cmdArg);
                if (itemsRead == 1){
                    write(fd, cmd, strlen(cmd));
                    write(fd, "\r\n", 2);
                }else if(itemsRead == 2){
                    write(fd, cmd, strlen(cmd));
                    write(fd, " ", 1);
                    write(fd, cmdArg, strlen(cmdArg));
                    write(fd, "\r\n", 2);
                }
            } else {
                // Data is waiting to be read from the serial port
                char read_buffer;
                do{
                    bytes_read = read(fd, &read_buffer, 1);
                    if(bytes_read > 0){
                        printf("%c", read_buffer);
                        fflush(stdout);  // Force the output to be printed immediately
                    }
                }while(bytes_read > 0);
            }
        }


        if (strcmp((char*)cmd, "STORE") == 0){
            fp = fopen(cmdArg, "r");

            if (fp == NULL) {
                printf("Could not open file %s", cmdArg);
                // return 1;
            }

            while((ch = fgetc(fp)) != EOF) {
                printf("%c", ch);
                write(fd, &ch, 1);
            }
            printf("\x04");
            write(fd, "\x04", 1); // Send the EOT character
            fclose(fp);

            memset((char *)cmd, '\0', strlen((char *)cmd));

        }else if(strcmp((char*)cmd, "DELETE") == 0){					//DONE

        }else if(strcmp((char*)cmd, "READ") == 0){						//DONE
            
        }else if(strcmp((char*)cmd, "DIR") == 0){
            
        }else if(strcmp((char*)cmd, "MEM") == 0){						//Double-Check functionality when STORE works

        }else if(strcmp((char*)cmd, "CLEAR") == 0){						//DONE

        }else{															//DONE
            //INVALID COMMAND

        }

    }
    return 0;
}

void disable_raw_mode(void){
  tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);
}

void enable_raw_mode(void){
  tcgetattr(STDIN_FILENO, &orig_termios);
  atexit(disable_raw_mode);
  struct termios raw = orig_termios;
  raw.c_lflag &= ~(ICANON);
  tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
}

// int handleCmd(void){
    
// }
