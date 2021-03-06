#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <string.h>

int set_interface_attribs (int fd, int speed, int parity) {
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0) {
	fprintf(stderr, "error %s from tcgetattr", strerror(errno));
	return -1;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &=(PARENB | PARODD);
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if( tcsetattr (fd, TCSANOW, &tty) != 0){
	fprintf(stderr, "error %d from tcsetattr", errno);
	return -1;
    }

    return 0;
}

void set_blocking(int fd, int should_block) {
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if(tcgetattr(fd, &tty) != 0) {
	fprintf(stderr, "error %d from tggetattr", errno);
	return;
    }

    tty.c_cc[VMIN] = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
	fprintf(stderr, "error %d setting term attributes", errno);
    }
}


int main(int argc, char **argv){
    
    char *portname0 = "/dev/ttyUSB1";
    char *portname1 = "/dev/ttyUSB0";

    int fd0 = open(portname0, O_RDWR | O_NOCTTY | O_SYNC);
    
    if (fd0 < 0) {
	fprintf(stderr, "error %d opening %s: %s", errno, portname0, strerror(errno));
	return;
    }

    set_interface_attribs(fd0, B9600, 0); //B115200
    set_blocking(fd0, 0);
    
    int fd1 = open(portname1, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd1 < 0) {
	fprintf(stderr, "error %d opening %s: %s", errno, portname1, strerror(errno));
	return;
    }
    set_interface_attribs(fd1, B9600, 0);
    set_blocking(fd1, 0);

    int num = 0;

while(1){
    	write(fd0, "1\n", 7);
	printf("1\n");	
        usleep((7+25)*1000);

        char buf[100];

        int n = read(fd1, buf, sizeof buf);
        printf("%d: %s",n, buf);
	
        scanf("%d",&num);
}	
    return 0;
}
