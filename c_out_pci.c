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
    
    char *portname0 = "/dev/tty0";
    char *portname1 = "/dev/ttyUSB1";

    int fd0 = open(portname0, O_RDWR | O_NOCTTY | O_SYNC);
    
    if (fd0 < 0) {
	fprintf(stderr, "error %d opening %s: %s", errno, portname0, strerror(errno));
	return;
    }

    set_interface_attribs(fd0, B115200, 0);
    set_blocking(fd0, 0);
    
    int cnt = 0;
    while(1){
    	write(fd0, "hello!\n", 7);
	printf("%d hello\n",cnt++);	
    	usleep((7+25)*100);
    }
/*
    char buf[100];


    int fd1 = open(portname1, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd1 < 0) {
	fprintf(stderr, "error %d opening %s: %s", errno, portname1, strerror(errno));
	return;
    }
    set_interface_attribs(fd1, B115200, 0);
    set_blocking(fd1, 0);

    int cnt = 0;
    while(1){
        int n = read(fd1, buf, sizeof buf);
        printf("%s",buf);
        usleep((7+25)*100);
    }
*/
    return 0;
}
