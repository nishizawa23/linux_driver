/*************************************************************************
 * Author: huangxinghua 
 * Email: <nishizawa23@gmail.com>
 * Web: http://www.nishizawa23.com                  
 * File: async_scull.c                   
 * Create Date: 2012-12-14 00:12:10
 *************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>
#include "test_scull.h"

int fd;
char rb[50];

void sig_handler(int sig)
{
	read(fd,rb,sizeof(rb));
	printf("read is: %s\n",rb);
}

int main(int argc, const char *argv[])
{
	int f_flags;

	const char *path = "/dev/scullpipe0";

	fd = open(path,O_RDWR);
	if(fd == -1){
		perror("fopen");
		return 0;
	}

	signal(SIGIO,sig_handler);
	fcntl(fd,F_SETOWN,getpid());
	f_flags = fcntl(fd,F_GETFL);
	fcntl(fd,F_SETFL,FASYNC|f_flags);

	while(1){
		printf("waiting \n");
		sleep(4);
	}

	close(fd);
	
	return 0;
}

