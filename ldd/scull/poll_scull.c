/*************************************************************************
 * Author: huangxinghua 
 * Email: <nishizawa23@gmail.com>
 * Web: http://www.nishizawa23.com                  
 * File: poll_scull.c                   
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
#include <poll.h>
#include "test_scull.h"

#define TIMEOUT -1


int main(int argc, const char *argv[])
{
	int fd,ret;
	struct pollfd event;
	const char *path = "/dev/scullpipe0";
	char rb[50];

	fd = open(path,O_RDWR);
	if(fd == -1){
		perror("fopen");
		return 0;
	}

	while(1){
		memset(&event,0,sizeof(event));
		event.fd = fd;
		event.events = POLLIN;

		ret = poll(&event,1,TIMEOUT);

		if(ret<0){
			perror("fail poll");
			close(fd);
			exit(1);
		}

		if(TIMEOUT != -1){
			if(ret == 0){
				printf("Time out\n");
				continue;
			}
		}

		if(event.revents&POLLIN){
			read(fd,rb,sizeof(rb));
			printf("read result :%s\n",rb);
		}
	}
		
	close(fd);
	
	return 0;
}

