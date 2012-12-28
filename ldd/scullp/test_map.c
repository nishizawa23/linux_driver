/*************************************************************************
 * Author: huangxinghua 
 * Email: <nishizawa23@gmail.com>
 * Web: http://www.nishizawa23.com                  
 * File: test_map.c                   
 * Create Date: 2012-12-14 00:12:10
 *************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
//#include "test_scull.h"


int main(int argc, const char *argv[])
{
	int fd;
	int wd,i;
	int scull_iocsquantum = 0;
	const char *path = "/dev/scullp0";
	const char *word = "hello world";
	char *vadr;

	fd = open(path,O_RDWR);
	if(fd == -1){
		perror("fopen");
		return 0;
	}

	vadr = mmap(0,100000,PROT_READ|PROT_WRITE,MAP_SHARED,fd,1<<12);

	close(fd);

	vadr[0] = 'w';
	vadr[1] = 'o';
	vadr[2] = 'r';
	vadr[3] = 'd';

	close(fd);

	sleep(20);

	munmap((void*)vadr,100000-(1<<12));
	
	return 0;
}

