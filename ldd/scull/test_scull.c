/*************************************************************************
 * Author: huangxinghua 
 * Email: <nishizawa23@gmail.com>
 * Web: http://www.nishizawa23.com                  
 * File: test_scull.c                   
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
#include "test_scull.h"


int main(int argc, const char *argv[])
{
	int fd;
	int wd,i;
	int scull_iocsquantum = 0;
	const char *path = "/dev/scull0";
	const char *word = "hello world";

	fd = open(path,O_RDWR);
	if(fd == -1){
		perror("fopen");
		return 0;
	}

	ioctl(fd,SCULL_IOCGQUANTUM,&scull_iocsquantum);
	printf("SCULL_IOCSQUANTUM is %d\n",scull_iocsquantum);

	for(i=0;i<5;i++){
		printf("at up\n");
		wd = write(fd,word,strlen(word));
		printf("at down fwritewd = %d\n",wd);
		sleep(2);
	}

	close(fd);
	
	return 0;
}

