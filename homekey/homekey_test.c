#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

int main(int argc, char **argv)
{
    int fd;
    int key_status[6];

    //以阻塞方式打开设备文件，非阻塞时flags=O_NONBLOCK
    fd = open("/dev/my2440_buttons", 0);

    if(fd < 0)
    {
        printf("Open Buttons Device Faild!\n");
        exit(1);
    }

    while(1)
    {
        int i;
        int ret;
        fd_set rds;
        
        FD_ZERO(&rds);
        FD_SET(fd, &rds);
        
        //应用程序进行轮询，查询是否可对设备进行访问
        ret = select(fd + 1, &rds, NULL, NULL, NULL);
        
        if(ret < 0)
        {
            printf("Read Buttons Device Faild!\n");
            exit(1);
        }
        
        if(ret == 0)
        {
            printf("Read Buttons Device Timeout!\n");
        }
        else if(FD_ISSET(fd, &rds))
        {
            //读设备
            ret = read(fd, key_status, sizeof(key_status));

            if(ret != sizeof(key_status))
            {
                if(errno != EAGAIN)
                {
                    printf("Read Button Device Faild!\n");
                }

                continue;
            }
            else
            {
                for(i = 0; i < 6; i++)
                {
                    //对应驱动中按键的状态，为0即按键被按下
                    if(key_status[i] == 0)
                    {
                        printf("Key%d DOWN\n", i + 1);
                    }
                }
            }
        }
    }

    close(fd);

    return 0;
}
