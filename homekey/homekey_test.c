#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

int main(int argc, char **argv)
{
    int fd;
    int key_status[6];

    //��������ʽ���豸�ļ���������ʱflags=O_NONBLOCK
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
        
        //Ӧ�ó��������ѯ����ѯ�Ƿ�ɶ��豸���з���
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
            //���豸
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
                    //��Ӧ�����а�����״̬��Ϊ0������������
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
