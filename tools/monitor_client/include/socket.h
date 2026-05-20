#ifndef	 MONITOR__SOCKET_H__
#define	 MONITOR__SOCKET_H__
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <netdb.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <limits.h>
#include <functional>

extern "C"{
//#define __SERVER__

#define  OPEN_MAX 10
#define  SOCKET_BUF_LEN		10240


typedef enum{
    DISCONNECT = -1,
    CONNECTED = 0
}socket_state_t;

class databuf_t
{
public:
    databuf_t()
    {
        data = new char[SOCKET_BUF_LEN];
    }
    ~databuf_t()
    {
        delete [] data;
    }
    int len;
    char* data;
};

int tcp_close(int fd);
int tcp_read_timeout(int fd, databuf_t *buf, int sec=0, int usec=10000);
int tcp_read_timeout2(int fd, databuf_t* buf, int sec = 0, int usec = 10000);
int tcp_write(int fd, char *buf, int len);
int tcp_read(int fd, char *buf, int len);
//int tcp_server_open(int port_number);
int tcp_server_accept(int fd_server);
int tcp_state(int tcp_fd);
int tcp_is_connect(int fd_client);
int tcp_server_setup(int port);
int tcp_server_setup_nonblock(int port);
int tcp_poll_client(int fd_server);
int tcp_connect_to(const char* url, int port);
int is_client_timeout(int fdsock, int sec, int usec);
int set_tcp_no_delay(int sockfd);
int set_tcp_non_blocking(int sockfd);
}

#endif