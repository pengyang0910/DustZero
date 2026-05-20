#include "socket.h"
extern "C" {
int tcp_write(int fd, char *buf, int len)
{
    int index, cnt;

    index = 0;
    while (1) {
        cnt = write(fd, buf + index, len - index);
        if (cnt > 0) {
            index += cnt;
        }
        else if ((cnt < 0) && ((errno == EINTR) || (errno == EWOULDBLOCK) || (errno == EAGAIN))) {
            cnt = 0;
        }
        else {
            return -1;
        }
        if (index >= len) break;
        else usleep(1000);
    }

    return index;
}
int tcp_read(int fd, char *buf, int len)
{
    int cnt;

    cnt = read(fd, buf, len);
    //printf("read %d bytes\n", cnt);
    if (cnt >= 0) return cnt;
    if ((cnt < 0) && ((errno == EINTR) || (errno == EWOULDBLOCK) || (errno == EAGAIN)))
        return cnt;
    else
        return 0;

}
int tcp_read_timeout2(int fd, databuf_t* buf, int sec, int usec)
{
    int cnt, fs_sel;
    fd_set fs_read;
    struct timeval time;

    if (-1 == fd)
    {
        printf("tcp_read error, fd=-1\r\n");
        return -1;
    }

    FD_ZERO(&fs_read);
    FD_SET(fd, &fs_read);

    time.tv_sec = sec;
    time.tv_usec = usec;


    fs_sel = select(fd + 1, &fs_read, NULL, NULL, &time);

    if (0 == fs_sel)
    {
        //printf("read timeout\r\n");
        return 0;
    }
    else if (fs_sel > 0)
    {
        usleep(2000);
        cnt = read(fd, buf->data + buf->len, SOCKET_BUF_LEN - buf->len);
        buf->len += cnt;
        printf("read %d bytes\n", cnt);
        if (cnt == 0)
            return 0;
        if ((cnt < 0) && ((errno == EINTR) || (errno == EWOULDBLOCK) || (errno == EAGAIN)))
            return 0;

        return cnt;
    }

    return 0;
}
int tcp_read_timeout(int fd, databuf_t *buf,int sec, int usec)
{
    int cnt, fs_sel;
    fd_set fs_read;
    struct timeval time;

    if (-1 == fd)
    {
        printf("tcp_read error, fd=-1\r\n");
        return -1;
    }

    FD_ZERO(&fs_read);
    FD_SET(fd, &fs_read);

    time.tv_sec = sec;
    time.tv_usec = usec;


    fs_sel = select(fd + 1, &fs_read, NULL, NULL, &time);
    if (fs_sel < 0)
    {
        //printf("read select error\r\n");
        return -1;
    }
    else if (0 == fs_sel)
    {
        //printf("read timeout\r\n");
        return 0;
    }
    else if (fs_sel > 0)
    {
        usleep(2000);
        cnt = read(fd, buf->data + buf->len, SOCKET_BUF_LEN - buf->len);
        buf->len += cnt;
        //printf("read %d bytes\n", cnt);
        if (cnt == 0)
            return -1;
        if ((cnt < 0) && ((errno == EINTR) || (errno == EWOULDBLOCK) || (errno == EAGAIN)))
            return 0;

        return cnt;
    }

    return 0;
}
int tcp_server_accept(int fd_server)
{
    const char chOpt = 1;
    int fd_client, sin_size;
    struct sockaddr_in client_addr;

    /* ����������,ֱ���ͻ����������� */
    sin_size = sizeof(struct sockaddr_in);
    if ((fd_client = accept(fd_server, (struct sockaddr *)(&client_addr),(socklen_t*) &sin_size)) < 0) {
        printf("tcp_server_accept: accept ret<0 (%s)\n", strerror(errno));
        return -1;
    }

    setsockopt(fd_client, IPPROTO_TCP, TCP_NODELAY, &chOpt, sizeof(char));
    //printf("connection from (%s)\n", inet_ntoa(client_addr.sin_addr));

    fcntl(fd_client, F_SETFL, O_NONBLOCK);
    return fd_client;
}
int tcp_server_open(int port_number)
{
    int fd_server;
    int opt;
    struct sockaddr_in server_addr;

    /* �������˿�ʼ����socket������ */
    if ((fd_server = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        printf("tcp_server_open: socket ret<0 (%s)\n", strerror(errno));
        return -1;
    }

    opt = 1;
    setsockopt(fd_server, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    /* ����������� sockaddr�ṹ */
    memset(&server_addr, 0, sizeof(struct sockaddr_in));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(port_number);

    /* ����sockfd������ */
    if (bind(fd_server, (struct sockaddr *)(&server_addr), sizeof(struct sockaddr)) < 0) {
        printf("tcp_server_open: bind ret<0 (%s)\n", strerror(errno));
        close(fd_server);
        return -1;
    }

    /* ����sockfd������ */
    if (listen(fd_server, 5) == -1) {
        printf("tcp_server_open: listen ret<0 (%s)\n", strerror(errno));
        close(fd_server);
        return -1;
    }
    else {
        printf("tcp_server  is  listening\n");
    }

    return fd_server;
}
int tcp_close(int fd)
{
    if (fd < 0) return -1;
    return close(fd);
}
#if 1
// read success or fail, ignore data, not for everyone
int tcp_is_connect(int fd_client)
{
    databuf_t buf;
    int cnt = 0;
    char ret_buf[128];
    cnt = tcp_read_timeout(fd_client, &buf, sizeof(ret_buf));

    if (cnt < 0)
    {
        tcp_close(fd_client);
        printf("disconnect!\r\n");
        return -1;
    }

    return 0;
}
#endif
int tcp_server_setup(int port)
{
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);

    struct sockaddr_in my_addr;
    bzero(&my_addr, sizeof(my_addr));
    my_addr.sin_family = AF_INET;
    my_addr.sin_port = htons(port);
    my_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    bind(sockfd, (struct sockaddr *)&my_addr, sizeof(my_addr));

    listen(sockfd, 10);
    return sockfd;


}

int tcp_server_setup_nonblock(int port)
{
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);

    struct sockaddr_in my_addr;
    bzero(&my_addr, sizeof(my_addr));
    my_addr.sin_family = AF_INET;
    my_addr.sin_port = htons(port);
    my_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    int flags = fcntl(sockfd, F_GETFL, 0);
    fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);
    bind(sockfd, (struct sockaddr *)&my_addr, sizeof(my_addr));

    listen(sockfd, 10);
    return sockfd;
}

int tcp_poll_client(int fd_server)
{
    struct pollfd client;
    client.fd = fd_server;
    client.events = POLLIN;

    int ret = poll(&client, 1, 100);

    if ((client.revents & POLLIN) == POLLIN)
    {
        struct sockaddr_in cli_addr;
        int clilen = sizeof(cli_addr);
        int connfd = 0;
        connfd = accept(fd_server, (struct sockaddr *)&cli_addr, (socklen_t*)&clilen);
        //printf("connection from (%s) fd=%d\r\n", inet_ntoa(cli_addr.sin_addr), connfd);
        return connfd;
    }

    return -1;
}

int tcp_connect_to(const char* _ip, int port)
{
    int sockfd = -1;
    //printf("%s %d\n", _ip, port);
    sockfd = socket( AF_INET, SOCK_STREAM, 0 );
    if ( -1 == sockfd ) {
        printf( "sock created" );
        return -1;
    }

    struct sockaddr_in server;
    memset( &server, 0, sizeof( struct sockaddr_in ) );
    server.sin_family = AF_INET;
    server.sin_port = htons(port);
    server.sin_addr.s_addr = inet_addr(_ip);
    bzero(&(server.sin_zero),sizeof(server.sin_zero));

    int res = -1;
    res = connect( sockfd, (struct sockaddr*)&server, sizeof( server ) );
    if( -1 == res ){
        printf( "sock connect error\n" );
        return -1;
    }
    printf("connect to %s:%d success!\n",_ip, port);

    return sockfd;
}

int set_tcp_no_delay(int sockfd)
{
    const char chOpt = 1;
    return setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, &chOpt, sizeof(char));
}

int set_tcp_non_blocking(int sockfd)
{
    return fcntl(sockfd, F_SETFL, O_NONBLOCK);
}

}

int is_client_timeout(int fdsock, int sec, int usec){
    fd_set fs_read;
    int cnt, fs_sel;
    struct timeval time;
    FD_ZERO(&fs_read);
    FD_SET(fdsock, &fs_read);
    time.tv_sec = sec;
    time.tv_usec = usec;
    fs_sel = select(fdsock + 1, &fs_read, NULL, NULL, &time);
    if(fs_sel > 0){
        return fs_sel;
    }else if(fs_sel == 0){
        return 0;
    }else {
        return -1;
    }
}
