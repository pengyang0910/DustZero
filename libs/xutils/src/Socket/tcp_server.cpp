#include "tcp_server.h"
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef WIN32
#include <windows.h>
#include <winsock.h>

#pragma comment(lib, "ws2_32")
#else
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#endif

#include <iostream>

int tcp_server_accept(int server_fd);
int tcp_server_set_timeout_recv(int client_fd, int second, int usecond);

int tcp_server_open(int port_number) {
#ifdef _WIN32
  static bool bWinSockInit = false;
  if (!bWinSockInit) {
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
      printf("winsock load faild!\n");
      bWinSockInit = false;
      return -1;
    }
    bWinSockInit = true;
  }
#endif

  int fd_server = -1;
  int opt;
  struct sockaddr_in server_addr;

  /* �������˿�ʼ����socket������ */
  if ((fd_server = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    printf("tcp_server_open: socket ret<0 (%s)\n", strerror(errno));
    return -1;
  }

  printf("**********fd_server *************%d \n", fd_server);
  opt = 1;
  setsockopt(fd_server, SOL_SOCKET, SO_REUSEADDR, (const char *)&opt,
             sizeof(opt));

  /* ����������� sockaddr�ṹ */
  memset(&server_addr, 0, sizeof(struct sockaddr_in));
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  server_addr.sin_port = htons(port_number);

  /* ����sockfd������ */
  if (bind(fd_server, (struct sockaddr *)(&server_addr),
           sizeof(struct sockaddr_in)) < 0) {
    printf("tcp_server_open: bind ret<0 (%s  %d )\n", strerror(errno),
           port_number);
#ifdef _WIN32
    closesocket(fd_server);
#else
    close(fd_server);
#endif
    return -1;
  }

  /* ����sockfd������ */
  if (listen(fd_server, 5) == -1) {
    printf("tcp_server_open: listen ret<0 (%s)\n", strerror(errno));
#ifdef _WIN32
    closesocket(fd_server);
#else
    close(fd_server);
#endif
    return -1;
  } else {
    printf("tcp_server  is  listening\n");
  }

  return fd_server;
}

int tcp_server_close(int fd) {
  if (fd < 0)
    return -1;
#ifdef _WIN32
  return closesocket(fd);
#else
  return close(fd);
#endif
}

int tcp_server_accept(int fd_server) {
  const char chOpt = 1;
  int fd_client, sin_size;
  struct sockaddr_in client_addr;

  /* ����������,ֱ���ͻ����������� */
  sin_size = sizeof(struct sockaddr_in);
#ifdef _WIN32
  if ((fd_client = accept(fd_server, (struct sockaddr *)(&client_addr),
                          (int *)&sin_size)) < 0)
#else
  if ((fd_client = accept(fd_server, (struct sockaddr *)(&client_addr),
                          (socklen_t *)&sin_size)) < 0)
#endif
  {
    printf("tcp_server_accept: accept ret<0 (%s)\n", strerror(errno));
    return -1;
  }

  setsockopt(fd_client, IPPROTO_TCP, TCP_NODELAY, &chOpt, sizeof(char));
  printf("tcp_server_accept: server get connection from (%s)\n",
         inet_ntoa(client_addr.sin_addr));
#ifdef _WIN32
  unsigned long ul = 1;
  int r = ioctlsocket(fd_client, FIONBIO, &ul);
  if (r == SOCKET_ERROR) {
    closesocket(fd_client);
    printf("%s, %d set socket non-blocking failed\n", __FILE__, __LINE__);
    return -1;
  }
#else
  fcntl(fd_client, F_SETFL, O_NONBLOCK);
#endif

  return fd_client;
}

int tcp_server_set_timeout_recv(int fd_client, int second, int usecond) {
  struct timeval ti;

  ti.tv_sec = second;
  ti.tv_usec = usecond;
  setsockopt(fd_client, SOL_SOCKET, SO_RCVTIMEO, (const char *)&ti, sizeof(ti));
  // socklen_t len=sizeof(ti);
  // getsockopt(fd_client, SOL_SOCKET, SO_RCVTIMEO, &ti, &len);

  return 0;
}

int tcp_server_write(int fd_client, char *buf, int len) {
  int index, cnt;
  index = 0;

  while (1) {
#ifdef _WIN32
    cnt = send(fd_client, buf + index, len - index, 0);
#else
    cnt = write(fd_client, buf + index, len - index);
#endif
    //	printf("  write cnt, real_send  %d     expect_send  %d    errno=  %d
    //\n", cnt, len - index , errno);
    if (cnt > 0) {
      index += cnt;
    } else if ((cnt < 0) && ((errno == EINTR) || (errno == EWOULDBLOCK) ||
                             (errno == EAGAIN))) {
      cnt = 0;
    } else {
      printf("  write error disconnect  %d     %d    %d   %p  %d \n", cnt,
             errno, fd_client, (void *)buf, len);
      return -1;
    }

    if (index >= len)
      break;
    else {
#ifdef _WIN32
      Sleep(50);
#else
      usleep(50000);
#endif
    }
  }

  return index;
}

int tcp_server_read(int fd_client, char *buf, int len) {
  int cnt;

  if (len <= 0) {
    return 0;
  }

  cnt = recv(fd_client, buf, len, 0);

  // printf("tcp_server_read  %d \n", cnt);

  if (cnt == 0) {
    printf("tcp_server_read ==0  disconnect \n"); // disconnect
    return -1;
  }
#ifdef _WIN32
  if (cnt < 0) {
    int err = WSAGetLastError();
    if ((err == WSAEINTR) || (err == WSAEWOULDBLOCK) || (err == EAGAIN)) {
      return 0;
    } else {
      printf("tcp_server_read errno is %d\n", WSAGetLastError());
      return cnt;
    }
  }
#else
  if ((cnt < 0) &&
      ((errno == EINTR) || (errno == EWOULDBLOCK) || (errno == EAGAIN))) {
    // printf("tcp_server_read < 0  %d   \n", errno);
    return 0;
  }
  if (cnt < 0) {
    printf("tcp_server_read errno is %d\n", errno);
    return cnt;
  }
#endif

  return cnt;
}

int tcp_server_read_timeout(int fd_client, char *buf, int len, int sec,
                            int usec) {
  if (fd_client < 0)
    return -1;
  int cnt, fs_sel;
  fd_set fs_read;
  struct timeval time;

  if (len <= 0)
    return 0;

  FD_ZERO(&fs_read);
  FD_SET(fd_client, &fs_read);

  time.tv_sec = sec;
  time.tv_usec = usec;

  //ʹ��selectʵ�ִ��ڵĶ�·ͨ��
  fs_sel = select(fd_client + 1, &fs_read, NULL, NULL, &time);

  if (fs_sel > 0) {
#ifdef _WIN32
    // Sleep(1); // comment by wq 20220107
    cnt = recv(fd_client, buf, len, 0);
#else
    // usleep(1000); // comment by wq 20220107
    cnt = read(fd_client, buf, len);
#endif
    if (0 == cnt) {
      printf("seclect>0,  read error  == 0, scoket disconnect  %d   %s\n",
             errno, strerror(errno));
      return -1;
    }
    if ((cnt < 0) &&
        ((errno == EINTR) || (errno == EWOULDBLOCK) || (errno == EAGAIN))) {
      printf("seclect >0 ,read error   < 0,socket disconnect \n");
      return -1;
    }

  } else if (0 == fs_sel) {
    //	printf("tcp_server_read_timeout select  == 0\n");
    return 0;
  } else {
    printf("tcp_server_read_timeout select  < 0 \n");
    return -1;
  }

  return cnt;
}

int is_requested_connections(int &server, int &client) {
  if (server < 0)
    return -1;

  int maxfdp = server + 1;

  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(server, &fds);

  struct timeval timeout_accept = {0, 0};
  int ret = select(maxfdp, &fds, NULL, NULL, &timeout_accept);
  if (ret <= 0) {
    return -1;
  } else {
    std::cout << "client request" << std::endl;
    if (FD_ISSET(server, &fds)) {
      client = tcp_server_accept(server);
      if (client < 0) {
        printf("tcp_server_accept: fd_client==%d\n", client);
        return -1;
      }
      tcp_server_set_timeout_recv(client, 1, 0);
      int flag = 1;
      ret = setsockopt(client, IPPROTO_TCP, TCP_NODELAY, (char *)&flag,
                       sizeof(flag));
      if (ret == -1) {
        printf("Couldn't setsockopt(TCP_NODELAY)\n");
        client = -1;
        return -1;
      }

      int buffer_size = 512 * 1024; // actual value would be 2*set value
      int tmp_len = sizeof(buffer_size);
      setsockopt(client, SOL_SOCKET, SO_SNDBUF, (char *)&buffer_size, tmp_len);
#ifdef _WIN32
      getsockopt(client, SOL_SOCKET, SO_RCVBUF, (char *)&buffer_size,
                 (int *)&tmp_len);
#else
      getsockopt(client, SOL_SOCKET, SO_RCVBUF, &buffer_size,
                 (socklen_t *)&tmp_len);
#endif
      printf("Socket %d recv buffer size is %d\n", client, buffer_size);
#ifdef _WIN32
      getsockopt(client, SOL_SOCKET, SO_SNDBUF, (char *)&buffer_size,
                 (int *)&tmp_len);
#else
      getsockopt(client, SOL_SOCKET, SO_SNDBUF, &buffer_size,
                 (socklen_t *)&tmp_len);
#endif
      printf("Socket %d send buffer size is %d\n", client, buffer_size);

      std::cout << "socket connect success, fd_client = " << client
                << std::endl;
      return 1;
    } else {
      return -1;
    }
  }
}

int is_lost_connection(int &fd_client) {
#ifdef _WIN32
  int seconds = 0;
  int len = sizeof seconds;
  if (getsockopt(fd_client, SOL_SOCKET, SO_CONNECT_TIME, (char *)&seconds,
                 &len) == SOCKET_ERROR) {
    printf("socket disconnect \n");
    tcp_server_close(fd_client);
    fd_client = -1;
    return -1;
  }
  return 0;
#else
  struct tcp_info info;
  int len = sizeof(info);
  getsockopt(fd_client, IPPROTO_TCP, TCP_INFO, &info, (socklen_t *)&len);
  if ((info.tcpi_state != TCP_ESTABLISHED)) {
    printf("socket disconnect \n");
    tcp_server_close(fd_client);
    fd_client = -1;
    return -1;
  }
  return 0;
#endif
}

int tcp_client_connect_to(const char *_ip, int port) {
#ifdef _WIN32
  static bool bWinSockInit = false;
  if (!bWinSockInit) {
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
      printf("winsock load faild!\n");
      bWinSockInit = false;
      return -1;
    }
    bWinSockInit = true;
  }
#endif

  int sockfd = -1;
  // printf("%s %d\n", _ip, port);
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (-1 == sockfd) {
    printf("sock create failed\n");
    return -1;
  }

  struct sockaddr_in server;
  memset(&server, 0, sizeof(struct sockaddr_in));
  server.sin_family = AF_INET;
  server.sin_port = htons(port);
  server.sin_addr.s_addr = inet_addr(_ip);
  memset(&(server.sin_zero), 0, sizeof(server.sin_zero));

  int res = -1;
  res = connect(sockfd, (struct sockaddr *)&server, sizeof(server));
  if (-1 == res) {
    printf("sock connect error\n");
    return -1;
  }
  printf("connect to %s:%d success!\n", _ip, port);

#if 0
	// set no delay
	const char chOpt = 1;
	int ret = setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, &chOpt, sizeof(char));
	if (ret < 0)
	{
#ifdef _WIN32
		closesocket(sockfd);
#else
		close(sockfd);
#endif
		//printf("%s, %d set socket no-delay failed\n", __FILE__, __LINE__);
		//return -1;
	}
#endif

  // set non-blocking
#ifdef _WIN32
  unsigned long ul = 1;
  int r = ioctlsocket(sockfd, FIONBIO, &ul);
  if (r < 0) {
#ifdef _WIN32
    closesocket(sockfd);
#else
    close(sockfd);
#endif
    printf("%s, %d set socket non-blocking failed\n", __FILE__, __LINE__);
    return -1;
  }
#else
  fcntl(sockfd, F_SETFL, O_NONBLOCK);
#endif
  return sockfd;
}

int tcp_client_close(int fd) { return tcp_server_close(fd); }