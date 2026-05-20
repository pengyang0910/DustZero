#ifndef __TCP_SERVER_H_
#define __TCP_SERVER_H_

int tcp_server_open(int port_number);
int tcp_server_close(int fd);
int tcp_server_write(int client_fd, char* buf, int len);
int tcp_server_read(int client_fd, char* buf, int len);
int tcp_server_read_timeout(int fd_client, char *buf, int len, int sec, int usec);



int is_requested_connections(int& server, int& client);
int is_lost_connection(int& fd_client);

/* client function */
int tcp_client_connect_to(const char* _ip, int port);
int tcp_client_close(int fd);
#endif


