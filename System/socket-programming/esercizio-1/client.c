#include <netdb.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

void error(char *msg) {
  perror(msg);
  exit(1);
}

int main(int argc, char *argv[]) {
  int sockfd, portno, n;
  struct sockaddr_in serv_addr;
  char buffer[512];
  size_t buf_size = 512;

  if (argc < 2) {
    fprintf(stderr, "usage %s: port\n", argv[0]);
    exit(1);
  }

  portno = atoi(argv[1]);
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0)
    error("ERROR opening socket");

  bzero((char *)&serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(portno);

  if (connect(sockfd, (const struct sockaddr *)&serv_addr, sizeof(serv_addr)) <
      0)
    error("ERROR connecting");

  bzero(buffer, buf_size);
  n = read(sockfd, buffer, buf_size);
  if (n < 0)
    error("ERROR reading from socket");

  printf("%s\n", buffer);

  close(sockfd);

  return 0;
}