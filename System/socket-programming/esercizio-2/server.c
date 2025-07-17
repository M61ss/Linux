#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

void error(char *msg) {
  perror(msg);
  exit(1);
}

int main(void) {
  int sockfd, newsockfd, portno, cli_len;
  char buffer[256];
  struct sockaddr_in serv_addr, cli_addr;
  int n;

  portno = 2525;
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0)
    error("ERROR opening socket");

  bzero((char *)&serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = INADDR_ANY;
  serv_addr.sin_port = htons(portno);

  if (bind(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    error("ERROR on binding");

  listen(sockfd, 5);
  while (1) {
    cli_len = sizeof(cli_addr);
    newsockfd = accept(sockfd, (struct sockaddr *)&cli_addr, &cli_len);
    if (newsockfd < 0)
      error("ERROR on accept");

    int pid = fork();
    if (pid < 0) {
      printf("WARNING fork failed.\n");
    } else if (pid == 0) {
      bzero(buffer, 256);
      n = read(newsockfd, buffer, 255);
      if (n < 0)
        error("ERROR reading from socket");

      printf("Connection from: %s\n", buffer);

      exit(0);
    }
  }

  exit(0);
}