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
  struct hostent *server;
  char buffer[256];
  if (argc < 3) {
    fprintf(stderr, "usage %s: hostname port\n", argv[0]);
    exit(1);
  }

  // Initialize port and socket
  portno = atoi(argv[2]);
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0)
    error("ERROR opening socket");

  // Retrive server hostname
  server = gethostbyname(argv[1]);
  if (server == NULL) {
    fprintf(stderr, "ERROR, no such host\n");
    exit(1);
  }

  //
  bzero((char *)&serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr,
        server->h_length);
  serv_addr.sin_port = htons(portno);

  // Establish a connection
  if (connect(sockfd, (const struct sockaddr *)&serv_addr, sizeof(serv_addr)) <
      0)
    error("ERROR connecting");

  // Take a message as input
  printf("Please enter the message: ");
  bzero(buffer, 256);
  fgets(buffer, 255, stdin);

  // Deliver message
  n = write(sockfd, buffer, strlen(buffer));
  if (n < 0)
    error("ERROR writing to socket");

  // Read reply
  bzero(buffer, 256);
  n = read(sockfd, buffer, 256);
  if (n < 0)
    error("ERROR reading from socket");
  printf("%s\n", buffer);

  close(sockfd);

  return 0;
}