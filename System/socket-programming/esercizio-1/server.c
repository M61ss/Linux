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

int main(void) {
  // Inizializzazione parametri statici
  int sockfd, newsockfd, portno, cli_len;
  char buffer[256];
  size_t buf_size = 256;
  struct sockaddr_in serv_addr, cli_addr;
  int n;
  portno = 2525;

  // Inizializzazione del socket
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0)
    error("ERROR opening socket");
  bzero((char *)&serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = INADDR_ANY;
  serv_addr.sin_port = htons(portno);

  // Binding del socket
  if (bind(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    error("ERROR on binding");

  // Ottengo l'hostname
  char hostname[64];
  size_t hostname_size = 64;
  bzero(hostname, hostname_size);
  if (gethostname(hostname, hostname_size) < 0)
    error("ERROR getting hostname");

  listen(sockfd, 5);
  while (1) {
    // Inizializzazione socket per la comunicazione con il client
    cli_len = sizeof(cli_addr);
    newsockfd = accept(sockfd, (struct sockaddr *)&cli_addr, &cli_len);
    if (newsockfd < 0)
      error("ERROR on accept");

    int pid = fork();
    if (pid < 0) {
      error("ERROR fork fallita");
    } else if (pid == 0) {
      printf("Fork pid: %d\n", getpid());

      // Messaggio di benvenuto
      bzero(buffer, buf_size);
      if (snprintf(buffer, buf_size, "Welcome to %s", hostname) < 0)
        error("ERROR writing hostname in buffer");
      n = write(newsockfd, buffer, strlen(buffer));
      if (n < 0)
        error("ERROR writing to socket");

      exit(0);
    }
  }

  exit(0);
}