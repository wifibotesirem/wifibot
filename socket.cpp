#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include <sys/socket.h>
int main() {
int sockComm = socket(AF_INET,SOCK_STREAM,0); // socket en mode connecté TCP
struct sockaddr_in servAddr;
int addrLength=sizeof(struct sockaddr_in);
int buffer[1024]; // buffer accueillant le message
servAddr.sin_addr.s_addr = htonl("192.168.1.1"); // IP du serveur
servAddr.sin_family= AF_INET; // famille d’adresse IPv4
servAddr.sin_port = htons(4444); // port 4444
connect(sockComm, (struct sockaddr *)& servAddr, addrLength);
recv(sockComm, buffer, 1023, 0); // reception du message
close(sockWork); // fermeture du descripteur
exit(0);
}
