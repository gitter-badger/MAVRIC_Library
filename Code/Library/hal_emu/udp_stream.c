#include "udp_stream.h"

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
/* Linux / MacOS POSIX timer headers */
#include <sys/time.h>
#include <time.h>
#include <arpa/inet.h>
#include "print_util.h"

static inline void udp_put_byte(udp_connection_t *udpconn, char data) {
	buffer_put(&udpconn->udp_buffer,data);
	
}

static inline void udp_transmit(udp_connection_t *udpconn) {
	int bytes_sent = sendto(udpconn->sock, &udpconn->udp_buffer, buffer_bytes_available(&udpconn->udp_buffer), 0, (struct sockaddr*)&(udpconn->Addr), sizeof(struct sockaddr_in));
	buffer_clear(&udpconn->udp_buffer);
}

static inline bool udp_buffer_empty(udp_connection_t *udpconn) {
	if (buffer_bytes_available(&udpconn->udp_buffer)!=0) {
		udp_transmit(udpconn);
	}
	return (buffer_bytes_available(&udpconn->udp_buffer) ==0);
}

static inline int udp_bytes_available(udp_connection_t *udpconn) {
	char buf[BUFFER_SIZE];
	socklen_t fromlen;
	int i;

	int recsize = recvfrom(udpconn->sock, (void *)buf, BUFFER_SIZE, 0, (struct sockaddr *)&(udpconn->Addr), &fromlen);
	for (i=0; i<recsize; i++) {
		buffer_put(&udpconn->udp_buffer, buf[i]);
	}
	return buffer_bytes_available(&udpconn->udp_buffer);
	
}

static inline char udp_get_byte(udp_connection_t *udpconn) {
	
	return buffer_get(&udpconn->udp_buffer);
}

void register_write_stream_udp(byte_stream_t *stream, udp_connection_t *udpconn, const char* target_ip, int port) {
 
	//udpconn->sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
	memset(&udpconn->Addr, 0, sizeof(&udpconn->Addr));
	udpconn->Addr.sin_family = AF_INET;
	udpconn->Addr.sin_addr.s_addr = inet_addr(target_ip);
	udpconn->Addr.sin_port = htons(port);
	
	buffer_init(&udpconn->udp_buffer);
	stream->data=udpconn;
	stream->put=&udp_put_byte;
	stream->flush=&udp_transmit;
	stream->get=NULL;
	stream->buffer_empty=udp_buffer_empty;
}

void register_read_stream_udp(byte_stream_t *stream, udp_connection_t *udpconn, int port) {
	udpconn->sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);

	memset(&udpconn->Addr, 0, sizeof(udpconn->Addr));
	udpconn->Addr.sin_family = AF_INET;
	udpconn->Addr.sin_addr.s_addr = INADDR_ANY;
	udpconn->Addr.sin_port = htons(port);
 
	/* Bind the socket to port 14551 - necessary to receive packets from qgroundcontrol */ 
	if (-1 == bind(udpconn->sock,(struct sockaddr *)&(udpconn->Addr), sizeof(struct sockaddr)))
    {
		printf("error bind failed\n");
		close(udpconn->sock);
		exit(-1);
    } 
 
	/* Attempt to make it non blocking */
	if (fcntl(udpconn->sock, F_SETFL, O_NONBLOCK | FASYNC) < 0)
    {
		fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
		close(udpconn->sock);
		exit(-1);
    }
	
	buffer_init(&udpconn->udp_buffer);
	
	stream->data=udpconn;
	stream->bytes_available=&udp_bytes_available;
	stream->get=&udp_get_byte;

}
