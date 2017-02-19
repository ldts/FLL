#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <errno.h>
#include <netdb.h>

#include "kernel_utils.h"
#include "servolib.h"

static int sockfd = -1;

static struct ip_servo {
	struct sockaddr_in serveraddr;
	struct hostent *host;
	char *hostname;
	char *name;
	int port;
	int duty;
	int calibration_step;
} server[] = {
	[pan_channel] = {
		/* db410c: gpio 36 (0 on Mezanine board) */
		.name = "pan servo",
		.hostname = "127.0.0.1",
		.calibration_step = 2,
		.port = 55555,
		.duty = 50,
	},
	[tilt_channel] = {
		/* db410c: gpio 13 (1 on Mezanine board) */
		.name = "tilt_servo",
		.hostname = "127.0.0.1",
		.calibration_step = 2,
		.port = 55556,
		.duty = MIN_DUTY,
	}
};

int servoio_set_pulse(int id, int duty)
{
	char buf[10];
	int serverlen;
	int n;

	if (sockfd < 0 || id >= 2)
		return -EIO;

	if (duty > MAX_DUTY)
		duty = MAX_DUTY;

	if (duty < MIN_DUTY)
		duty = MIN_DUTY;

	serverlen = sizeof(server[id].serveraddr);
	snprintf(buf, sizeof(buf), "%d", duty);
	n = sendto(sockfd,
		buf,
		strlen(buf),
		0,
		(struct sockaddr*) &server[id].serveraddr,
		serverlen);
	if (n < 0)
		return -EIO;

	usleep(15000);
	server[id].duty = duty;

	return 0;
}

int servoio_get_position(int id)
{
	if (sockfd < 0)
		return -EINVAL;

	if (id >= 2)
		return -EINVAL;

	usleep(15000);
	return 	server[id].duty;
}

int servoio_init(void)
{
	int i, ret;

	sockfd = socket(AF_INET, SOCK_DGRAM, 0);
	if (sockfd < 0) {
		printf("ERROR opening socket");
		return -EIO;
	}

	for (i = 0; i < 2; i++) {
		server[i].host = gethostbyname(server[i].hostname);
		if (server[i].host == NULL) {
			printf("ERROR, no such host as %s\n", server[i].hostname);
			return -EINVAL;
		}
		bzero((char *) &server[i].serveraddr, sizeof(server[i].serveraddr));
		bcopy((char *) server[i].host->h_addr, (char *) &server[i].serveraddr.sin_addr.s_addr, server[i].host->h_length);
		server[i].serveraddr.sin_port = htons(server[i].port);
		server[i].serveraddr.sin_family = AF_INET;

		ret = servoio_set_pulse(i, server[i].duty);
		if (ret < 0)
			return -EIO;
	}

	return 0;
}

