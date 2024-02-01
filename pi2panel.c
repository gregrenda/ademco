/*
MIT License

Copyright (c) 2023 Greg Renda

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/select.h>
#include <sys/time.h>
#include <termios.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <signal.h>
#include <time.h>

#define GPIO_PIN		4

#define FRAME_MS		176	// frame width in milliseconds

#define GPIO_PATH 		"/sys/class/gpio/"
#define STRINGIFY(pin)		#pin
#define GPIO_PIN_PATH(pin)	GPIO_PATH "gpio" STRINGIFY(pin) "/"

#define MAX_CLIENTS		10
#define CLIENT_TIMEOUT_SECS	60

static void
terminate(int sig)
{
    FILE *fp;

    if ((fp = fopen(GPIO_PATH "unexport", "w")))
    {
	fprintf(fp, "%d", GPIO_PIN);
	fclose(fp);
    }

    exit(0);
}

static void
error(char *msg)
{
    fprintf(stderr, "error %s.\n", msg);
    terminate(0);
}

static bool
timeval_subtract(struct timeval *result, struct timeval *x, struct timeval *y)
{
    // Perform the carry for the later subtraction by updating y
    if (x->tv_usec < y->tv_usec)
    {
	int nsec = (y->tv_usec - x->tv_usec) / 1000000 + 1;
	y->tv_usec -= 1000000 * nsec;
	y->tv_sec += nsec;
    }

    if (x->tv_usec - y->tv_usec > 1000000)
    {
	int nsec = (y->tv_usec - x->tv_usec) / 1000000;
	y->tv_usec += 1000000 * nsec;
	y->tv_sec -= nsec;
    }

    result->tv_sec = x->tv_sec - y->tv_sec;
    result->tv_usec = x->tv_usec - y->tv_usec;

    return x->tv_sec < y->tv_sec;    // Return true if result is negative.
}

static void
setUartBits(int fd, uint32_t numBits)
{
    struct termios options;
    tcgetattr(fd, &options);

    options.c_cflag = B2400 | numBits | CSTOPB | CLOCAL | CREAD | PARENB;
    options.c_iflag = 0;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &options);
}

static int
startNet(uint32_t port)
{
    struct sockaddr_in saddr =
        { .sin_port = htons(port), .sin_family = AF_INET,
          .sin_addr.s_addr = INADDR_ANY };

    int fd = socket(AF_INET, SOCK_DGRAM, 0);

    if (bind(fd, (struct sockaddr *) &saddr, sizeof(saddr)))
        error("binding socket");

    return fd;
}

static int
initGpio(void)
{
    int fd;
    FILE *fp;

    if (!(fp = fopen(GPIO_PATH "export", "w")))
	error("exporting gpio");

    fprintf(fp, "%d", GPIO_PIN);
    fclose(fp);

    if (!(fp = fopen(GPIO_PIN_PATH(GPIO_PIN) "direction", "w")))
	error("setting gpio direction " GPIO_PIN_PATH(GPIO_PIN) "direction");

    fprintf(fp, "in");
    fclose(fp);

    if (!(fp = fopen(GPIO_PIN_PATH(GPIO_PIN) "edge", "w")))
	error("setting gpio edge");

    fprintf(fp, "rising");
    fclose(fp);

    if ((fd = open(GPIO_PIN_PATH(GPIO_PIN) "value", O_RDONLY)) < 0)
	error("opening gpio");

    return fd;
}

int
main(int argc, char **argv)
{
    int gpio, uart, net;

    if (argc != 2)
    {
	printf("usage: %s <port>\n", argv[0]);
	exit(1);
    }

    signal(SIGINT, terminate);

    gpio = initGpio();

    if ((uart = open("/dev/serial0", O_RDWR | O_NOCTTY | O_NDELAY)) == -1)
	error("opening serial port");

    setUartBits(uart, CS8);

    net = startNet(atoi(argv[1]));

    fd_set gpioSet, fdSet;
    FD_ZERO(&gpioSet);
    FD_SET(gpio, &gpioSet);

    FD_ZERO(&fdSet);
    FD_SET(net, &fdSet);
    FD_SET(uart, &fdSet);

    int maxFd = gpio > net ? gpio : net;
    uint8_t keys[10], numKeys = 0, *key;
    struct timeval last;
    struct
    {
	struct sockaddr_in saddr;
	time_t timeout;
    } clients[MAX_CLIENTS] = {};

    gettimeofday(&last, 0);

    while (1)
    {
	fd_set exceptSet = gpioSet, readSet = fdSet;

	if (select(maxFd + 1, &readSet,  NULL, &exceptSet, NULL) < 1)
	    break;

	if (FD_ISSET(gpio, &exceptSet))	// gpio interrupt
	{
	    struct timeval now, diff;
	    gettimeofday(&now, 0);

	    uint8_t c;
	    lseek(gpio, 0, SEEK_SET);
	    read(gpio, &c, sizeof(c));

	    timeval_subtract(&diff, &now, &last);
	    last = now;

	    // wait for the start of a sync pulse
	    if (!diff.tv_sec && diff.tv_usec > (FRAME_MS - 5) * 1000 && numKeys)
	    {
		uint8_t buf[] = { *key, *key, *key };
		numKeys--;
		key++;
		usleep(4000);
		setUartBits(uart, CS5);
		write(uart, buf, sizeof(buf));
		usleep(12000);	// wait for the bytes to be sent
		setUartBits(uart, CS8);
	    }
	}

	if (FD_ISSET(net, &readSet))	// data from network
	{
	    struct sockaddr_in saddr;
	    socklen_t len = sizeof(saddr);
	    uint8_t buf[sizeof(keys)];
	    ssize_t n = recvfrom(net, buf, sizeof(buf), 0,
				 (struct sockaddr *) &saddr, &len);
	    uint32_t i, c = MAX_CLIENTS;
	    time_t now = time(NULL);

	    // expire clients and look for matching client
	    for (i = 0; i < MAX_CLIENTS; i++)
		if (clients[i].saddr.sin_port)
		{
		    if (now - clients[i].timeout >= CLIENT_TIMEOUT_SECS)
			clients[i].saddr.sin_port = 0;
	            else if (c == MAX_CLIENTS &&
			     !memcmp(&saddr, &clients[i].saddr, len))
			c = i;
		}

	    if (c == MAX_CLIENTS)		   // not a current client
		for (i = 0; i < MAX_CLIENTS; i++)  // look for an empty slot
		    if (!clients[i].saddr.sin_port)
		    {
			c = i;
			memcpy(&clients[c].saddr, &saddr, len);
			break;
		    }

	    if (c != MAX_CLIENTS)	// set client timeout
		clients[c].timeout = now;

	    if (!numKeys && buf[0] != 0xff)	// 0xff is used for a keepalive
	    {
		memcpy(keys, buf, n);
		key = keys;
		numKeys = n;
	    }
	}

	if (FD_ISSET(uart, &readSet))	// data from uart
	{
	    uint8_t buf[10];
	    int n = read(uart, buf, sizeof(buf));
	    time_t now = time(NULL);
	    uint32_t i;

	    // expire clients
	    for (i = 0; i < MAX_CLIENTS; i++)
		if (clients[i].saddr.sin_port &&
		    now - clients[i].timeout >= CLIENT_TIMEOUT_SECS)
		    clients[i].saddr.sin_port = 0;

	    if (n == 6)
		for (i = 0; i < MAX_CLIENTS; i++)
		    if (clients[i].saddr.sin_port)
			sendto(net, buf + 1, 4, 0,
			       (struct sockaddr *) &clients[i].saddr,
			       sizeof(clients[i].saddr));
	}
    }

    return 0;
}
