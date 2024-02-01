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
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/select.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <time.h>
#include <netinet/in.h>

#define MAX_CLIENTS		10
#define CLIENT_TIMEOUT_SECS	60

#define HIGH			ioctl(uart, TIOCCBRK, 0)
#define LOW			ioctl(uart, TIOCSBRK, 0)

#define WAIT(us)							      \
do {									      \
    t.tv_usec = us;							      \
    select(0, NULL, NULL, NULL, &t);					      \
} while (0)

#define SYNC_PULSE(ms)							      \
do {									      \
    HIGH;								      \
    WAIT((ms) * 1000);							      \
} while(0)

static void
error(char *msg)
{
    fprintf(stderr, "error %s.\n", msg);
    exit(1);
}

static int
startNet(uint32_t port)
{
    struct sockaddr_in saddr =
        { .sin_port = htons(port), .sin_family = AF_INET,
          .sin_addr.s_addr = INADDR_ANY };

    int net = socket(AF_INET, SOCK_DGRAM, 0), flags;

    if ((flags = fcntl(net, F_GETFL, 0)) == -1 ||
	fcntl(net, F_SETFL, flags | O_NONBLOCK) == -1 ||
	bind(net, (struct sockaddr *) &saddr, sizeof(saddr)))
        error("starting net");

    return net;
}

static int
startUart(void)
{
    int uart;

    if ((uart = open("/dev/serial0", O_RDWR | O_NOCTTY | O_NDELAY)) == -1)
	error("opening serial port");

    struct termios options;
    tcgetattr(uart, &options);

    options.c_cflag = B4800 | CS8 | CSTOPB | CLOCAL | CREAD | PARENB;
    options.c_iflag = 0;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(uart, TCIFLUSH);
    tcsetattr(uart, TCSANOW, &options);

    return uart;
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
writeUart(int uart, void *buf, uint32_t len)
{
    struct timeval t = {};

    write(uart, buf, len);
    // start + 8 data + parity + 2 stop = 12 bits/char
    WAIT(len * 12 * 10000 / 48);
    LOW;
}

int
main(int argc, char **argv)
{
    if (argc != 2)
    {
	printf("usage: %s <port>\n", argv[0]);
	exit(1);
    }

    int uart = startUart(), net = startNet(atoi(argv[1]));
    uint8_t maskBit = 0;
    struct
    {
	struct sockaddr_in saddr;
	time_t timeout;
    } clients[MAX_CLIENTS] = {};
    fd_set uartFdSet;

    FD_ZERO(&uartFdSet);
    FD_SET(uart, &uartFdSet);

    LOW;

    while (1)
    {
	struct timeval start, t = {};

	gettimeofday(&start, 0);
	tcflush(uart, TCIFLUSH); 		// flush uart

	SYNC_PULSE(13);

	// poll bytes
	uint8_t buf[3] = {};
	writeUart(uart, buf, sizeof(buf));

	fd_set fdSet = uartFdSet;
	t.tv_usec = 12000;	// enough time to read three bytes

	if (select(uart + 1, &fdSet, NULL, NULL, &t) == 1)
	{
	    int n = read(uart, buf, sizeof(buf));

	    if (n == 3 && buf[0] == 0xff && buf[1] == 0xff && buf[2] != 0xff)
	    {
		uint8_t mask = buf[2] ^ 0xff;

		while (!(mask & (1 << maskBit)))
		    if (++maskBit == 8)
			maskBit = 0;

		WAIT(60000);	// spacing for keypad query

		// query bytes
		buf[0] = 0xf6;
		buf[1] = 16 + maskBit;
		SYNC_PULSE(4);
		writeUart(uart, buf, 2);

		if (++maskBit == 8)
		    maskBit = 0;

		t.tv_usec = 30000;
		fdSet = uartFdSet;

		uint8_t addr;

		if (select(uart + 1, &fdSet, NULL, NULL, &t) == 1 &&
		    read(uart, &addr, 1) == 1)
		{
		    uint8_t len;

		    t.tv_usec = 5000;
		    fdSet = uartFdSet;

		    if (select(uart + 1, &fdSet, NULL, NULL, &t) == 1 &&
			read(uart, &len, 1) == 1)
		    {
			uint8_t data[20 + 2];

			data[0] = addr;
			data[1] = len;

			if (len == 0x87)	// special power up message
			    len = 7;

			if (len <= sizeof(data) - 2)
			{
			    uint8_t *p = data + 2, remaining = len;

			    while (remaining)
			    {
				t.tv_usec = 5000;
				fdSet = uartFdSet;

				if (select(uart + 1, &fdSet, NULL, NULL, &t)
				    == 1)
				{
				    int n = read(uart, p, remaining);

				    if (n <= 0)
					break;

				    remaining -= n;
				    p += n;
				}
			    }

			    if (!remaining)
			    {
				uint8_t i, checksum = 0;

				for (i = 0; i < len + 2; i++)
				    checksum += data[i];

				if (!checksum)
				{
				    // ack the message
				    SYNC_PULSE(4);
				    writeUart(uart, &addr, 1);

				    time_t now = time(NULL);
				    uint32_t i;

				    // expire clients
				    for (i = 0; i < MAX_CLIENTS; i++)
					if (clients[i].saddr.sin_port &&
					    now - clients[i].timeout >=
					    CLIENT_TIMEOUT_SECS)
					    clients[i].saddr.sin_port = 0;

				    // send to clients
				    for (i = 0; i < MAX_CLIENTS; i++)
					if (clients[i].saddr.sin_port)
					    sendto(net, data, len + 2, 0,
						   (struct sockaddr *)
						   &clients[i].saddr,
						   sizeof(clients[i].saddr));
				}
			    }
			}
		    }
		}
	    }
	}

	// read data from client
	struct sockaddr_in saddr;
	socklen_t len = sizeof(saddr);
	uint8_t netBuf[50];
	ssize_t n = recvfrom(net, netBuf, sizeof(netBuf), 0,
			     (struct sockaddr *) &saddr, &len);

	if (n > 0)
	{
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

	    WAIT(50000);
	    SYNC_PULSE(4);
	    writeUart(uart, netBuf, n);
	}

	// delay for remaining period
	struct timeval now, diff;
	gettimeofday(&now, 0);
	timeval_subtract(&diff, &now, &start);
	WAIT(330 * 1000 - diff.tv_usec);
    }
}
