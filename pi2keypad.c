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
#include <fcntl.h>
#include <signal.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <termios.h>
#include <sys/select.h>

#include "bcm2835.h"

#define BIT_RATE		4800
#define FRAME_MS		176
#define SYNC_MS			17
#define PWM_HZ			1000000

#define DMA_CHANNEL_PWM		7
#define DMA_CHANNEL_SPI_RX	8
#define DMA_CHANNEL_SPI_TX	9

#define MEM_BUS_ADDR(m, a)	((uint32_t) (a) - (uint32_t) (m)->virt +      \
				 (uint32_t) (m)->bus)
#define REG_BUS_ADDR(m, r)	((uint32_t) m.bus + (uint32_t) r)
#define BUS_PHYS_ADDR(a)	((void *) ((uint32_t) (a) & ~0xc0000000))

#define SPI(r)			REG32(g->spiRegs, SPI_ ## r)
#define GPIO(r)			REG32(g->gpioRegs, GPIO_ ## r)
#define PWM(r)			REG32(g->pwmRegs, PWM_ ## r)
#define CLK(r)			REG32(g->clkRegs, CLK_ ## r)
#define DMA(ch, r)		REG32(g->dmaRegs, DMA_REG(ch, DMA_ ## r))

typedef enum
{
    MEM_FLAG_DISCARDABLE    = 1 << 0,	// can be resized to 0 at any time.
					// Use for cached data
    MEM_FLAG_NORMAL         = 0 << 2,	// normal allocating alias.
					// Don't use from ARM
    MEM_FLAG_DIRECT         = 1 << 2,	// 0xc alias uncached
    MEM_FLAG_COHERENT       = 2 << 2,	// 0x8 alias.
					// Non-allocating in L2 but coherent
    MEM_FLAG_ZERO           = 1 << 4,	// initialise buffer to all zeros
    MEM_FLAG_NO_INIT        = 1 << 5,	// don't initialise
					// (default is initialise to all ones)
    MEM_FLAG_HINT_PERMALOCK = 1 << 6,	// Likely to be locked for long periods
					// of time
    MEM_FLAG_L1_NONALLOCATING = MEM_FLAG_DIRECT | MEM_FLAG_COHERENT
					// Allocating in L2
} VC_ALLOC_FLAGS;

#define DMA_MEM_FLAGS MEM_FLAG_COHERENT

typedef struct
{
    uint32_t len,   // Overall length (bytes)
        req,        // Zero for request, 1<<31 for response
        tag,        // Command number
        blen,       // Buffer length (bytes)
        dlen,       // Data length (bytes)
	uints[32-5];// Data (108 bytes maximum)
} VC_MSG __attribute__ ((aligned (16)));

typedef struct
{
    int fd,         // File descriptor
        h,          // Memory handle
        size;       // Memory size
    void *bus,      // Bus address
	 *virt,     // Virtual address
	 *phys;     // Physical address
} MEM_MAP;

// shave off a few bytes so we don't overrun our frame timing
// we never set bits in the last few bytes anyway
#define FRAME_DATA_SIZE							      \
    (((FRAME_MS * 1000000 / (1000000 / BIT_RATE) + 500) / 8000) - 5)

typedef struct
{
    DMA_CB cb[18];
    uint32_t spiCS, spiDLEN, onceLink, dummy;
    uint8_t pulse[FRAME_DATA_SIZE],
	ping[FRAME_DATA_SIZE], pong[FRAME_DATA_SIZE], once[FRAME_DATA_SIZE];
} DMA_DATA;

typedef struct
{
    MEM_MAP spiRegs, dmaRegs, gpioRegs, pwmRegs, clkRegs, vcMem;
    DMA_DATA *dma;
    int net, serial;
    struct
    {
	uint8_t *buf;
	uint32_t link;
    } pingPong[2];
    uint32_t pp;
} Globals;

#define PING_PONG(_msg)							      \
do {									      \
    setMsg(g->pingPong[g->pp & 1].buf, (uint8_t *) (_msg), 4);		      \
    g->dma->cb[DMA_CB_TX_LINK].next = g->pingPong[g->pp & 1].link;	      \
    g->pp++;								      \
} while (0)

#define ONCE(_msg1, _msg2)						      \
do {									      \
    setMsg(g->dma->once, (uint8_t *) (_msg1), 4);			      \
    setMsg(g->pingPong[g->pp & 1].buf, (uint8_t *) (_msg2), 4);		      \
    g->dma->onceLink = g->pingPong[g->pp & 1].link;			      \
    g->dma->cb[DMA_CB_TX_LINK].next = NEXT(DMA_CB_TX_ONCE);		      \
    g->pp++;								      \
} while (0)

static Globals _globals; // needs to be global for signal function access
static void terminate(int sig);

static void
error(char *msg)
{
    fprintf(stderr, msg);
    terminate(0);
}

static void *
mapSegment(void *addr, uint32_t size)
{
    int fd;

    if ((fd = open("/dev/mem", O_RDWR|O_SYNC|O_CLOEXEC)) < 0)
        error("Error: can't open /dev/mem, run using sudo\n");

    void *mem = mmap(0, PAGE_ROUNDUP(size), PROT_WRITE | PROT_READ, MAP_SHARED,
		     fd, (uint32_t) addr);
    close(fd);

    if (mem == MAP_FAILED)
        error("Error: can't map memory\n");

    return mem;
}

static void
unmapSegment(void *mem, uint32_t size)
{
    if (mem)
        munmap(mem, PAGE_ROUNDUP(size));
}

static int
openMbox(void)
{
   int fd;

   if ((fd = open("/dev/vcio", 0)) < 0)
       error("Error: can't open VC mailbox\n");

   return fd;
}

static void
closeMbox(int fd)
{
    if (fd >= 0)
        close(fd);
}

static uint32_t
msgMbox(int fd, VC_MSG *msgp)
{
    uint32_t ret = 0, i;

    for (i = msgp->dlen / 4; i <= msgp->blen / 4; i += 4)
        msgp->uints[i++] = 0;

    msgp->len = (msgp->blen + 6) * 4;
    msgp->req = 0;

    if (ioctl(fd, _IOWR(100, 0, void *), msgp) < 0)
        printf("VC IOCTL failed\n");
    else if (!(msgp->req & 0x80000000))
        printf("VC IOCTL error\n");
    else if (msgp->req == 0x80000001)
        printf("VC IOCTL partial error\n");
    else
        ret = msgp->uints[0];

    return ret;
}

static uint32_t
allocVcMem(int fd, uint32_t size, VC_ALLOC_FLAGS flags)
{
    VC_MSG msg = { .tag = 0x3000c, .blen = 12, .dlen = 12,
		   .uints = { PAGE_ROUNDUP(size), PAGE_SIZE, flags } };

    return msgMbox(fd, &msg);
}

static void *
lockVcMem(int fd, int h)
{
    VC_MSG msg = { .tag = 0x3000d, .blen = 4, .dlen = 4, .uints = { h } };

    return h ? (void *) msgMbox(fd, &msg) : 0;
}

static uint32_t
unlockVcMem(int fd, int h)
{
    VC_MSG msg = { .tag = 0x3000e, .blen = 4, .dlen = 4, .uints = { h } };

    return h ? msgMbox(fd, &msg) : 0;
}

static uint32_t
freeVcMem(int fd, int h)
{
    VC_MSG msg = { .tag = 0x3000f, .blen = 4, .dlen = 4, .uints = { h } };

    return h ? msgMbox(fd, &msg) : 0;
}

static void *
mapUncachedMem(MEM_MAP *mp, uint32_t size)
{
    mp->size = PAGE_ROUNDUP(size);
    mp->fd = openMbox();

    return (mp->h = allocVcMem(mp->fd, mp->size, DMA_MEM_FLAGS)) > 0 &&
        (mp->bus = lockVcMem(mp->fd, mp->h)) &&
        (mp->virt = mapSegment(BUS_PHYS_ADDR(mp->bus), mp->size)) ? mp->virt : 0;
}

static void *
mapPeriphMem(MEM_MAP *mp, void *phys, uint32_t size)
{
    mp->phys = phys;
    mp->size = PAGE_ROUNDUP(size);
    mp->bus = (uint8_t *) phys - (uint8_t *) PHYS_REG_BASE +
	(uint8_t *) BUS_REG_BASE;
    return mp->virt = mapSegment(phys, mp->size);
}

static void
unmapPeriphMem(MEM_MAP *mp)
{
    if (mp)
    {
	unmapSegment(mp->virt, mp->size);

        if (mp->fd)
        {
            unlockVcMem(mp->fd, mp->h);
            freeVcMem(mp->fd, mp->h);
            closeMbox(mp->fd);
        }
    }
}

static void
gpioPull(Globals *g, uint32_t pin, uint32_t pull)
{
    volatile uint32_t *reg = GPIO(GPPUDCLK0) + (pin >> 5);

    *GPIO(GPPUD) = pull;
    usleep(2);
    *reg = 1 << (pin & 0x1f);
    usleep(2);
    *GPIO(GPPUD) = 0;
    *reg = 0;
}

static void
gpioMode(Globals *g, uint32_t pin, uint32_t mode)
{
    volatile uint32_t *reg = GPIO(FSEL0) + pin / 10,
	shift = (pin % 10) * 3;

    *reg = (*reg & ~(7 << shift)) | (mode << shift);
}

static void
gpioSet(Globals *g, uint32_t pin, uint32_t mode, uint32_t pull)
{
    gpioMode(g, pin, mode);
    gpioPull(g, pin, pull);
}

static void
startDma(Globals *g, MEM_MAP *mp, uint32_t ch, DMA_CB *cb)
{
    *DMA(ch, CONBLK_AD) = MEM_BUS_ADDR(mp, cb);
    *DMA(ch, CS) = DMA_CS_END;
    *DMA(ch, DEBUG) = DMA_DEBUG_ERRORS;
    *DMA(ch, CS) = DMA_CS_ACTIVE;
}

static uint8_t *
setBits(uint8_t *p, uint32_t *shift, uint32_t value, uint32_t n)
{
    uint32_t sh = *shift;

    while (n--)
    {
	*p = (*p & ~(1 << sh)) | (value << sh);

	if (sh)
	    sh--;
	else
	{
	    p++;
	    sh = 7;
	}
    }

    *shift = sh;
    return p;
}

// set usecs worth of bits
#define setBitsUs(p, shift, value, usecs)				      \
    setBits(p, shift, value,						      \
	    ((((usecs) * 1000) / (1000000 / BIT_RATE) + 500) / 1000))

// the SPI rate is 2x our desired uart baud rate so each uart bit is 2 SPI bits
#define setUartBit(p, shift, value) setBits(p, shift, value, 2)

// start message around the middle of the frame
#define MSG_START_BIT (FRAME_DATA_SIZE * 8 / 2)

static void
setMsg(uint8_t *mem, uint8_t *msg, uint32_t len)
{
    uint8_t *p = mem + (MSG_START_BIT >> 3);
    uint32_t shift = 7 - (MSG_START_BIT & 7), i;

    // 6ms on, 6ms off
    for (i = 0; i < 2; i++)
	p = setBitsUs(p, &shift, 1 - i, 6000);

    while (len--)
    {
	p = setBitsUs(p, &shift, 1, 1000);	// 1ms pulse

	p = setUartBit(p, &shift, 0);		// start bit

	// shift out byte lsb first with even parity
	uint8_t m, parity = 0;

	for (m = 1; m; m <<= 1)
	{
	    uint8_t bit = (*msg & m) ? 1 : 0;
	    parity ^= bit;
	    p = setUartBit(p, &shift, bit);
	}

	p = setUartBit(p, &shift, parity);	// parity bit
	msg++;
    }

    setBitsUs(p, &shift, 1, 1000);		// 1ms pulse
}

static void
startKeypad(Globals *g)
{
    mapPeriphMem(&g->spiRegs, (void *) SPI0_BASE, PAGE_SIZE);
    mapPeriphMem(&g->dmaRegs, (void *) DMA_BASE, PAGE_SIZE);
    mapPeriphMem(&g->gpioRegs, (void *) GPIO_BASE, PAGE_SIZE);
    mapPeriphMem(&g->pwmRegs, (void *) PWM_BASE, PAGE_SIZE);
    mapPeriphMem(&g->clkRegs, (void *) CLK_BASE, PAGE_SIZE);

    mapUncachedMem(&g->vcMem, sizeof(DMA_DATA));

    g->dma = g->vcMem.virt;

    DMA_DATA data =
    {
#define NEXT(_next) MEM_BUS_ADDR(&g->vcMem, &g->dma->cb[_next])

#define DMA_CB_PWM 0
	// pwm channel - triggers dma transfers
	.cb[DMA_CB_PWM] =
	{
	    .ti = DMA_TI_DEST_DREQ | DMA_TI_PERMAP(DMA_PWM_DREQ) |
	          DMA_TI_WAIT_RESP,
	    .src = MEM_BUS_ADDR(&g->vcMem, &g->dma->dummy),
	    .dst = REG_BUS_ADDR(g->pwmRegs, PWM_FIF1),
	    .len = 4,
	    .next = NEXT(DMA_CB_PWM + 1)
	},
	.cb[DMA_CB_PWM + 1] =
	{
	    .src = MEM_BUS_ADDR(&g->vcMem, &g->dma->spiDLEN),
	    .dst = REG_BUS_ADDR(g->spiRegs, SPI_DLEN),
	    .len = 4,
	    .next = NEXT(DMA_CB_PWM + 2)
	},
	.cb[DMA_CB_PWM + 2] =
	{
	    .src = MEM_BUS_ADDR(&g->vcMem, &g->dma->spiCS),
	    .dst = REG_BUS_ADDR(g->spiRegs, SPI_CS),
	    .len = 4,
	    .next = NEXT(DMA_CB_PWM)
	},

#define DMA_CB_SPI_RX 3
	// spi rx channel - receives (and throws away) spi data
	.cb[DMA_CB_SPI_RX] =
	{
	    .ti = DMA_TI_SRC_DREQ | DMA_TI_PERMAP(DMA_SPI_RX_DREQ) |
	          DMA_TI_WAIT_RESP,
	    .src = REG_BUS_ADDR(g->spiRegs, SPI_FIFO),
	    .dst = MEM_BUS_ADDR(&g->vcMem, &g->dma->dummy),
	    .len = FRAME_DATA_SIZE,
	    .next = NEXT(DMA_CB_SPI_RX)
	},

#define DMA_CB_SPI_TX	4
	// spi tx channel
	// messages are transmitted every 11 frames
	// 10 cbs point to data containing only the sync pulse
	// the 10th cb points to one of three possible frames:
	//     - ping frame
	//     - pong frame
	//     - once frame
	// the ping and pong frames are used to send continuous messages.
	// a new message is written to the frame not in use and then the
	// 10th cb (link) is updated to point to the frame with the new message.
	//
	// to send a one-time message, the new message is written to the once
	// frame and the subsequent continuous message is written to the frame
	// not in use.  the onceLink variable is pointed to the new
	// continuous message frame and the 10th cb (link) is updated to point
	// to the once frame

#define _TX(_x, _data, _next)						      \
	.cb[(_x)] =							      \
	{								      \
	    .ti = DMA_TI_DEST_DREQ | DMA_TI_PERMAP(DMA_SPI_TX_DREQ) |	      \
	          DMA_TI_WAIT_RESP | DMA_TI_SRC_INC,			      \
	    .src = MEM_BUS_ADDR(&g->vcMem, g->dma->_data),		      \
	    .dst = REG_BUS_ADDR(g->spiRegs, SPI_FIFO),			      \
	    .len = FRAME_DATA_SIZE,					      \
	    .next = NEXT(_next)						      \
	}
#define TX(x) _TX(x, pulse, (x) + 1)

	// transmit message every 11 frames
	TX(0 + DMA_CB_SPI_TX), TX(1 + DMA_CB_SPI_TX),
	TX(2 + DMA_CB_SPI_TX), TX(3 + DMA_CB_SPI_TX),
	TX(4 + DMA_CB_SPI_TX), TX(5 + DMA_CB_SPI_TX),
	TX(6 + DMA_CB_SPI_TX), TX(7 + DMA_CB_SPI_TX),
	TX(8 + DMA_CB_SPI_TX),

#define DMA_CB_TX_LINK	(9 + DMA_CB_SPI_TX)
#define DMA_CB_TX_PING	(10 + DMA_CB_SPI_TX)
#define DMA_CB_TX_PONG	(11 + DMA_CB_SPI_TX)

	_TX(DMA_CB_TX_LINK, pulse, DMA_CB_TX_PING),
	_TX(DMA_CB_TX_PING, ping, DMA_CB_SPI_TX),
	_TX(DMA_CB_TX_PONG, pong, DMA_CB_SPI_TX),

#define DMA_CB_TX_ONCE	(12 + DMA_CB_SPI_TX)
	_TX(DMA_CB_TX_ONCE, once, DMA_CB_TX_ONCE + 1),

	// sets the link to the cb specified in onceLink
	.cb[DMA_CB_TX_ONCE + 1] =
	{
	    .src = MEM_BUS_ADDR(&g->vcMem, &g->dma->onceLink),
	    .dst = MEM_BUS_ADDR(&g->vcMem, &g->dma->cb[DMA_CB_TX_LINK].next),
	    .len = 4,
	    .next = NEXT(DMA_CB_SPI_TX)
	},

	.spiCS = SPI_CS_DMAEN | SPI_CS_TA | SPI_CS_RX_FIFO_CLR,
	.spiDLEN = FRAME_DATA_SIZE,
    };

    memcpy(g->dma, &data, sizeof(data));

    g->pingPong[0].buf = g->dma->ping;
    g->pingPong[0].link = NEXT(DMA_CB_TX_PING);
    g->pingPong[1].buf = g->dma->pong;
    g->pingPong[1].link = NEXT(DMA_CB_TX_PONG);

#define INIT_SYNC_PULSE(p)						      \
    shift = 7; setBitsUs(g->dma->p, &shift, 1, SYNC_MS * 1000)

    // init transmit buffers with sync pulse
    uint32_t shift;
    INIT_SYNC_PULSE(pulse);
    INIT_SYNC_PULSE(ping);
    INIT_SYNC_PULSE(pong);
    INIT_SYNC_PULSE(once);

    gpioSet(g, SPI0_MOSI_PIN, GPIO_ALT0, GPIO_NOPULL);

    // init spi
    *SPI(CLK) = (CLOCK_HZ / BIT_RATE) & ~1;
    *SPI(DC) = SPI_DC_TDREQ(1) | SPI_DC_TPANIC(8);
    *SPI(CS) = SPI_CS_FIFO_CLR;

    *PWM(CTL) = 0;

    if (*PWM(STA) & PWM_STA_BERR)
    {
        printf("PWM bus error\n");
        *PWM(STA) = PWM_STA_BERR;
    }

    // setup PWM clock
    *CLK(PWM_CTL) = CLK_PASSWD | CLK_CTL_KILL;
    while (*CLK(PWM_CTL) & CLK_CTL_BUSY);
    *CLK(PWM_DIV) = CLK_PASSWD | CLK_DIV_DIVI(PLLD_HZ / PWM_HZ);
    *CLK(PWM_CTL) = CLK_PASSWD | CLK_CTL_SRC_PLLD | CLK_CTL_ENAB;
    while (!(*CLK(PWM_CTL) & CLK_CTL_BUSY));

    usleep(100);
    *PWM(RNG1) = FRAME_MS * PWM_HZ / 1000;
    *PWM(DMAC) = PWM_DMAC_ENAB | PWM_DMAC_DREQ(1);

    startDma(g, &g->vcMem, DMA_CHANNEL_PWM, &g->dma->cb[DMA_CB_PWM]);
    startDma(g, &g->vcMem, DMA_CHANNEL_SPI_RX, &g->dma->cb[DMA_CB_SPI_RX]);
    startDma(g, &g->vcMem, DMA_CHANNEL_SPI_TX, &g->dma->cb[DMA_CB_SPI_TX]);

    // start pwm which triggers dma
    *PWM(CTL) = PWM_CTL_USEF1 | PWM_CTL_PWEN1;
}

static void
startNet(Globals *g, uint32_t port)
{
    struct sockaddr_in saddr =
        { .sin_port = htons(port), .sin_family = AF_INET,
          .sin_addr.s_addr = INADDR_ANY };

    g->net = socket(AF_INET, SOCK_DGRAM, 0);

    if (bind(g->net, (struct sockaddr *) &saddr, sizeof(saddr)))
    {
        printf("error binding.\n");
        exit(1);
    }
}

static void
startSerial(Globals *g)
{
    if ((g->serial = open("/dev/serial0", O_RDONLY | O_NOCTTY | O_NDELAY)) == -1)
	error("error opening serial port.\n");

    struct termios options;
    tcgetattr(g->serial, &options);

    options.c_cflag = B2400 | CS5 | CLOCAL | CREAD | PARENB;
    options.c_iflag = 0;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(g->serial, TCIFLUSH);
    tcsetattr(g->serial, TCSANOW, &options);
}

int
main(int argc, char **argv)
{
    Globals *g = &_globals;
    fd_set fdSet;
    struct sockaddr_in saddr = {};

    if (argc != 2)
    {
	printf("usage: %s <port>\n", argv[0]);
	exit(1);
    }

    signal(SIGINT, terminate);

    startSerial(g);
    startNet(g, atoi(argv[1]));
    startKeypad(g);

    FD_ZERO(&fdSet);
    FD_SET(g->serial, &fdSet);
    FD_SET(g->net, &fdSet);

    int maxFd = g->net > g->serial ? g->net : g->serial;

    while (1)
    {
	fd_set set = fdSet;

	if (select(maxFd + 1, &set,  NULL, NULL, NULL) < 1)
	    break;

	if (FD_ISSET(g->serial, &set))
	{
	    uint8_t buf[3];
	    int n = read(g->serial, buf, sizeof(buf));

	    if (saddr.sin_port && n == sizeof(buf))
	    {
		// all received bytes should be the same
		for (n = 1; n < sizeof(buf) && buf[n - 1] == buf[n]; n++);

		if (n == sizeof(buf))
		    sendto(g->net, buf, 1, 0, (struct sockaddr *) &saddr,
			   sizeof(saddr));
	    }
	}

	if (FD_ISSET(g->net, &set))
	{
	    socklen_t len = sizeof(saddr);
	    uint8_t buf[8];
	    ssize_t n = recvfrom(g->net, buf, sizeof(buf), 0,
				 (struct sockaddr *) &saddr, &len);

	    if (n == 4)
		PING_PONG(buf);
	    else if (n == 8)
		ONCE(buf, buf + 4);
	}
    }

    terminate(0);
}

static void
terminate(int sig)
{
    Globals *g = &_globals;

    if (g->pwmRegs.virt)	// disable pwm
	*PWM(CTL) = 0;

    if (g->spiRegs.virt)	// disable spi
    {
	*SPI(CS) = SPI_CS_FIFO_CLR;
	*SPI(CS) = 0;
    }

    if (g->dmaRegs.virt)	// stop dma
    {
	*DMA(DMA_CHANNEL_PWM, CS) = DMA_CS_RESET;
	*DMA(DMA_CHANNEL_SPI_RX, CS) = DMA_CS_RESET;
	*DMA(DMA_CHANNEL_SPI_TX, CS) = DMA_CS_RESET;
    }

    unmapPeriphMem(&g->spiRegs);
    unmapPeriphMem(&g->dmaRegs);
    unmapPeriphMem(&g->gpioRegs);
    unmapPeriphMem(&g->pwmRegs);
    unmapPeriphMem(&g->clkRegs);
    unmapPeriphMem(&g->vcMem);

    exit(0);
}
