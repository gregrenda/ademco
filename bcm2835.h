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

#ifndef _BCM2835_H_
#define _BCM2835_H_

#define CLOCK_HZ	250000000
#define PLLD_HZ		500000000

#define PHYS_REG_BASE	0x20000000
#define BUS_REG_BASE	0x7e000000

#define REG32(m, x)							      \
    ((volatile uint32_t *) ((uint32_t) (m.virt) + (uint32_t)(x)))

#define PAGE_SIZE	0x1000
#define PAGE_ROUNDUP(n)							      \
    (!((n) % PAGE_SIZE) ? (n) : ((n) + PAGE_SIZE) & ~(PAGE_SIZE - 1))

#define SPI0_CE0_PIN	8
#define SPI0_CE1_PIN	7
#define SPI0_MISO_PIN	9
#define SPI0_MOSI_PIN	10
#define SPI0_SCLK_PIN	11

#define SPI0_BASE	(PHYS_REG_BASE + 0x204000)
#define SPI_CS		0x00
#define SPI_FIFO	0x04
#define SPI_CLK		0x08
#define SPI_DLEN	0x0c
#define SPI_DC		0x14

#define SPI_CS_TX_FIFO_CLR	(1 << 4)
#define SPI_CS_RX_FIFO_CLR	(2 << 4)
#define SPI_CS_FIFO_CLR		(SPI_CS_TX_FIFO_CLR | SPI_CS_RX_FIFO_CLR)
#define SPI_CS_TA		(1 << 7)
#define SPI_CS_DMAEN		(1 << 8)

#define SPI_DC_TDREQ(x)		((x) << 0)
#define SPI_DC_TPANIC(x)	((x) << 8)

#define DMA_PWM_DREQ	5
#define DMA_SPI_TX_DREQ 6
#define DMA_SPI_RX_DREQ 7

#define DMA_BASE	(PHYS_REG_BASE + 0x7000)
// DMA register addresses offset by 0x100 * channel number
#define DMA_CS		0x00
#define DMA_CONBLK_AD	0x04
#define DMA_TI		0x08
#define DMA_SOURCE_AD	0x0c
#define DMA_DEST_AD	0x10
#define DMA_TXFR_LEN	0x14
#define DMA_STRIDE	0x18
#define DMA_NEXTCONBK	0x1c
#define DMA_DEBUG	0x20
#define DMA_REG(ch, r)	((ch) * 0x100 + (r))

#define DMA_CS_ACTIVE	(1 << 0)
#define DMA_CS_END	(1 << 1)
#define DMA_CS_RESET	(1 << 31)

#define DMA_DEBUG_READ_LAST_NOT_SET_ERROR	(1 << 0)
#define DMA_DEBUG_FIFO_ERROR			(1 << 1)
#define DMA_DEBUG_READ_ERROR			(1 << 2)

#define DMA_DEBUG_ERRORS	(DMA_DEBUG_READ_LAST_NOT_SET_ERROR |	      \
				 DMA_DEBUG_FIFO_ERROR | DMA_DEBUG_READ_ERROR)

#define DMA_TI_PERMAP(x)	((x) << 16)
#define DMA_TI_WAIT_RESP	(1 << 3)
#define DMA_TI_DST_INC		(1 << 4)
#define DMA_TI_DEST_DREQ	(1 << 6)
#define DMA_TI_DST_IGNORE	(1 << 7)
#define DMA_TI_SRC_INC		(1 << 8)
#define DMA_TI_SRC_DREQ		(1 << 10)
#define DMA_TI_SRC_IGNORE	(1 << 11)

#define DMA_CS_REG(ch)	REG32(dmaRegs, DMA_REG(ch, DMA_CS))

// DMA control block - 32-byte aligned
typedef struct
{
    uint32_t ti, src, dst, len, stride, next, debug, unused;
} DMA_CB __attribute__ ((aligned(32)));

#define GPIO_BASE	(PHYS_REG_BASE + 0x200000)
#define GPIO_FSEL0	0x00
#define GPIO_SET0	0x1c
#define GPIO_CLR0	0x28
#define GPIO_LEV0	0x34
#define GPIO_GPPUD	0x94
#define GPIO_GPPUDCLK0	0x98

#define GPIO_IN		0
#define GPIO_OUT	1
#define GPIO_ALT0	4
#define GPIO_ALT1	5
#define GPIO_ALT2	6
#define GPIO_ALT3	7
#define GPIO_ALT4	3
#define GPIO_ALT5	2
#define GPIO_NOPULL	0
#define GPIO_PULLDN	1
#define GPIO_PULLUP	2

#define PWM_BASE        (PHYS_REG_BASE + 0x20c000)
#define PWM_CTL         0x00   // Control
#define PWM_STA         0x04   // Status
#define PWM_DMAC        0x08   // DMA control
#define PWM_RNG1        0x10   // Channel 1 range
#define PWM_DAT1        0x14   // Channel 1 data
#define PWM_FIF1        0x18   // Channel 1 fifo
#define PWM_RNG2        0x20   // Channel 2 range
#define PWM_DAT2        0x24   // Channel 2 data

#define PWM_CTL_PWEN1		(1 << 0)
#define PWM_CTL_USEF1		(1 << 5)
#define PWM_CTL_MSEN1		(1 << 7)

#define PWM_STA_BERR		(1 << 8)

#define PWM_DMAC_DREQ(x)	((x) << 0)
#define PWM_DMAC_ENAB   	(1 << 31)

#define CLK_BASE        (PHYS_REG_BASE + 0x101000)
#define CLK_PWM_CTL     0xa0
#define CLK_PWM_DIV     0xa4

#define CLK_CTL_ENAB	(1 << 4)
#define CLK_CTL_KILL	(1 << 5)
#define CLK_CTL_BUSY	(1 << 7)

#define CLK_CTL_SRC(x)	((x) << 0)
#define CLK_CTL_SRC_PLLD	CLK_CTL_SRC(6)
#define CLK_DIV_DIVI(x)	((x) << 12)

#define CLK_PASSWD      0x5a000000

#endif // _BCM2835_H_
