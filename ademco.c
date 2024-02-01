/*
MIT License

Copyright (c) 2024 Greg Renda

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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/serdev.h>
#include <linux/of_device.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/jiffies.h>
#include <linux/circ_buf.h>
#include <linux/poll.h>

//#define DEBUG
//#define SOFT_UART		17		// rx GPIO

#define NAME			"ademco_panel"
#define FRAME_START_PULSE_MS	10
#define BAUD_RATE		4800

static dev_t ademco_dev;
static struct cdev ademco_cdev;
static struct class *ademco_class;
static struct gpio_desc *panel;
static struct task_struct *kthread;
static int edge;
static uint32_t address;
static bool startOfFrame;
#ifdef SOFT_UART
static bool uartStartOfFrame;
#endif // SOFT_UART
static uint8_t keyBuf[32],	// size must be a power of 2
    outBuf[256];		// size must be a power of 2
static struct circ_buf outCirc = { .buf = outBuf }, keyCirc = { .buf = keyBuf };

DECLARE_WAIT_QUEUE_HEAD(wqEdge);
DECLARE_WAIT_QUEUE_HEAD(wqData);
DEFINE_SPINLOCK(out_spinlock);

#define ERROR(_msg)							      \
do {									      \
    pr_err(NAME " - " _msg "\n");					      \
    return -1;								      \
} while (0)

#ifdef DEBUG
static void
hexdump(void *address, unsigned int length)
{
    char buf[100];
    void *start = address;

    while (length)
    {
        int i, n = length < 16 ? length : 16;
	char *p = buf;

	p += sprintf(p, "%08x ", address - start);

        for (i = 0; i < n; i++)
            p += sprintf(p, "%02x ", *(uint8_t *) ((uint8_t *) address + i));

        for (; i < 16; i++)
            p += sprintf(p, "   ");

        for (i = 0; i < n; i++)
        {
            uint8_t c = *(uint8_t *) ((uint8_t *) address + i);
            p += sprintf(p, "%c", c >= ' ' && c < 0x7f ? c : '.');
        }

        sprintf(p, "\n");
	printk(buf);
        length -= n;
        address = (uint8_t *) address + n;
    }
}
#endif // DEBUG

static ssize_t
ademco_read(struct file *filp, char __user *buf, size_t len, loff_t *off)
{
    size_t n = 0;
    bool err = true;

    if (!wait_event_interruptible(wqData, CIRC_CNT(outCirc.head, outCirc.tail,
						   sizeof(outBuf))))
    {
	int toEnd;

	spin_lock(&out_spinlock);

	n = CIRC_CNT(outCirc.head, outCirc.tail, sizeof(outBuf));

	if (n > len)
	    n = len;

	toEnd = CIRC_CNT_TO_END(outCirc.head, outCirc.tail, sizeof(outBuf));

	err = toEnd >= n ? copy_to_user(buf, outCirc.buf + outCirc.tail, n) :
	    copy_to_user(buf, outCirc.buf + outCirc.tail, toEnd) ||
	    copy_to_user(buf + toEnd, outCirc.buf, n - toEnd);

	outCirc.tail = (outCirc.tail + n) & (sizeof(outBuf) - 1);
	spin_unlock(&out_spinlock);
    }

    return err ? -EFAULT: n;
}

#define MAX_KEY_MSG (10 + 3)	// num keys + length, address and checksum

static ssize_t
ademco_write(struct file *filp, const char __user *buf, size_t len, loff_t *off)
{
    static uint8_t seq, want, keys[MAX_KEY_MSG], *p = keys;
    static unsigned long then;

    if (!len)
	return len;

    // timeout bytes wanted after two seconds
    if (want && jiffies_to_msecs((long) jiffies - (long) then) >= 2000)
    {
	want = 0;
	p = keys;
    }

    {
	uint32_t remaining = (sizeof(keys) - 1) - (p - keys),
	    n = want ? (len > want ? want : len) : len;

	if (n > remaining || copy_from_user(p, buf, n))
	    return -EFAULT;

	if (!want)
	{
	    want = keys[0];	// first byte is the length of the message
	    then = jiffies;	// set timeout
	}

	p += n;
	want -= n;
    }

    if (!want)			// got everything we were waiting for
    {
	uint32_t i, n = p - keys;
	uint8_t checksum = 0;

	p = keys;
	address = keys[1];

	if (address < 16 || address > 23) // make sure the address is in range
	    return -EFAULT;

	keys[0] = address + seq;	// address and sequence number
	keys[1] = n - 1;		// length

	for (i = 0; i < n; i++)		// calculate checksum
	    checksum += keys[i];

	keys[i] = -checksum;

	n++;				// address + length + keys + checksum

	if (CIRC_SPACE(keyCirc.head, keyCirc.tail, sizeof(keyBuf)) < n)
	    return -EFAULT;

	i = CIRC_SPACE_TO_END(keyCirc.head, keyCirc.tail, sizeof(keyBuf));

	if (i >= n)
	    memcpy(keyCirc.buf + keyCirc.head, keys, n);
	else
	{
	    memcpy(keyCirc.buf + keyCirc.head, keys, i);
	    memcpy(keyCirc.buf, keys + i, n - i);
	}

	keyCirc.head = (keyCirc.head + n) & (sizeof(keyBuf) - 1);
	seq += 0x40;		// increment sequence number
    }

    return len;
}

static unsigned int
ademco_poll(struct file *filp, struct poll_table_struct *wait)
{
    poll_wait(filp, &wqData, wait);
    return outCirc.head == outCirc.tail ? 0 : POLLIN | POLLRDNORM;
}

static struct of_device_id ademco_driver_ids[] =
{
    { .compatible = NAME }, {}
};

MODULE_DEVICE_TABLE(of, ademco_driver_ids);

static irqreturn_t
irqHandler(int irq, void *data)
{
    edge = gpiod_get_value(panel);
    wake_up_interruptible(&wqEdge);
    return IRQ_HANDLED;
}

static int
thread(void *data)
{
    struct serdev_device *serdev = data;
    int irqNum = gpiod_to_irq(panel);

    if (request_irq(irqNum, irqHandler, IRQF_TRIGGER_RISING |
		    IRQF_TRIGGER_FALLING, NAME, NULL))
	ERROR("Error requesting irq");

    while (!kthread_should_stop())
    {
	// wait for start of frame
	if (!wait_event_interruptible(wqEdge, edge))		// rising edge
	{
	    ktime_t start = ktime_get();

	    if (!wait_event_interruptible(wqEdge, !edge) &&	// falling edge
		// frame start
		ktime_ms_delta(ktime_get(), start) > FRAME_START_PULSE_MS)
	    {
		uint8_t buf[] = { 0xff, 0xff,
				  CIRC_CNT(keyCirc.head, keyCirc.tail,
					   sizeof(keyBuf)) ?
				  ~(1 << (address - 16)) :
				  0xff }, *p = buf;
		uint32_t i = sizeof(buf);

		serdev_device_set_parity(serdev, SERDEV_PARITY_NONE);
		startOfFrame = true;
#ifdef SOFT_UART
		uartStartOfFrame = true;
#endif // SOFT_UART

		// send query response data bytes after the falling edge of
		// the sync pulses
		do
		{
		    edge = 1;
		    serdev_device_write_buf(serdev, p++, 1);
		} while (--i && !wait_event_interruptible(wqEdge, !edge));
	    }
	}
    }

    free_irq(irqNum, NULL);
    return 0;
}

static int
serdev_recv(struct serdev_device *serdev, const unsigned char *buffer,
	    size_t size)
{
    static uint8_t buf[100], *p = buf, ackAddress;
    size_t n = size, remaining, consumed = 0, used = 0;
    bool output = false;

    if (startOfFrame)
    {
	while (n && !*buffer)	// strip leading 0s from start of frame
	{
	    buffer++;
	    n--;
	    consumed++;
	}

	p = buf;

	if (!n)			// used everything
	    return consumed;

	startOfFrame = false;
    }

    if (!(remaining = sizeof(buf) - (p - buf)))	// room remaining in buf
	return size;

    if (n > remaining)
	n = remaining;

    memcpy(p, buffer, n);
    p += n;
    consumed += n;

    n = p - buf;	// bytes available

    if (*buf == 0xf6)	// data query
    {
	if (n >= 3)
	{
	    uint32_t len = CIRC_CNT(keyCirc.head, keyCirc.tail, sizeof(keyBuf));

	    if (len && buf[1] == address) // send key data if address matches
	    {
		uint32_t toEnd =
		    CIRC_CNT_TO_END(keyCirc.head, keyCirc.tail, sizeof(keyBuf));

		// grab a single message - message length + 2 byte header
		len = keyCirc.buf[keyCirc.tail + 1] + 2;

		serdev_device_set_parity(serdev, SERDEV_PARITY_EVEN);

		if (toEnd >= len)
		    serdev_device_write_buf(serdev, keyCirc.buf + keyCirc.tail,
					    len);
		else
		{
		    uint8_t k[MAX_KEY_MSG];
		    memcpy(k, keyCirc.buf + keyCirc.tail, toEnd);
		    memcpy(k + toEnd, keyCirc.buf, len - toEnd);
		    serdev_device_write_buf(serdev, k, len);
		}
	    }

	    used = 3;
	    ackAddress = buf[1];
	}
    }
    else if (*buf == 0xf7)			// panel message
    {
	if (n >= 49)
	{
	    used = 49;
	    output = true;
	}
    }
    else if (*buf == 0xf2)			// panel additional info
    {
	if (n > 2 && n >= buf[1] + 3)
	{
	    used = buf[1] + 3;
	    output = true;
	}
    }
    else if (ackAddress && (*buf & 0x3f) == ackAddress)	// keypad ack message
    {
	if (n >= 2)
	{
	    uint32_t len = CIRC_CNT(keyCirc.head, keyCirc.tail, sizeof(keyBuf));

	    if (len && *buf == keyCirc.buf[keyCirc.tail])
		keyCirc.tail = (keyCirc.tail +
				keyCirc.buf[keyCirc.tail + 1] + 2) &
		    (sizeof(keyBuf) - 1);

	    ackAddress = 0;
	    used = 2;
	}
    }

    if (used)
    {
	if (output)	// make data available to the char device
	{
	    // if there's not enough room in the output buffer, overwrite the
	    // oldest data
	    spin_lock(&out_spinlock);

	    remaining = CIRC_SPACE(outCirc.head, outCirc.tail, sizeof(outBuf));

	    if (remaining < used)
		outCirc.tail =
		    (outCirc.tail + (used - remaining)) & (sizeof(outBuf) - 1);

	    spin_unlock(&out_spinlock);

	    remaining = CIRC_SPACE_TO_END(outCirc.head, outCirc.tail,
					  sizeof(outBuf));

	    if (remaining >= used)
		memcpy(outCirc.buf + outCirc.head, buf, used);
	    else
	    {
		memcpy(outCirc.buf + outCirc.head, buf, remaining);
		memcpy(outCirc.buf, buf + remaining, used - remaining);
	    }

	    outCirc.head = (outCirc.head + used) & (sizeof(outBuf) - 1);
	    wake_up_interruptible(&wqData);
	}

#ifdef DEBUG
	hexdump(buf, used);
#endif // DEBUG

	p = buf;

	// if there's unused data move it to the start of the buffer
	if (n > used)
	{
	    memmove(p, p + n, n - used);
	    p += n - used;
	}
    }

    return consumed;
}

static const struct serdev_device_ops serdev_ops =
{
    .receive_buf = serdev_recv
};

static int
serdev_probe(struct serdev_device *serdev)
{
    static struct file_operations fops =
    {
	.owner	= THIS_MODULE,
	.read	= ademco_read,
	.write	= ademco_write,
	.poll	= ademco_poll
    };

    struct device *dev = &serdev->dev;

    if (!device_property_present(dev, "panel-gpio"))
	ERROR("Device property 'panel-gpio' not found");

    if (IS_ERR(panel = devm_gpiod_get(dev, "panel", GPIOD_IN)))
	ERROR("Could not setup the GPIO");

    if (alloc_chrdev_region(&ademco_dev, 0, 1, NAME) < 0)
	ERROR("Cannot allocate major number");

    cdev_init(&ademco_cdev, &fops);

    if (cdev_add(&ademco_cdev, ademco_dev, 1) < 0)
    {
	unregister_chrdev_region(ademco_dev, 1);
	ERROR("Cannot add the device to the system");
    }

    if (IS_ERR(ademco_class = class_create(THIS_MODULE, NAME)))
    {
	cdev_del(&ademco_cdev);
	unregister_chrdev_region(ademco_dev, 1);
	ERROR("Cannot create the class");
    }

    if (IS_ERR(device_create(ademco_class, NULL, ademco_dev, NULL, NAME)))
    {
	class_destroy(ademco_class);
	cdev_del(&ademco_cdev);
	unregister_chrdev_region(ademco_dev, 1);
	ERROR("Cannot create the device");
    }

    serdev_device_set_client_ops(serdev, &serdev_ops);

    if (devm_serdev_device_open(dev, serdev))
    {
	device_destroy(ademco_class, ademco_dev);
	class_destroy(ademco_class);
	cdev_del(&ademco_cdev);
	unregister_chrdev_region(ademco_dev, 1);
	ERROR("Cannot open serdev");
    }

    serdev_device_set_baudrate(serdev, BAUD_RATE);
    serdev_device_set_flow_control(serdev, false);

    if (!(kthread = kthread_run(thread, (void *) serdev, NAME)))
    {
	device_destroy(ademco_class, ademco_dev);
	class_destroy(ademco_class);
	cdev_del(&ademco_cdev);
	unregister_chrdev_region(ademco_dev, 1);
	ERROR("Cannot run thread");
    }

    return 0;
}

static void
serdev_remove(struct serdev_device *serdev)
{
    kthread_stop(kthread);
    device_destroy(ademco_class, ademco_dev);
    class_destroy(ademco_class);
    cdev_del(&ademco_cdev);
    unregister_chrdev_region(ademco_dev, 1);
}

static struct serdev_device_driver ademco_driver =
{
    .probe = serdev_probe,
    .remove = serdev_remove,
    .driver =
    {
	.name = NAME,
	.of_match_table = ademco_driver_ids
    }
};

#ifdef SOFT_UART
#include <linux/gpio.h>
#include <linux/hrtimer.h>

#define UART_NAME	"ademco_devices"
#define BIT_RATE	(1000000000 / BAUD_RATE)
#define FRAME_MS	300

typedef enum { UART_SYNC, UART_DATA } uart_state_t;

static dev_t uart_dev;
static struct cdev uart_cdev;
static struct class *uart_class;
static int uart_irq_num;
static ktime_t uart_last_irq;
static struct hrtimer uart_timer;
static uint32_t uart_rx, uart_open_count, uart_bit_count, drop;
static uart_state_t uart_state;

static uint8_t uartOutBuf[256];		// size must be a power of 2
static struct circ_buf uartOutCirc = { .buf = uartOutBuf };

DECLARE_WAIT_QUEUE_HEAD(wqUartData);
DEFINE_SPINLOCK(uartOut_spinlock);

static ssize_t
uart_read(struct file *filp, char __user *buf, size_t len, loff_t *off)
{
    size_t n = 0;
    bool err = true;

    if (!wait_event_interruptible(wqUartData,
				  CIRC_CNT(uartOutCirc.head, uartOutCirc.tail,
					   sizeof(uartOutBuf))))
    {
	int toEnd;

	spin_lock(&uartOut_spinlock);

	n = CIRC_CNT(uartOutCirc.head, uartOutCirc.tail, sizeof(uartOutBuf));

	if (n > len)
	    n = len;

	toEnd = CIRC_CNT_TO_END(uartOutCirc.head, uartOutCirc.tail,
				sizeof(uartOutBuf));

	err = toEnd >= n ?
	    copy_to_user(buf, uartOutCirc.buf + uartOutCirc.tail, n) :
	    copy_to_user(buf, uartOutCirc.buf + uartOutCirc.tail, toEnd) ||
	    copy_to_user(buf + toEnd, uartOutCirc.buf, n - toEnd);

	uartOutCirc.tail = (uartOutCirc.tail + n) & (sizeof(uartOutBuf) - 1);
	spin_unlock(&uartOut_spinlock);
    }

    return err ? -EFAULT: n;
}

static unsigned int
uart_poll(struct file *filp, struct poll_table_struct *wait)
{
    poll_wait(filp, &wqUartData, wait);
    return uartOutCirc.head == uartOutCirc.tail ? 0 : POLLIN | POLLRDNORM;
}

static irqreturn_t
uart_irq_handler(int irq, void *data)
{
    switch (uart_state)
    {
	case UART_SYNC:
	{
	    if (!uartStartOfFrame)
		break;

	    uart_state = UART_DATA;
	}
	// fall through
	case UART_DATA:
	    if (!uart_bit_count)
	    {
		if (uartStartOfFrame)
		{
		    drop = 3;	// drop the 3 query response bytes
		    uartStartOfFrame = false;
		}

		uart_bit_count = 9;
		uart_rx = 0;
		hrtimer_start(&uart_timer, ktime_set(0, BIT_RATE / 2 + BIT_RATE),
			      HRTIMER_MODE_REL);
	    }
	    break;
    }

    return IRQ_HANDLED;
}

static enum hrtimer_restart
uart_timer_handler(struct hrtimer *timer)
{
    uart_rx = (uart_rx >> 1) | (gpio_get_value(SOFT_UART) << 8);

    if (--uart_bit_count)
    {
	hrtimer_forward_now(timer, BIT_RATE);
	return HRTIMER_RESTART;
    }

    if (!drop)
    {
	// if there's not enough room in the output buffer, overwrite the
	// oldest data
	spin_lock(&uartOut_spinlock);

	if (!CIRC_SPACE(uartOutCirc.head, uartOutCirc.tail, sizeof(uartOutBuf)))
	    uartOutCirc.tail = (uartOutCirc.tail + 1) & (sizeof(uartOutBuf) - 1);

	spin_unlock(&uartOut_spinlock);

	uartOutCirc.buf[uartOutCirc.head] = uart_rx;
	uartOutCirc.head = (uartOutCirc.head + 1) & (sizeof(uartOutBuf) - 1);
	wake_up_interruptible(&wqUartData);
    }
    else
	drop--;

    return HRTIMER_NORESTART;
}

static int
uart_open(struct inode *inode, struct file *file)
{
    if (!uart_open_count)
    {
	if (gpio_request(SOFT_UART, UART_NAME))
	    ERROR("Cannot request uart gpio");

	if (gpio_direction_input(SOFT_UART))
	{
	    gpio_free(SOFT_UART);
	    ERROR("Cannot set uart gpio direction");
	}

	uart_state = UART_SYNC;
	uart_last_irq = ktime_get();

	if (request_irq(uart_irq_num, uart_irq_handler,
			IRQF_TRIGGER_FALLING, "uart_irq_handler", NULL))
	{
	    gpio_free(SOFT_UART);
	    ERROR("Cannot request uart irq");
	}

	hrtimer_init(&uart_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	uart_timer.function = &uart_timer_handler;

	uart_open_count++;
    }

    return 0;
}

static int
uart_release(struct inode *inode, struct file *file)
{
    if (uart_open_count && !--uart_open_count)
    {
	hrtimer_cancel(&uart_timer);
	disable_irq(uart_irq_num);
	free_irq(uart_irq_num, NULL);
	gpio_free(SOFT_UART);
    }

    return 0;
}

static int
soft_uart_init(void)
{
    static struct file_operations fops =
    {
	.owner		= THIS_MODULE,
	.read		= uart_read,
	.poll		= uart_poll,
	.open		= uart_open,
	.release	= uart_release
    };

    if ((uart_irq_num = gpio_to_irq(SOFT_UART)) < 0)
	ERROR("Cannot get uart irq");

    if (alloc_chrdev_region(&uart_dev, 0, 1, UART_NAME) < 0)
	ERROR("Cannot allocate uart major number");

    cdev_init(&uart_cdev, &fops);

    if (cdev_add(&uart_cdev, uart_dev, 1) < 0)
    {
	unregister_chrdev_region(uart_dev, 1);
	ERROR("Cannot add the uart device to the system");
    }

    if (IS_ERR(uart_class = class_create(THIS_MODULE, UART_NAME)))
    {
	cdev_del(&uart_cdev);
	unregister_chrdev_region(uart_dev, 1);
	ERROR("Cannot create the uart class");
    }

    if (IS_ERR(device_create(uart_class, NULL, uart_dev, NULL, UART_NAME)))
    {
	class_destroy(uart_class);
	cdev_del(&uart_cdev);
	unregister_chrdev_region(uart_dev, 1);
	ERROR("Cannot create the uart device");
    }

    return 0;
}
#endif // SOFT_UART

static int __init
ademco_init(void)
{
    if (serdev_device_driver_register(&ademco_driver))
	ERROR("Could not load driver");

#ifdef SOFT_UART
    if (soft_uart_init())
    {
	serdev_device_driver_unregister(&ademco_driver);
	ERROR("Could not init soft uart");
    }
#endif // SOFT_UART

    return 0;
}

static void __exit
ademco_exit(void)
{
    serdev_device_driver_unregister(&ademco_driver);

#ifdef SOFT_UART
    device_destroy(uart_class, uart_dev);
    class_destroy(uart_class);
    cdev_del(&uart_cdev);
    unregister_chrdev_region(uart_dev, 1);
#endif // SOFT_UART
}

module_init(ademco_init);
module_exit(ademco_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Greg Renda");
MODULE_DESCRIPTION("Ademco alarm panel interface");
MODULE_VERSION("1.0");
