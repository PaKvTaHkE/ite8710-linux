// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Driver for ITE Tech Inc. IT8712F/IT8512 CIR
 *
 * Out-of-tree variant: platform/ACPI driver for the ITE8710 (IT8305) CIR
 * found on HP laptops, with IT8512 CIR registers mapped directly (16 bytes).
 *
 * Copyright (C) 2010 Juan Jesús García de Soria <skandalfo@gmail.com>
 *
 * Inspired by the original lirc_it87 and lirc_ite8709 drivers, on top of the
 * skeleton provided by the nuvoton-cir driver.
 *
 * The lirc_it87 driver was originally written by Hans-Gunter Lutke Uphues
 * <hg_lu@web.de> in 2001, with enhancements by Christoph Bartelmus
 * <lirc@bartelmus.de>, Andrew Calkin <r_tay@hotmail.com> and James Edwards
 * <jimbo-lirc@edwardsclan.net>.
 *
 * The lirc_ite8709 driver was written by Grégory Lardière
 * <spmf2004-lirc@yahoo.fr> in 2008.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/acpi.h>
#include <linux/mod_devicetable.h>
#include <linux/pm.h>
#include <linux/timer.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/bitops.h>
#include <media/rc-core.h>
#include <linux/pci_ids.h>

#include "ite-cir.h"

/* module parameters */

/* default sample period */
static long sample_period = NSEC_PER_SEC / 115200;
module_param(sample_period, long, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(sample_period, "sample period");

/* override detected model id */
static int model_number = -1;
module_param(model_number, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(model_number, "Use this model number, don't autodetect");

/* poll the RX FIFO from a timer instead of using the IRQ */
static bool poll;
module_param(poll, bool, S_IRUGO);
MODULE_PARM_DESC(poll, "Poll the RX FIFO instead of using the IRQ");

/* poll interval in ms */
static unsigned int poll_ms = 2;
module_param(poll_ms, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(poll_ms, "Poll interval in ms (poll=1 only)");


/* HW-independent code functions */

/* check whether carrier frequency is high frequency */
static inline bool ite_is_high_carrier_freq(unsigned int freq)
{
	return freq >= ITE_HCF_MIN_CARRIER_FREQ;
}

/* get the bits required to program the carrier frequency in CFQ bits,
 * unshifted */
static u8 ite_get_carrier_freq_bits(unsigned int freq)
{
	if (ite_is_high_carrier_freq(freq)) {
		if (freq < 425000)
			return ITE_CFQ_400;

		else if (freq < 465000)
			return ITE_CFQ_450;

		else if (freq < 490000)
			return ITE_CFQ_480;

		else
			return ITE_CFQ_500;
	} else {
			/* trim to limits */
		if (freq < ITE_LCF_MIN_CARRIER_FREQ)
			freq = ITE_LCF_MIN_CARRIER_FREQ;
		if (freq > ITE_LCF_MAX_CARRIER_FREQ)
			freq = ITE_LCF_MAX_CARRIER_FREQ;

		/* convert to kHz and subtract the base freq */
		freq = DIV_ROUND_CLOSEST(freq - ITE_LCF_MIN_CARRIER_FREQ, 1000);

		return (u8) freq;
	}
}

/* get the bits required to program the pulse with in TXMPW */
static u8 ite_get_pulse_width_bits(unsigned int freq, int duty_cycle)
{
	unsigned long period_ns, on_ns;

	/* sanitize freq into range */
	if (freq < ITE_LCF_MIN_CARRIER_FREQ)
		freq = ITE_LCF_MIN_CARRIER_FREQ;
	if (freq > ITE_HCF_MAX_CARRIER_FREQ)
		freq = ITE_HCF_MAX_CARRIER_FREQ;

	period_ns = 1000000000UL / freq;
	on_ns = period_ns * duty_cycle / 100;

	if (ite_is_high_carrier_freq(freq)) {
		if (on_ns < 750)
			return ITE_TXMPW_A;

		else if (on_ns < 850)
			return ITE_TXMPW_B;

		else if (on_ns < 950)
			return ITE_TXMPW_C;

		else if (on_ns < 1080)
			return ITE_TXMPW_D;

		else
			return ITE_TXMPW_E;
	} else {
		if (on_ns < 6500)
			return ITE_TXMPW_A;

		else if (on_ns < 7850)
			return ITE_TXMPW_B;

		else if (on_ns < 9650)
			return ITE_TXMPW_C;

		else if (on_ns < 11950)
			return ITE_TXMPW_D;

		else
			return ITE_TXMPW_E;
	}
}

/* decode raw bytes as received by the hardware, and push them to the ir-core
 * layer */
static void ite_decode_bytes(struct ite_dev *dev, const u8 * data, int
			     length)
{
	unsigned long *ldata;
	unsigned int next_one, next_zero, size;
	struct ir_raw_event ev = {};

	if (length == 0)
		return;

	ldata = (unsigned long *)data;
	size = length << 3;
	next_one = find_next_bit_le(ldata, size, 0);
	if (next_one > 0) {
		ev.pulse = true;
		ev.duration = ITE_BITS_TO_US(next_one, sample_period);
		ir_raw_event_store_with_filter(dev->rdev, &ev);
	}

	while (next_one < size) {
		next_zero = find_next_zero_bit_le(ldata, size, next_one + 1);
		ev.pulse = false;
		ev.duration = ITE_BITS_TO_US(next_zero - next_one, sample_period);
		ir_raw_event_store_with_filter(dev->rdev, &ev);

		if (next_zero < size) {
			next_one = find_next_bit_le(ldata, size, next_zero + 1);
			ev.pulse = true;
			ev.duration = ITE_BITS_TO_US(next_one - next_zero,
						     sample_period);
			ir_raw_event_store_with_filter(dev->rdev, &ev);
		} else
			next_one = size;
	}

	ir_raw_event_handle(dev->rdev);

	dev_dbg(&dev->rdev->dev, "decoded %d bytes\n", length);
}

/* set all the rx/tx carrier parameters; this must be called with the device
 * spinlock held */
static void ite_set_carrier_params(struct ite_dev *dev)
{
	unsigned int freq, low_freq, high_freq;
	int allowance;
	bool use_demodulator;
	bool for_tx = dev->transmitting;

	if (for_tx) {
		/* we don't need no stinking calculations */
		freq = dev->tx_carrier_freq;
		allowance = ITE_RXDCR_DEFAULT;
		use_demodulator = false;
	} else {
		low_freq = dev->rx_low_carrier_freq;
		high_freq = dev->rx_high_carrier_freq;

		if (low_freq == 0) {
			/* don't demodulate */
			freq = ITE_DEFAULT_CARRIER_FREQ;
			allowance = ITE_RXDCR_DEFAULT;
			use_demodulator = false;
		} else {
			/* calculate the middle freq */
			freq = (low_freq + high_freq) / 2;

			/* calculate the allowance */
			allowance =
			    DIV_ROUND_CLOSEST(10000 * (high_freq - low_freq),
					      ITE_RXDCR_PER_10000_STEP
					      * (high_freq + low_freq));

			if (allowance < 1)
				allowance = 1;

			if (allowance > ITE_RXDCR_MAX)
				allowance = ITE_RXDCR_MAX;

			use_demodulator = true;
		}
	}

	/* set the carrier parameters in a device-dependent way */
	dev->params->set_carrier_params(dev, ite_is_high_carrier_freq(freq),
		 use_demodulator, ite_get_carrier_freq_bits(freq), allowance,
		 ite_get_pulse_width_bits(freq, dev->tx_duty_cycle));
}

/* interrupt service routine for incoming and outgoing CIR data */
static irqreturn_t ite_cir_isr(int irq, void *data)
{
	struct ite_dev *dev = data;
	irqreturn_t ret = IRQ_RETVAL(IRQ_NONE);
	u8 rx_buf[ITE_RX_FIFO_LEN];
	int rx_bytes;
	int iflags;

	/* grab the spinlock */
	spin_lock(&dev->lock);

	/* read the interrupt flags */
	iflags = dev->params->get_irq_causes(dev);

	/* Check for RX overflow */
	if (iflags & ITE_IRQ_RX_FIFO_OVERRUN) {
		dev_warn(&dev->rdev->dev, "receive overflow\n");
		ir_raw_event_overflow(dev->rdev);
	}

	/* check for the receive interrupt */
	if (iflags & (ITE_IRQ_RX_FIFO | ITE_IRQ_RX_FIFO_OVERRUN)) {
		/* read the FIFO bytes */
		rx_bytes = dev->params->get_rx_bytes(dev, rx_buf,
						    ITE_RX_FIFO_LEN);

		dev_dbg(&dev->rdev->dev, "interrupt %d RX bytes\n", rx_bytes);

		if (rx_bytes > 0) {
			/* drop the spinlock, since the ir-core layer
			 * may call us back again through
			 * ite_s_idle() */
			spin_unlock(&dev->lock);

			/* decode the data we've just received */
			ite_decode_bytes(dev, rx_buf, rx_bytes);

			/* reacquire the spinlock */
			spin_lock(&dev->lock);

			/* mark the interrupt as serviced */
			ret = IRQ_RETVAL(IRQ_HANDLED);
		}
	} else if (iflags & ITE_IRQ_TX_FIFO) {
		/* FIFO space available interrupt */
		dev_dbg(&dev->rdev->dev, "interrupt TX FIFO\n");

		/* wake any sleeping transmitter */
		wake_up_interruptible(&dev->tx_queue);

		/* mark the interrupt as serviced */
		ret = IRQ_RETVAL(IRQ_HANDLED);
	}

	/* drop the spinlock */
	spin_unlock(&dev->lock);

	return ret;
}

/* set the rx carrier freq range, guess it's in Hz... */
static int ite_set_rx_carrier_range(struct rc_dev *rcdev, u32 carrier_low, u32
				    carrier_high)
{
	unsigned long flags;
	struct ite_dev *dev = rcdev->priv;

	spin_lock_irqsave(&dev->lock, flags);
	dev->rx_low_carrier_freq = carrier_low;
	dev->rx_high_carrier_freq = carrier_high;
	ite_set_carrier_params(dev);
	spin_unlock_irqrestore(&dev->lock, flags);

	return 0;
}

/* set the tx carrier freq, guess it's in Hz... */
static int ite_set_tx_carrier(struct rc_dev *rcdev, u32 carrier)
{
	unsigned long flags;
	struct ite_dev *dev = rcdev->priv;

	spin_lock_irqsave(&dev->lock, flags);
	dev->tx_carrier_freq = carrier;
	ite_set_carrier_params(dev);
	spin_unlock_irqrestore(&dev->lock, flags);

	return 0;
}

/* set the tx duty cycle by controlling the pulse width */
static int ite_set_tx_duty_cycle(struct rc_dev *rcdev, u32 duty_cycle)
{
	unsigned long flags;
	struct ite_dev *dev = rcdev->priv;

	spin_lock_irqsave(&dev->lock, flags);
	dev->tx_duty_cycle = duty_cycle;
	ite_set_carrier_params(dev);
	spin_unlock_irqrestore(&dev->lock, flags);

	return 0;
}

/* transmit out IR pulses; what you get here is a batch of alternating
 * pulse/space/pulse/space lengths that we should write out completely through
 * the FIFO, blocking on a full FIFO */
static int ite_tx_ir(struct rc_dev *rcdev, unsigned *txbuf, unsigned n)
{
	unsigned long flags;
	struct ite_dev *dev = rcdev->priv;
	bool is_pulse = false;
	int remaining_us, fifo_avail, fifo_remaining, last_idx = 0;
	int max_rle_us, next_rle_us;
	int ret = n;
	u8 last_sent[ITE_TX_FIFO_LEN];
	u8 val;

	/* clear the array just in case */
	memset(last_sent, 0, sizeof(last_sent));

	spin_lock_irqsave(&dev->lock, flags);

	/* let everybody know we're now transmitting */
	dev->transmitting = true;

	/* and set the carrier values for transmission */
	ite_set_carrier_params(dev);

	/* calculate how much time we can send in one byte */
	max_rle_us =
	    (ITE_BAUDRATE_DIVISOR * sample_period *
	     ITE_TX_MAX_RLE) / 1000;

	/* disable the receiver */
	dev->params->disable_rx(dev);

	/* this is where we'll begin filling in the FIFO, until it's full.
	 * then we'll just activate the interrupt, wait for it to wake us up
	 * again, disable it, continue filling the FIFO... until everything
	 * has been pushed out */
	fifo_avail = ITE_TX_FIFO_LEN - dev->params->get_tx_used_slots(dev);

	while (n > 0) {
		/* transmit the next sample */
		is_pulse = !is_pulse;
		remaining_us = *(txbuf++);
		n--;

		dev_dbg(&dev->rdev->dev, "%s: %d\n",
			is_pulse ? "pulse" : "space", remaining_us);

		/* repeat while the pulse is non-zero length */
		while (remaining_us > 0) {
			if (remaining_us > max_rle_us)
				next_rle_us = max_rle_us;

			else
				next_rle_us = remaining_us;

			remaining_us -= next_rle_us;

			/* check what's the length we have to pump out */
			val = (ITE_TX_MAX_RLE * next_rle_us) / max_rle_us;

			/* put it into the sent buffer */
			last_sent[last_idx++] = val;
			last_idx &= (ITE_TX_FIFO_LEN);

			/* encode it for 7 bits */
			val = (val - 1) & ITE_TX_RLE_MASK;

			/* take into account pulse/space prefix */
			if (is_pulse)
				val |= ITE_TX_PULSE;

			else
				val |= ITE_TX_SPACE;

			/*
			 * if we get to 0 available, read again, just in case
			 * some other slot got freed
			 */
			if (fifo_avail <= 0)
				fifo_avail = ITE_TX_FIFO_LEN - dev->params->get_tx_used_slots(dev);

			/* if it's still full */
			if (fifo_avail <= 0) {
				/* enable the tx interrupt */
				dev->params->enable_tx_interrupt(dev);

				/* drop the spinlock */
				spin_unlock_irqrestore(&dev->lock, flags);

				/* wait for the FIFO to empty enough */
				wait_event_interruptible(dev->tx_queue,
					(fifo_avail = ITE_TX_FIFO_LEN - dev->params->get_tx_used_slots(dev)) >= 8);

				/* get the spinlock again */
				spin_lock_irqsave(&dev->lock, flags);

				/* disable the tx interrupt again. */
				dev->params->disable_tx_interrupt(dev);
			}

			/* now send the byte through the FIFO */
			dev->params->put_tx_byte(dev, val);
			fifo_avail--;
		}
	}

	/* wait and don't return until the whole FIFO has been sent out;
	 * otherwise we could configure the RX carrier params instead of the
	 * TX ones while the transmission is still being performed! */
	fifo_remaining = dev->params->get_tx_used_slots(dev);
	remaining_us = 0;
	while (fifo_remaining > 0) {
		fifo_remaining--;
		last_idx--;
		last_idx &= (ITE_TX_FIFO_LEN - 1);
		remaining_us += last_sent[last_idx];
	}
	remaining_us = (remaining_us * max_rle_us) / (ITE_TX_MAX_RLE);

	/* drop the spinlock while we sleep */
	spin_unlock_irqrestore(&dev->lock, flags);

	/* sleep remaining_us microseconds */
	mdelay(DIV_ROUND_UP(remaining_us, 1000));

	/* reacquire the spinlock */
	spin_lock_irqsave(&dev->lock, flags);

	/* now we're not transmitting anymore */
	dev->transmitting = false;

	/* and set the carrier values for reception */
	ite_set_carrier_params(dev);

	/* re-enable the receiver */
	dev->params->enable_rx(dev);

	/* notify transmission end */
	wake_up_interruptible(&dev->tx_ended);

	spin_unlock_irqrestore(&dev->lock, flags);

	return ret;
}

/* idle the receiver if needed */
static void ite_s_idle(struct rc_dev *rcdev, bool enable)
{
	unsigned long flags;
	struct ite_dev *dev = rcdev->priv;

	if (enable) {
		spin_lock_irqsave(&dev->lock, flags);
		dev->params->idle_rx(dev);
		spin_unlock_irqrestore(&dev->lock, flags);
	}
}


/* IT8712F HW-specific functions */

/* retrieve a bitmask of the current causes for a pending interrupt; this may
 * be composed of ITE_IRQ_TX_FIFO, ITE_IRQ_RX_FIFO and ITE_IRQ_RX_FIFO_OVERRUN
 * */
static int it87_get_irq_causes(struct ite_dev *dev)
{
	u8 iflags;
	int ret = 0;

	/* read the interrupt flags */
	iflags = inb(dev->cir_addr + IT87_IIR) & IT87_II;

	switch (iflags) {
	case IT87_II_RXDS:
		ret = ITE_IRQ_RX_FIFO;
		break;
	case IT87_II_RXFO:
		ret = ITE_IRQ_RX_FIFO_OVERRUN;
		break;
	case IT87_II_TXLDL:
		ret = ITE_IRQ_TX_FIFO;
		break;
	}

	return ret;
}

/* set the carrier parameters; to be called with the spinlock held */
static void it87_set_carrier_params(struct ite_dev *dev, bool high_freq,
				    bool use_demodulator,
				    u8 carrier_freq_bits, u8 allowance_bits,
				    u8 pulse_width_bits)
{
	u8 val;

	/* program the RCR register */
	val = inb(dev->cir_addr + IT87_RCR)
		& ~(IT87_HCFS | IT87_RXEND | IT87_RXDCR);

	if (high_freq)
		val |= IT87_HCFS;

	if (use_demodulator)
		val |= IT87_RXEND;

	val |= allowance_bits;

	outb(val, dev->cir_addr + IT87_RCR);

	/* program the TCR2 register */
	outb((carrier_freq_bits << IT87_CFQ_SHIFT) | pulse_width_bits,
		dev->cir_addr + IT87_TCR2);
}

/* read up to buf_size bytes from the RX FIFO; to be called with the spinlock
 * held */
static int it87_get_rx_bytes(struct ite_dev *dev, u8 * buf, int buf_size)
{
	int fifo, read = 0;

	/* read how many bytes are still in the FIFO */
	fifo = inb(dev->cir_addr + IT87_RSR) & IT87_RXFBC;

	while (fifo > 0 && buf_size > 0) {
		*(buf++) = inb(dev->cir_addr + IT87_DR);
		fifo--;
		read++;
		buf_size--;
	}

	return read;
}

/* return how many bytes are still in the FIFO; this will be called
 * with the device spinlock NOT HELD while waiting for the TX FIFO to get
 * empty; let's expect this won't be a problem */
static int it87_get_tx_used_slots(struct ite_dev *dev)
{
	return inb(dev->cir_addr + IT87_TSR) & IT87_TXFBC;
}

/* put a byte to the TX fifo; this should be called with the spinlock held */
static void it87_put_tx_byte(struct ite_dev *dev, u8 value)
{
	outb(value, dev->cir_addr + IT87_DR);
}

/* idle the receiver so that we won't receive samples until another
  pulse is detected; this must be called with the device spinlock held */
static void it87_idle_rx(struct ite_dev *dev)
{
	/* disable streaming by clearing RXACT writing it as 1 */
	outb(inb(dev->cir_addr + IT87_RCR) | IT87_RXACT,
		dev->cir_addr + IT87_RCR);

	/* clear the FIFO */
	outb(inb(dev->cir_addr + IT87_TCR1) | IT87_FIFOCLR,
		dev->cir_addr + IT87_TCR1);
}

/* disable the receiver; this must be called with the device spinlock held */
static void it87_disable_rx(struct ite_dev *dev)
{
	/* disable the receiver interrupts */
	outb(inb(dev->cir_addr + IT87_IER) & ~(IT87_RDAIE | IT87_RFOIE),
		dev->cir_addr + IT87_IER);

	/* disable the receiver */
	outb(inb(dev->cir_addr + IT87_RCR) & ~IT87_RXEN,
		dev->cir_addr + IT87_RCR);

	/* clear the FIFO and RXACT (actually RXACT should have been cleared
	* in the previous outb() call) */
	it87_idle_rx(dev);
}

/* enable the receiver; this must be called with the device spinlock held */
static void it87_enable_rx(struct ite_dev *dev)
{
	/* enable the receiver by setting RXEN */
	outb(inb(dev->cir_addr + IT87_RCR) | IT87_RXEN,
		dev->cir_addr + IT87_RCR);

	/* just prepare it to idle for the next reception */
	it87_idle_rx(dev);

	/* enable the receiver interrupts and master enable flag */
	outb(inb(dev->cir_addr + IT87_IER) | IT87_RDAIE | IT87_RFOIE | IT87_IEC,
		dev->cir_addr + IT87_IER);
}

/* disable the transmitter interrupt; this must be called with the device
 * spinlock held */
static void it87_disable_tx_interrupt(struct ite_dev *dev)
{
	/* disable the transmitter interrupts */
	outb(inb(dev->cir_addr + IT87_IER) & ~IT87_TLDLIE,
		dev->cir_addr + IT87_IER);
}

/* enable the transmitter interrupt; this must be called with the device
 * spinlock held */
static void it87_enable_tx_interrupt(struct ite_dev *dev)
{
	/* enable the transmitter interrupts and master enable flag */
	outb(inb(dev->cir_addr + IT87_IER) | IT87_TLDLIE | IT87_IEC,
		dev->cir_addr + IT87_IER);
}

/* disable the device; this must be called with the device spinlock held */
static void it87_disable(struct ite_dev *dev)
{
	/* clear out all interrupt enable flags */
	outb(inb(dev->cir_addr + IT87_IER) &
		~(IT87_IEC | IT87_RFOIE | IT87_RDAIE | IT87_TLDLIE),
		dev->cir_addr + IT87_IER);

	/* disable the receiver */
	it87_disable_rx(dev);

	/* erase the FIFO */
	outb(IT87_FIFOCLR | inb(dev->cir_addr + IT87_TCR1),
		dev->cir_addr + IT87_TCR1);
}

/* initialize the hardware */
static void it87_init_hardware(struct ite_dev *dev)
{
	/* enable just the baud rate divisor register,
	disabling all the interrupts at the same time */
	outb((inb(dev->cir_addr + IT87_IER) &
		~(IT87_IEC | IT87_RFOIE | IT87_RDAIE | IT87_TLDLIE)) | IT87_BR,
		dev->cir_addr + IT87_IER);

	/* write out the baud rate divisor */
	outb(ITE_BAUDRATE_DIVISOR & 0xff, dev->cir_addr + IT87_BDLR);
	outb((ITE_BAUDRATE_DIVISOR >> 8) & 0xff, dev->cir_addr + IT87_BDHR);

	/* disable the baud rate divisor register again */
	outb(inb(dev->cir_addr + IT87_IER) & ~IT87_BR,
		dev->cir_addr + IT87_IER);

	/* program the RCR register defaults */
	outb(ITE_RXDCR_DEFAULT, dev->cir_addr + IT87_RCR);

	/* program the TCR1 register */
	outb(IT87_TXMPM_DEFAULT | IT87_TXENDF | IT87_TXRLE
		| IT87_FIFOTL_DEFAULT | IT87_FIFOCLR,
		dev->cir_addr + IT87_TCR1);

	/* program the carrier parameters */
	ite_set_carrier_params(dev);
}

/* IT8512F on ITE8708 HW-specific functions */

/* retrieve a bitmask of the current causes for a pending interrupt; this may
 * be composed of ITE_IRQ_TX_FIFO, ITE_IRQ_RX_FIFO and ITE_IRQ_RX_FIFO_OVERRUN
 * */
static int it8708_get_irq_causes(struct ite_dev *dev)
{
	u8 iflags;
	int ret = 0;

	/* read the interrupt flags */
	iflags = inb(dev->cir_addr + IT8708_C0IIR);

	if (iflags & IT85_TLDLI)
		ret |= ITE_IRQ_TX_FIFO;
	if (iflags & IT85_RDAI)
		ret |= ITE_IRQ_RX_FIFO;
	if (iflags & IT85_RFOI)
		ret |= ITE_IRQ_RX_FIFO_OVERRUN;

	return ret;
}

/* set the carrier parameters; to be called with the spinlock held */
static void it8708_set_carrier_params(struct ite_dev *dev, bool high_freq,
				      bool use_demodulator,
				      u8 carrier_freq_bits, u8 allowance_bits,
				      u8 pulse_width_bits)
{
	u8 val;

	/* program the C0CFR register, with HRAE=1 */
	outb(inb(dev->cir_addr + IT8708_BANKSEL) | IT8708_HRAE,
		dev->cir_addr + IT8708_BANKSEL);

	val = (inb(dev->cir_addr + IT8708_C0CFR)
		& ~(IT85_HCFS | IT85_CFQ)) | carrier_freq_bits;

	if (high_freq)
		val |= IT85_HCFS;

	outb(val, dev->cir_addr + IT8708_C0CFR);

	outb(inb(dev->cir_addr + IT8708_BANKSEL) & ~IT8708_HRAE,
		   dev->cir_addr + IT8708_BANKSEL);

	/* program the C0RCR register */
	val = inb(dev->cir_addr + IT8708_C0RCR)
		& ~(IT85_RXEND | IT85_RXDCR);

	if (use_demodulator)
		val |= IT85_RXEND;

	val |= allowance_bits;

	outb(val, dev->cir_addr + IT8708_C0RCR);

	/* program the C0TCR register */
	val = inb(dev->cir_addr + IT8708_C0TCR) & ~IT85_TXMPW;
	val |= pulse_width_bits;
	outb(val, dev->cir_addr + IT8708_C0TCR);
}

/* read up to buf_size bytes from the RX FIFO; to be called with the spinlock
 * held */
static int it8708_get_rx_bytes(struct ite_dev *dev, u8 * buf, int buf_size)
{
	int fifo, read = 0;

	/* read how many bytes are still in the FIFO */
	fifo = inb(dev->cir_addr + IT8708_C0RFSR) & IT85_RXFBC;

	while (fifo > 0 && buf_size > 0) {
		*(buf++) = inb(dev->cir_addr + IT8708_C0DR);
		fifo--;
		read++;
		buf_size--;
	}

	return read;
}

/* return how many bytes are still in the FIFO; this will be called
 * with the device spinlock NOT HELD while waiting for the TX FIFO to get
 * empty; let's expect this won't be a problem */
static int it8708_get_tx_used_slots(struct ite_dev *dev)
{
	return inb(dev->cir_addr + IT8708_C0TFSR) & IT85_TXFBC;
}

/* put a byte to the TX fifo; this should be called with the spinlock held */
static void it8708_put_tx_byte(struct ite_dev *dev, u8 value)
{
	outb(value, dev->cir_addr + IT8708_C0DR);
}

/* idle the receiver so that we won't receive samples until another
  pulse is detected; this must be called with the device spinlock held */
static void it8708_idle_rx(struct ite_dev *dev)
{
	/* disable streaming by clearing RXACT writing it as 1 */
	outb(inb(dev->cir_addr + IT8708_C0RCR) | IT85_RXACT,
		dev->cir_addr + IT8708_C0RCR);

	/* clear the FIFO */
	outb(inb(dev->cir_addr + IT8708_C0MSTCR) | IT85_FIFOCLR,
		dev->cir_addr + IT8708_C0MSTCR);
}

/* disable the receiver; this must be called with the device spinlock held */
static void it8708_disable_rx(struct ite_dev *dev)
{
	/* disable the receiver interrupts */
	outb(inb(dev->cir_addr + IT8708_C0IER) &
		~(IT85_RDAIE | IT85_RFOIE),
		dev->cir_addr + IT8708_C0IER);

	/* disable the receiver */
	outb(inb(dev->cir_addr + IT8708_C0RCR) & ~IT85_RXEN,
		dev->cir_addr + IT8708_C0RCR);

	/* clear the FIFO and RXACT (actually RXACT should have been cleared
	 * in the previous outb() call) */
	it8708_idle_rx(dev);
}

/* enable the receiver; this must be called with the device spinlock held */
static void it8708_enable_rx(struct ite_dev *dev)
{
	/* enable the receiver by setting RXEN */
	outb(inb(dev->cir_addr + IT8708_C0RCR) | IT85_RXEN,
		dev->cir_addr + IT8708_C0RCR);

	/* just prepare it to idle for the next reception */
	it8708_idle_rx(dev);

	/* enable the receiver interrupts and master enable flag */
	outb(inb(dev->cir_addr + IT8708_C0IER)
		|IT85_RDAIE | IT85_RFOIE | IT85_IEC,
		dev->cir_addr + IT8708_C0IER);
}

/* disable the transmitter interrupt; this must be called with the device
 * spinlock held */
static void it8708_disable_tx_interrupt(struct ite_dev *dev)
{
	/* disable the transmitter interrupts */
	outb(inb(dev->cir_addr + IT8708_C0IER) & ~IT85_TLDLIE,
		dev->cir_addr + IT8708_C0IER);
}

/* enable the transmitter interrupt; this must be called with the device
 * spinlock held */
static void it8708_enable_tx_interrupt(struct ite_dev *dev)
{
	/* enable the transmitter interrupts and master enable flag */
	outb(inb(dev->cir_addr + IT8708_C0IER)
		|IT85_TLDLIE | IT85_IEC,
		dev->cir_addr + IT8708_C0IER);
}

/* disable the device; this must be called with the device spinlock held */
static void it8708_disable(struct ite_dev *dev)
{
	/* clear out all interrupt enable flags */
	outb(inb(dev->cir_addr + IT8708_C0IER) &
		~(IT85_IEC | IT85_RFOIE | IT85_RDAIE | IT85_TLDLIE),
		dev->cir_addr + IT8708_C0IER);

	/* disable the receiver */
	it8708_disable_rx(dev);

	/* erase the FIFO */
	outb(IT85_FIFOCLR | inb(dev->cir_addr + IT8708_C0MSTCR),
		dev->cir_addr + IT8708_C0MSTCR);
}

/* initialize the hardware */
static void it8708_init_hardware(struct ite_dev *dev)
{
	/* disable all the interrupts */
	outb(inb(dev->cir_addr + IT8708_C0IER) &
		~(IT85_IEC | IT85_RFOIE | IT85_RDAIE | IT85_TLDLIE),
		dev->cir_addr + IT8708_C0IER);

	/* program the baud rate divisor */
	outb(inb(dev->cir_addr + IT8708_BANKSEL) | IT8708_HRAE,
		dev->cir_addr + IT8708_BANKSEL);

	outb(ITE_BAUDRATE_DIVISOR & 0xff, dev->cir_addr + IT8708_C0BDLR);
	outb((ITE_BAUDRATE_DIVISOR >> 8) & 0xff,
		   dev->cir_addr + IT8708_C0BDHR);

	outb(inb(dev->cir_addr + IT8708_BANKSEL) & ~IT8708_HRAE,
		   dev->cir_addr + IT8708_BANKSEL);

	/* program the C0MSTCR register defaults */
	outb((inb(dev->cir_addr + IT8708_C0MSTCR) &
			~(IT85_ILSEL | IT85_ILE | IT85_FIFOTL |
			  IT85_FIFOCLR | IT85_RESET)) |
		       IT85_FIFOTL_DEFAULT,
		       dev->cir_addr + IT8708_C0MSTCR);

	/* program the C0RCR register defaults */
	outb((inb(dev->cir_addr + IT8708_C0RCR) &
			~(IT85_RXEN | IT85_RDWOS | IT85_RXEND |
			  IT85_RXACT | IT85_RXDCR)) |
		       ITE_RXDCR_DEFAULT,
		       dev->cir_addr + IT8708_C0RCR);

	/* program the C0TCR register defaults */
	outb((inb(dev->cir_addr + IT8708_C0TCR) &
			~(IT85_TXMPM | IT85_TXMPW))
		       |IT85_TXRLE | IT85_TXENDF |
		       IT85_TXMPM_DEFAULT | IT85_TXMPW_DEFAULT,
		       dev->cir_addr + IT8708_C0TCR);

	/* program the carrier parameters */
	ite_set_carrier_params(dev);
}

/* IT8512F on ITE8709 HW-specific functions */

/* read a byte from the SRAM module */
static inline u8 it8709_rm(struct ite_dev *dev, int index)
{
	outb(index, dev->cir_addr + IT8709_RAM_IDX);
	return inb(dev->cir_addr + IT8709_RAM_VAL);
}

/* write a byte to the SRAM module */
static inline void it8709_wm(struct ite_dev *dev, u8 val, int index)
{
	outb(index, dev->cir_addr + IT8709_RAM_IDX);
	outb(val, dev->cir_addr + IT8709_RAM_VAL);
}

static void it8709_wait(struct ite_dev *dev)
{
	int i = 0;
	/*
	 * loop until device tells it's ready to continue
	 * iterations count is usually ~750 but can sometimes achieve 13000
	 */
	for (i = 0; i < 15000; i++) {
		udelay(2);
		if (it8709_rm(dev, IT8709_MODE) == IT8709_IDLE)
			break;
	}
}

/* read the value of a CIR register */
static u8 it8709_rr(struct ite_dev *dev, int index)
{
	/* just wait in case the previous access was a write */
	it8709_wait(dev);
	it8709_wm(dev, index, IT8709_REG_IDX);
	it8709_wm(dev, IT8709_READ, IT8709_MODE);

	/* wait for the read data to be available */
	it8709_wait(dev);

	/* return the read value */
	return it8709_rm(dev, IT8709_REG_VAL);
}

/* write the value of a CIR register */
static void it8709_wr(struct ite_dev *dev, u8 val, int index)
{
	/* we wait before writing, and not afterwards, since this allows us to
	 * pipeline the host CPU with the microcontroller */
	it8709_wait(dev);
	it8709_wm(dev, val, IT8709_REG_VAL);
	it8709_wm(dev, index, IT8709_REG_IDX);
	it8709_wm(dev, IT8709_WRITE, IT8709_MODE);
}

/* retrieve a bitmask of the current causes for a pending interrupt; this may
 * be composed of ITE_IRQ_TX_FIFO, ITE_IRQ_RX_FIFO and ITE_IRQ_RX_FIFO_OVERRUN
 * */
static int it8709_get_irq_causes(struct ite_dev *dev)
{
	u8 iflags;
	int ret = 0;

	/* read the interrupt flags */
	iflags = it8709_rm(dev, IT8709_IIR);

	if (iflags & IT85_TLDLI)
		ret |= ITE_IRQ_TX_FIFO;
	if (iflags & IT85_RDAI)
		ret |= ITE_IRQ_RX_FIFO;
	if (iflags & IT85_RFOI)
		ret |= ITE_IRQ_RX_FIFO_OVERRUN;

	return ret;
}

/* set the carrier parameters; to be called with the spinlock held */
static void it8709_set_carrier_params(struct ite_dev *dev, bool high_freq,
				      bool use_demodulator,
				      u8 carrier_freq_bits, u8 allowance_bits,
				      u8 pulse_width_bits)
{
	u8 val;

	val = (it8709_rr(dev, IT85_C0CFR)
		     &~(IT85_HCFS | IT85_CFQ)) |
	    carrier_freq_bits;

	if (high_freq)
		val |= IT85_HCFS;

	it8709_wr(dev, val, IT85_C0CFR);

	/* program the C0RCR register */
	val = it8709_rr(dev, IT85_C0RCR)
		& ~(IT85_RXEND | IT85_RXDCR);

	if (use_demodulator)
		val |= IT85_RXEND;

	val |= allowance_bits;

	it8709_wr(dev, val, IT85_C0RCR);

	/* program the C0TCR register */
	val = it8709_rr(dev, IT85_C0TCR) & ~IT85_TXMPW;
	val |= pulse_width_bits;
	it8709_wr(dev, val, IT85_C0TCR);
}

/* read up to buf_size bytes from the RX FIFO; to be called with the spinlock
 * held */
static int it8709_get_rx_bytes(struct ite_dev *dev, u8 * buf, int buf_size)
{
	int fifo, read = 0;

	/* read how many bytes are still in the FIFO */
	fifo = it8709_rm(dev, IT8709_RFSR) & IT85_RXFBC;

	while (fifo > 0 && buf_size > 0) {
		*(buf++) = it8709_rm(dev, IT8709_FIFO + read);
		fifo--;
		read++;
		buf_size--;
	}

	/* 'clear' the FIFO by setting the writing index to 0; this is
	 * completely bound to be racy, but we can't help it, since it's a
	 * limitation of the protocol */
	it8709_wm(dev, 0, IT8709_RFSR);

	return read;
}

/* return how many bytes are still in the FIFO; this will be called
 * with the device spinlock NOT HELD while waiting for the TX FIFO to get
 * empty; let's expect this won't be a problem */
static int it8709_get_tx_used_slots(struct ite_dev *dev)
{
	return it8709_rr(dev, IT85_C0TFSR) & IT85_TXFBC;
}

/* put a byte to the TX fifo; this should be called with the spinlock held */
static void it8709_put_tx_byte(struct ite_dev *dev, u8 value)
{
	it8709_wr(dev, value, IT85_C0DR);
}

/* idle the receiver so that we won't receive samples until another
  pulse is detected; this must be called with the device spinlock held */
static void it8709_idle_rx(struct ite_dev *dev)
{
	/* disable streaming by clearing RXACT writing it as 1 */
	it8709_wr(dev, it8709_rr(dev, IT85_C0RCR) | IT85_RXACT,
			    IT85_C0RCR);

	/* clear the FIFO */
	it8709_wr(dev, it8709_rr(dev, IT85_C0MSTCR) | IT85_FIFOCLR,
			    IT85_C0MSTCR);
}

/* disable the receiver; this must be called with the device spinlock held */
static void it8709_disable_rx(struct ite_dev *dev)
{
	/* disable the receiver interrupts */
	it8709_wr(dev, it8709_rr(dev, IT85_C0IER) &
			    ~(IT85_RDAIE | IT85_RFOIE),
			    IT85_C0IER);

	/* disable the receiver */
	it8709_wr(dev, it8709_rr(dev, IT85_C0RCR) & ~IT85_RXEN,
			    IT85_C0RCR);

	/* clear the FIFO and RXACT (actually RXACT should have been cleared
	 * in the previous it8709_wr(dev, ) call) */
	it8709_idle_rx(dev);
}

/* enable the receiver; this must be called with the device spinlock held */
static void it8709_enable_rx(struct ite_dev *dev)
{
	/* enable the receiver by setting RXEN */
	it8709_wr(dev, it8709_rr(dev, IT85_C0RCR) | IT85_RXEN,
			    IT85_C0RCR);

	/* just prepare it to idle for the next reception */
	it8709_idle_rx(dev);

	/* enable the receiver interrupts and master enable flag */
	it8709_wr(dev, it8709_rr(dev, IT85_C0IER)
			    |IT85_RDAIE | IT85_RFOIE | IT85_IEC,
			    IT85_C0IER);
}

/* disable the transmitter interrupt; this must be called with the device
 * spinlock held */
static void it8709_disable_tx_interrupt(struct ite_dev *dev)
{
	/* disable the transmitter interrupts */
	it8709_wr(dev, it8709_rr(dev, IT85_C0IER) & ~IT85_TLDLIE,
			    IT85_C0IER);
}

/* enable the transmitter interrupt; this must be called with the device
 * spinlock held */
static void it8709_enable_tx_interrupt(struct ite_dev *dev)
{
	/* enable the transmitter interrupts and master enable flag */
	it8709_wr(dev, it8709_rr(dev, IT85_C0IER)
			    |IT85_TLDLIE | IT85_IEC,
			    IT85_C0IER);
}

/* disable the device; this must be called with the device spinlock held */
static void it8709_disable(struct ite_dev *dev)
{
	/* clear out all interrupt enable flags */
	it8709_wr(dev, it8709_rr(dev, IT85_C0IER) &
			~(IT85_IEC | IT85_RFOIE | IT85_RDAIE | IT85_TLDLIE),
		  IT85_C0IER);

	/* disable the receiver */
	it8709_disable_rx(dev);

	/* erase the FIFO */
	it8709_wr(dev, IT85_FIFOCLR | it8709_rr(dev, IT85_C0MSTCR),
			    IT85_C0MSTCR);
}

/* initialize the hardware */
static void it8709_init_hardware(struct ite_dev *dev)
{
	/* disable all the interrupts */
	it8709_wr(dev, it8709_rr(dev, IT85_C0IER) &
			~(IT85_IEC | IT85_RFOIE | IT85_RDAIE | IT85_TLDLIE),
		  IT85_C0IER);

	/* program the baud rate divisor */
	it8709_wr(dev, ITE_BAUDRATE_DIVISOR & 0xff, IT85_C0BDLR);
	it8709_wr(dev, (ITE_BAUDRATE_DIVISOR >> 8) & 0xff,
			IT85_C0BDHR);

	/* program the C0MSTCR register defaults */
	it8709_wr(dev, (it8709_rr(dev, IT85_C0MSTCR) &
			~(IT85_ILSEL | IT85_ILE | IT85_FIFOTL
			  | IT85_FIFOCLR | IT85_RESET)) | IT85_FIFOTL_DEFAULT,
		  IT85_C0MSTCR);

	/* program the C0RCR register defaults */
	it8709_wr(dev, (it8709_rr(dev, IT85_C0RCR) &
			~(IT85_RXEN | IT85_RDWOS | IT85_RXEND | IT85_RXACT
			  | IT85_RXDCR)) | ITE_RXDCR_DEFAULT,
		  IT85_C0RCR);

	/* program the C0TCR register defaults */
	it8709_wr(dev, (it8709_rr(dev, IT85_C0TCR) & ~(IT85_TXMPM | IT85_TXMPW))
			| IT85_TXRLE | IT85_TXENDF | IT85_TXMPM_DEFAULT
			| IT85_TXMPW_DEFAULT,
		  IT85_C0TCR);

	/* program the carrier parameters */
	ite_set_carrier_params(dev);
}


/* IT8305 (ACPI ITE8710) HW-specific functions: the IT8512 CIR register file
 * is mapped directly onto the 16-byte I/O window, no bank switching */

/* hardware baud rate divisor matching sample_period (base clock 115200 Hz) */
static unsigned int it85_hw_divisor(void)
{
	unsigned long d = DIV_ROUND_CLOSEST((unsigned long)sample_period * 115200UL,
					    NSEC_PER_SEC);

	return clamp(d, 1UL, 0xffffUL);
}

static int it85_get_irq_causes(struct ite_dev *dev)
{
	u8 iflags;
	int ret = 0;

	iflags = inb(dev->cir_addr + IT85_C0IIR);

	if (iflags & IT85_TLDLI)
		ret |= ITE_IRQ_TX_FIFO;
	if (iflags & IT85_RDAI)
		ret |= ITE_IRQ_RX_FIFO;
	if (iflags & IT85_RFOI)
		ret |= ITE_IRQ_RX_FIFO_OVERRUN;

	return ret;
}

static void it85_set_carrier_params(struct ite_dev *dev, bool high_freq,
				    bool use_demodulator,
				    u8 carrier_freq_bits, u8 allowance_bits,
				    u8 pulse_width_bits)
{
	u8 val;

	/* program the C0CFR register */
	val = (inb(dev->cir_addr + IT85_C0CFR)
		& ~(IT85_HCFS | IT85_CFQ)) | carrier_freq_bits;

	if (high_freq)
		val |= IT85_HCFS;

	outb(val, dev->cir_addr + IT85_C0CFR);

	/* program the C0RCR register */
	val = inb(dev->cir_addr + IT85_C0RCR) & ~(IT85_RXEND | IT85_RXDCR);

	if (use_demodulator)
		val |= IT85_RXEND;

	val |= allowance_bits;

	outb(val, dev->cir_addr + IT85_C0RCR);

	/* program the C0TCR register */
	val = inb(dev->cir_addr + IT85_C0TCR) & ~IT85_TXMPW;
	val |= pulse_width_bits;
	outb(val, dev->cir_addr + IT85_C0TCR);
}

static int it85_get_rx_bytes(struct ite_dev *dev, u8 *buf, int buf_size)
{
	int fifo, read = 0;

	fifo = inb(dev->cir_addr + IT85_C0RFSR) & IT85_RXFBC;

	while (fifo > 0 && buf_size > 0) {
		*(buf++) = inb(dev->cir_addr + IT85_C0DR);
		fifo--;
		read++;
		buf_size--;
	}

	return read;
}

static int it85_get_tx_used_slots(struct ite_dev *dev)
{
	return inb(dev->cir_addr + IT85_C0TFSR) & IT85_TXFBC;
}

static void it85_put_tx_byte(struct ite_dev *dev, u8 value)
{
	outb(value, dev->cir_addr + IT85_C0DR);
}

static void it85_idle_rx(struct ite_dev *dev)
{
	/* disable streaming by clearing RXACT writing it as 1 */
	outb(inb(dev->cir_addr + IT85_C0RCR) | IT85_RXACT,
		dev->cir_addr + IT85_C0RCR);

	/* clear the FIFO */
	outb(inb(dev->cir_addr + IT85_C0MSTCR) | IT85_FIFOCLR,
		dev->cir_addr + IT85_C0MSTCR);
}

static void it85_disable_rx(struct ite_dev *dev)
{
	outb(inb(dev->cir_addr + IT85_C0IER) & ~(IT85_RDAIE | IT85_RFOIE),
		dev->cir_addr + IT85_C0IER);

	outb(inb(dev->cir_addr + IT85_C0RCR) & ~IT85_RXEN,
		dev->cir_addr + IT85_C0RCR);

	it85_idle_rx(dev);
}

static void it85_enable_rx(struct ite_dev *dev)
{
	outb(inb(dev->cir_addr + IT85_C0RCR) | IT85_RXEN,
		dev->cir_addr + IT85_C0RCR);

	it85_idle_rx(dev);

	outb(inb(dev->cir_addr + IT85_C0IER)
		| IT85_RDAIE | IT85_RFOIE | IT85_IEC,
		dev->cir_addr + IT85_C0IER);
}

static void it85_disable_tx_interrupt(struct ite_dev *dev)
{
	outb(inb(dev->cir_addr + IT85_C0IER) & ~IT85_TLDLIE,
		dev->cir_addr + IT85_C0IER);
}

static void it85_enable_tx_interrupt(struct ite_dev *dev)
{
	outb(inb(dev->cir_addr + IT85_C0IER) | IT85_TLDLIE | IT85_IEC,
		dev->cir_addr + IT85_C0IER);
}

static void it85_disable(struct ite_dev *dev)
{
	outb(inb(dev->cir_addr + IT85_C0IER) &
		~(IT85_IEC | IT85_RFOIE | IT85_RDAIE | IT85_TLDLIE),
		dev->cir_addr + IT85_C0IER);

	it85_disable_rx(dev);

	outb(IT85_FIFOCLR | inb(dev->cir_addr + IT85_C0MSTCR),
		dev->cir_addr + IT85_C0MSTCR);
}

static void it85_init_hardware(struct ite_dev *dev)
{
	unsigned int div = it85_hw_divisor();

	/* disable all the interrupts */
	outb(inb(dev->cir_addr + IT85_C0IER) &
		~(IT85_IEC | IT85_RFOIE | IT85_RDAIE | IT85_TLDLIE),
		dev->cir_addr + IT85_C0IER);

	/* program the baud rate divisor */
	outb(div & 0xff, dev->cir_addr + IT85_C0BDLR);
	outb((div >> 8) & 0xff, dev->cir_addr + IT85_C0BDHR);

	/* program the C0MSTCR register defaults */
	outb((inb(dev->cir_addr + IT85_C0MSTCR) &
			~(IT85_ILSEL | IT85_ILE | IT85_FIFOTL |
			  IT85_FIFOCLR | IT85_RESET)) | IT85_FIFOTL_DEFAULT,
		dev->cir_addr + IT85_C0MSTCR);

	/* program the C0RCR register defaults */
	outb((inb(dev->cir_addr + IT85_C0RCR) &
			~(IT85_RXEN | IT85_RDWOS | IT85_RXEND |
			  IT85_RXACT | IT85_RXDCR)) | ITE_RXDCR_DEFAULT,
		dev->cir_addr + IT85_C0RCR);

	/* program the C0TCR register defaults */
	outb((inb(dev->cir_addr + IT85_C0TCR) &
			~(IT85_TXMPM | IT85_TXMPW))
		| IT85_TXRLE | IT85_TXENDF |
		IT85_TXMPM_DEFAULT | IT85_TXMPW_DEFAULT,
		dev->cir_addr + IT85_C0TCR);

	/* the wakeup registers (C0WCL/C0WCR/C0WPS) are left untouched, so
	 * that power-on from the remote keeps working */

	/* program the carrier parameters */
	ite_set_carrier_params(dev);
}


/* polling mode */

static void ite_poll_timer(struct timer_list *t)
{
	struct ite_dev *dev = from_timer(dev, t, poll_timer);
	u8 rx_buf[ITE_RX_FIFO_LEN];
	unsigned long flags;
	int rx_bytes = 0;
	bool again;

	spin_lock_irqsave(&dev->lock, flags);
	if (dev->polling)
		rx_bytes = dev->params->get_rx_bytes(dev, rx_buf,
						     ITE_RX_FIFO_LEN);
	spin_unlock_irqrestore(&dev->lock, flags);

	if (rx_bytes > 0)
		ite_decode_bytes(dev, rx_buf, rx_bytes);

	spin_lock_irqsave(&dev->lock, flags);
	again = dev->polling;
	spin_unlock_irqrestore(&dev->lock, flags);

	if (again)
		mod_timer(&dev->poll_timer,
			  jiffies + max(1UL, msecs_to_jiffies(poll_ms)));
}

static void ite_poll_start(struct ite_dev *dev)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);
	dev->polling = true;
	spin_unlock_irqrestore(&dev->lock, flags);

	mod_timer(&dev->poll_timer, jiffies + 1);
}

static void ite_poll_stop(struct ite_dev *dev)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);
	dev->polling = false;
	spin_unlock_irqrestore(&dev->lock, flags);

	timer_delete_sync(&dev->poll_timer);
}


/* generic hardware setup/teardown code */

/* activate the device for use */
static int ite_open(struct rc_dev *rcdev)
{
	struct ite_dev *dev = rcdev->priv;
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);

	/* enable the receiver */
	dev->params->enable_rx(dev);

	spin_unlock_irqrestore(&dev->lock, flags);

	if (poll)
		ite_poll_start(dev);

	return 0;
}

/* deactivate the device for use */
static void ite_close(struct rc_dev *rcdev)
{
	struct ite_dev *dev = rcdev->priv;
	unsigned long flags;

	if (poll)
		ite_poll_stop(dev);

	spin_lock_irqsave(&dev->lock, flags);

	/* wait for any transmission to end */
	spin_unlock_irqrestore(&dev->lock, flags);
	wait_event_interruptible(dev->tx_ended, !dev->transmitting);
	spin_lock_irqsave(&dev->lock, flags);

	dev->params->disable(dev);

	spin_unlock_irqrestore(&dev->lock, flags);
}

/* supported models and their parameters */
static const struct ite_dev_params ite_dev_descs[] = {
	{	/* 0: ITE8704 */
	       .model = "ITE8704 CIR transceiver",
	       .io_region_size = IT87_IOREG_LENGTH,
	       .io_rsrc_no = 0,

		/* operations */
	       .get_irq_causes = it87_get_irq_causes,
	       .enable_rx = it87_enable_rx,
	       .idle_rx = it87_idle_rx,
	       .disable_rx = it87_idle_rx,
	       .get_rx_bytes = it87_get_rx_bytes,
	       .enable_tx_interrupt = it87_enable_tx_interrupt,
	       .disable_tx_interrupt = it87_disable_tx_interrupt,
	       .get_tx_used_slots = it87_get_tx_used_slots,
	       .put_tx_byte = it87_put_tx_byte,
	       .disable = it87_disable,
	       .init_hardware = it87_init_hardware,
	       .set_carrier_params = it87_set_carrier_params,
	       },
	{	/* 1: ITE8713 */
	       .model = "ITE8713 CIR transceiver",
	       .io_region_size = IT87_IOREG_LENGTH,
	       .io_rsrc_no = 0,

		/* operations */
	       .get_irq_causes = it87_get_irq_causes,
	       .enable_rx = it87_enable_rx,
	       .idle_rx = it87_idle_rx,
	       .disable_rx = it87_idle_rx,
	       .get_rx_bytes = it87_get_rx_bytes,
	       .enable_tx_interrupt = it87_enable_tx_interrupt,
	       .disable_tx_interrupt = it87_disable_tx_interrupt,
	       .get_tx_used_slots = it87_get_tx_used_slots,
	       .put_tx_byte = it87_put_tx_byte,
	       .disable = it87_disable,
	       .init_hardware = it87_init_hardware,
	       .set_carrier_params = it87_set_carrier_params,
	       },
	{	/* 2: ITE8708 */
	       .model = "ITE8708 CIR transceiver",
	       .io_region_size = IT8708_IOREG_LENGTH,
	       .io_rsrc_no = 0,

		/* operations */
	       .get_irq_causes = it8708_get_irq_causes,
	       .enable_rx = it8708_enable_rx,
	       .idle_rx = it8708_idle_rx,
	       .disable_rx = it8708_idle_rx,
	       .get_rx_bytes = it8708_get_rx_bytes,
	       .enable_tx_interrupt = it8708_enable_tx_interrupt,
	       .disable_tx_interrupt =
	       it8708_disable_tx_interrupt,
	       .get_tx_used_slots = it8708_get_tx_used_slots,
	       .put_tx_byte = it8708_put_tx_byte,
	       .disable = it8708_disable,
	       .init_hardware = it8708_init_hardware,
	       .set_carrier_params = it8708_set_carrier_params,
	       },
	{	/* 3: ITE8709 */
	       .model = "ITE8709 CIR transceiver",
	       .io_region_size = IT8709_IOREG_LENGTH,
	       .io_rsrc_no = 2,

		/* operations */
	       .get_irq_causes = it8709_get_irq_causes,
	       .enable_rx = it8709_enable_rx,
	       .idle_rx = it8709_idle_rx,
	       .disable_rx = it8709_idle_rx,
	       .get_rx_bytes = it8709_get_rx_bytes,
	       .enable_tx_interrupt = it8709_enable_tx_interrupt,
	       .disable_tx_interrupt =
	       it8709_disable_tx_interrupt,
	       .get_tx_used_slots = it8709_get_tx_used_slots,
	       .put_tx_byte = it8709_put_tx_byte,
	       .disable = it8709_disable,
	       .init_hardware = it8709_init_hardware,
	       .set_carrier_params = it8709_set_carrier_params,
	       },
	{	/* 4: ITE8710 (IT8305), IT8512 CIR registers mapped directly */
	       .model = "ITE8710 (IT8305) CIR transceiver",
	       .io_region_size = IT85_IOREG_LENGTH,
	       .io_rsrc_no = 0,

		/* operations */
	       .get_irq_causes = it85_get_irq_causes,
	       .enable_rx = it85_enable_rx,
	       .idle_rx = it85_idle_rx,
	       .disable_rx = it85_disable_rx,
	       .get_rx_bytes = it85_get_rx_bytes,
	       .enable_tx_interrupt = it85_enable_tx_interrupt,
	       .disable_tx_interrupt = it85_disable_tx_interrupt,
	       .get_tx_used_slots = it85_get_tx_used_slots,
	       .put_tx_byte = it85_put_tx_byte,
	       .disable = it85_disable,
	       .init_hardware = it85_init_hardware,
	       .set_carrier_params = it85_set_carrier_params,
	       },
};

/* only the ITE8710 is claimed here; the other IDs stay with the in-tree
 * PNP driver */
static const struct acpi_device_id ite_acpi_ids[] = {
	{ "ITE8710", 4 },
	{ }
};
MODULE_DEVICE_TABLE(acpi, ite_acpi_ids);

/* allocate memory, probe hardware, and initialize everything */
static int ite_probe(struct platform_device *pdev)
{
	const struct ite_dev_params *dev_desc = NULL;
	const struct acpi_device_id *id;
	struct ite_dev *itdev = NULL;
	struct rc_dev *rdev = NULL;
	struct resource *res;
	int irq = -1;
	int ret = -ENOMEM;
	int model_no;

	/* a slower sample rate is needed when polling, so that the 32-byte
	 * FIFO doesn't overflow between polls */
	if (poll && sample_period == NSEC_PER_SEC / 115200)
		sample_period *= 8;

	itdev = kzalloc(sizeof(struct ite_dev), GFP_KERNEL);
	if (!itdev)
		return ret;

	/* input device for IR remote (and tx) */
	rdev = rc_allocate_device(RC_DRIVER_IR_RAW);
	if (!rdev)
		goto exit_free_dev_rdev;
	itdev->rdev = rdev;

	ret = -ENODEV;

	/* get the model number */
	id = acpi_match_device(ite_acpi_ids, &pdev->dev);
	model_no = id ? (int)id->driver_data : 4;

	if (model_number >= 0 && model_number < ARRAY_SIZE(ite_dev_descs)) {
		model_no = model_number;
		dev_info(&pdev->dev, "model has been forced to: %s",
			 ite_dev_descs[model_no].model);
	}

	/* get the description for the device */
	dev_desc = &ite_dev_descs[model_no];

	/* validate resources */
	res = platform_get_resource(pdev, IORESOURCE_IO, 0);
	if (!res || resource_size(res) < dev_desc->io_region_size) {
		dev_err(&pdev->dev, "IR I/O port not valid!\n");
		goto exit_free_dev_rdev;
	}

	if (!poll) {
		irq = platform_get_irq(pdev, 0);
		if (irq < 0) {
			dev_err(&pdev->dev, "IRQ not valid!\n");
			ret = irq;
			goto exit_free_dev_rdev;
		}
	}

	/* store resource values */
	itdev->cir_addr = res->start;
	itdev->cir_irq = irq;

	/* initialize spinlocks */
	spin_lock_init(&itdev->lock);

	/* set driver data into the platform device */
	platform_set_drvdata(pdev, itdev);
	itdev->pdev = pdev;

	/* initialize waitqueues for transmission */
	init_waitqueue_head(&itdev->tx_queue);
	init_waitqueue_head(&itdev->tx_ended);

	/* polling timer */
	timer_setup(&itdev->poll_timer, ite_poll_timer, 0);

	/* Set model-specific parameters */
	itdev->params = dev_desc;

	/* set up hardware initial state */
	itdev->tx_duty_cycle = 33;
	itdev->tx_carrier_freq = ITE_DEFAULT_CARRIER_FREQ;
	itdev->params->init_hardware(itdev);

	/* set up ir-core props */
	rdev->priv = itdev;
	rdev->dev.parent = &pdev->dev;
	rdev->allowed_protocols = RC_PROTO_BIT_ALL_IR_DECODER;
	rdev->open = ite_open;
	rdev->close = ite_close;
	rdev->s_idle = ite_s_idle;
	rdev->s_rx_carrier_range = ite_set_rx_carrier_range;
	/* FIFO threshold is 17 bytes, so 17 * 8 samples minimum */
	rdev->min_timeout = 17 * 8 * ITE_BAUDRATE_DIVISOR *
			    sample_period / 1000;
	rdev->timeout = IR_DEFAULT_TIMEOUT;
	rdev->max_timeout = 10 * IR_DEFAULT_TIMEOUT;
	rdev->rx_resolution = ITE_BAUDRATE_DIVISOR * sample_period / 1000;

	/* set up transmitter related values */
	rdev->tx_ir = ite_tx_ir;
	rdev->s_tx_carrier = ite_set_tx_carrier;
	rdev->s_tx_duty_cycle = ite_set_tx_duty_cycle;

	rdev->device_name = dev_desc->model;
	rdev->input_id.bustype = BUS_HOST;
	rdev->input_id.vendor = PCI_VENDOR_ID_ITE;
	rdev->input_id.product = 0x8710;
	rdev->input_id.version = 0;
	rdev->driver_name = ITE_DRIVER_NAME;
	rdev->map_name = RC_MAP_RC6_MCE;

	ret = rc_register_device(rdev);
	if (ret)
		goto exit_free_dev_rdev;

	ret = -EBUSY;
	/* now claim resources */
	if (!request_region(itdev->cir_addr,
				dev_desc->io_region_size, ITE_DRIVER_NAME))
		goto exit_unregister_device;

	if (!poll && request_irq(itdev->cir_irq, ite_cir_isr, IRQF_SHARED,
				 ITE_DRIVER_NAME, (void *)itdev))
		goto exit_release_cir_addr;

	dev_info(&pdev->dev, "%s at 0x%lx, %s %d, sample period %ld ns (divisor %u)\n",
		 dev_desc->model, itdev->cir_addr,
		 poll ? "polling every ms:" : "IRQ",
		 poll ? (int)poll_ms : itdev->cir_irq,
		 sample_period, it85_hw_divisor());

	return 0;

exit_release_cir_addr:
	release_region(itdev->cir_addr, itdev->params->io_region_size);
exit_unregister_device:
	rc_unregister_device(rdev);
exit_free_dev_rdev:
	rc_free_device(rdev);
	kfree(itdev);

	return ret;
}

static void ite_remove(struct platform_device *pdev)
{
	struct ite_dev *dev = platform_get_drvdata(pdev);
	unsigned long flags;

	ite_poll_stop(dev);

	spin_lock_irqsave(&dev->lock, flags);

	/* disable hardware */
	dev->params->disable(dev);

	spin_unlock_irqrestore(&dev->lock, flags);

	/* free resources */
	if (!poll)
		free_irq(dev->cir_irq, dev);
	release_region(dev->cir_addr, dev->params->io_region_size);

	rc_unregister_device(dev->rdev);
	rc_free_device(dev->rdev);

	kfree(dev);
}

static int ite_suspend(struct device *d)
{
	struct ite_dev *dev = dev_get_drvdata(d);
	unsigned long flags;

	/* wait for any transmission to end */
	wait_event_interruptible(dev->tx_ended, !dev->transmitting);

	dev->resume_poll = dev->polling;
	if (dev->resume_poll)
		ite_poll_stop(dev);

	spin_lock_irqsave(&dev->lock, flags);

	/* disable all interrupts */
	dev->params->disable(dev);

	spin_unlock_irqrestore(&dev->lock, flags);

	return 0;
}

static int ite_resume(struct device *d)
{
	struct ite_dev *dev = dev_get_drvdata(d);
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);

	/* reinitialize hardware config registers */
	dev->params->init_hardware(dev);
	/* enable the receiver */
	dev->params->enable_rx(dev);

	spin_unlock_irqrestore(&dev->lock, flags);

	if (dev->resume_poll)
		ite_poll_start(dev);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(ite_pm_ops, ite_suspend, ite_resume);

static void ite_shutdown(struct platform_device *pdev)
{
	struct ite_dev *dev = platform_get_drvdata(pdev);
	unsigned long flags;

	ite_poll_stop(dev);

	spin_lock_irqsave(&dev->lock, flags);

	/* disable all interrupts */
	dev->params->disable(dev);

	spin_unlock_irqrestore(&dev->lock, flags);
}

static struct platform_driver ite_driver = {
	.driver = {
		.name			= "ite-cir-8305",
		.acpi_match_table	= ite_acpi_ids,
		.pm			= pm_sleep_ptr(&ite_pm_ops),
	},
	.probe		= ite_probe,
	.remove		= ite_remove,
	.shutdown	= ite_shutdown,
};

MODULE_DESCRIPTION("ITE8710 (IT8305) CIR driver, based on ite-cir");

MODULE_AUTHOR("Juan J. Garcia de Soria <skandalfo@gmail.com>");
MODULE_LICENSE("GPL");

module_platform_driver(ite_driver);
