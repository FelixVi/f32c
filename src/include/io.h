
#ifndef _IO_H_
#define	_IO_H_

#define IO_BASE		-32768

#define	IO_ADDR(a)	(IO_BASE + (a))

#define	IO_LED		IO_ADDR(0x00)	/* byte, WR */
#define	IO_PUSHBTN	IO_ADDR(0x00)	/* byte, RD */
#define	IO_DIPSW	IO_ADDR(0x01)	/* byte, RD */
#define	IO_SIO_BYTE	IO_ADDR(0x04)	/* byte, RW */
#define	IO_SIO_STATUS	IO_ADDR(0x05)	/* byte, RD */
#define	IO_SIO_BAUD	IO_ADDR(0x06)	/* half, WR */
#define	IO_TSC		IO_ADDR(0x08)	/* word, RD */
#define	IO_PCM_OUT	IO_ADDR(0x0c)	/* word, WR */
#define	IO_SPI_FLASH	IO_ADDR(0x10)	/* byte, RW */
#define	IO_SPI_MICROSD	IO_ADDR(0x14)	/* byte, RW */
#define	IO_DDS		IO_ADDR(0x1c)	/* word, WR */

/* SIO status bitmask */
#define	SIO_TX_BUSY	0x4
#define	SIO_RX_OVERRUN	0x2
#define	SIO_RX_FULL	0x1

/* SPI bitmask: outputs */
#define	SPI_SI		0x80
#define	SPI_SCK		0x40
#define	SPI_CEN		0x20
/* SPI bitmask: input */
#define	SPI_SO_BITPOS	0
#define	SPI_SO		(1 << SPI_SO_BITPOS)

/* Pushbutton bitmask */
#define	BTN_CENTER	0x10
#define	BTN_UP		0x08
#define	BTN_DOWN	0x04
#define	BTN_LEFT	0x02
#define	BTN_RIGHT	0x01

#if !defined(__ASSEMBLER__)

/* I/O macros */

#define	OUTB(port, data)						   \
	do {								   \
		__asm __volatile ("sb %0, %1($0)"	/* IO_BASE = 0xf* */ \
			:				/* outputs */	   \
			: "r" (data), "i" (port));	/* inputs */	   \
	} while (0)

#define	OUTH(port, data)						   \
	do {								   \
		__asm __volatile ("sh %0, %1($0)"	/* IO_BASE = 0xf* */ \
			:				/* outputs */	   \
			: "r" (data), "i" (port));	/* inputs */	   \
	} while (0)

#define	OUTW(port, data)						   \
	do {								   \
		__asm __volatile ("sw %0, %1($0)"	/* IO_BASE = 0xf* */ \
			:				/* outputs */	   \
			: "r" (data), "i" (port));	/* inputs */	   \
	} while (0)

#define	INB(data, port)							   \
	do {								   \
		__asm __volatile ("lb %0, %1($0)"	/* IO_BASE = 0xf* */ \
			: "=r" (data)			/* outputs */	   \
			: "i" (port));			/* inputs */	   \
	} while (0)

#define	INH(data, port)							   \
	do {								   \
		__asm __volatile ("lh %0, %1($0)"	/* IO_BASE = 0xf* */ \
			: "=r" (data)			/* outputs */	   \
			: "i" (port));			/* inputs */	   \
	} while (0)

#define	INW(data, port)							   \
	do {								   \
		__asm __volatile ("lw %0, %1($0)"	/* IO_BASE = 0xf* */ \
			: "=r" (data)			/* outputs */	   \
			: "i" (port));			/* inputs */	   \
	} while (0)


/*
 * Declaration of misc. IO functions.
 */

#define	spi_start_transaction()	OUTB(IO_SPI_FLASH, 0);
#define	spi_stop_transaction()	OUTB(IO_SPI_FLASH, SPI_CEN);

int spi_byte(int);
int spi_byte_in(void);

inline int
rdtsc(void) {
	int tsc;

	INW(tsc, IO_TSC);
	return (tsc);
}

#endif /* __ASSEMBLER__ */

#endif /* !_IO_H_ */

