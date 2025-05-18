#ifndef _SPIDER_PROTOCOL_H
#define _SPIDER_PROTOCOL_H

// Definitions that are shared between Amiga and RP2040.

// Clockport registers.
#define REG_RESERVED_0          0
#define REG_RESERVED_1          1
#define REG_UPPER_LENGTH        2   // WO, Upper byte for lengths
#define REG_CARD_DETECT         3   // RO, Read CD
#define REG_RX_HEAD             4   // RO
#define REG_RX_TAIL             5   // RO
#define REG_TX_HEAD             6   // RO
#define REG_TX_TAIL             7   // RO
#define REG_RX_DISCARD          8   // WO, Lower byte
#define REG_TX_FEED             9   // WO, Lower byte
#define REG_SPI_FREQ            10  // WO, Set SPI frequency
#define REG_SLAVE_SELECT        11  // WO, Write SS
#define REG_INT_FIRED           12  // RW
#define REG_INT_ARMED           13  // WO
#define REG_FIFO                14  // RW, Write to TX, Read from RX
#define REG_IDENT               15  // RO

// Interrupts.
#define IRQ_CD_CHANGED          1

#endif // _SPIDER_PROTOCOL_H
