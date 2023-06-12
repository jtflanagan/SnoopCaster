#pragma once

#include "eth.h"
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pico/critical_section.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"

#define ETH_SPI_PORT spi0
#define ETH_PIN_INT 21
#define ETH_PIN_SCK 18
#define ETH_PIN_MOSI 19
#define ETH_PIN_MISO 16
#define ETH_PIN_CS 17
#define ETH_PIN_RST 20

void eth_read(uint32_t addr, uint8_t* buf, uint16_t len, bool blocking);
void eth_write(uint32_t addr, uint8_t* buf, uint16_t len, bool blocking);
uint16_t eth_read16(uint32_t reg);
void eth_write16(uint32_t reg, uint16_t val);
uint8_t eth_read8(uint32_t reg);
void eth_write8(uint32_t reg, uint8_t val);
void eth_read_packet(uint8_t* buf, uint16_t len);
void eth_write_packet(uint8_t* buf, uint16_t len);

bool is_wizchip_busy();

#define _W5500_IO_BASE_ 0x00000000

#define _W5500_SPI_READ_ (0x00 << 2)
#define _W5500_SPI_WRITE_ (0x01 << 2)
#define _W5500_SPI_VDM_OP_ (0x00)

#define WIZCHIP_CREG_BLOCK 0x00
#define WIZCHIP_SREG_BLOCK(N) (1+4*N)
#define WIZCHIP_TXBUF_BLOCK(N) (2+4*N)
#define WIZCHIP_RXBUF_BLOCK(N) (3+4*N)

// common registers

// mode register
#define ETH_MR (_W5500_IO_BASE_ + (0x0000 << 8) + (WIZCHIP_CREG_BLOCK << 3))

// gateway IP register
#define ETH_GAR (_W5500_IO_BASE_ + (0x0001 << 8) + (WIZCHIP_CREG_BLOCK << 3))

// Subnet mask register
#define ETH_SUBR (_W5500_IO_BASE_ + (0x0005 << 8) + (WIZCHIP_CREG_BLOCK << 3))

// source hardware mac
#define ETH_SHAR (_W5500_IO_BASE_ + (0x0009 << 8) + (WIZCHIP_CREG_BLOCK << 3))

// source IP register
#define ETH_SIPR (_W5500_IO_BASE_ + (0x000F << 8) + (WIZCHIP_CREG_BLOCK << 3))

// interrupt low level timer
#define ETH_INTLEVEL (_W5500_IO_BASE_ + (0x0013 << 8) + (WIZCHIP_CREG_BLOCK << 3))

// interrupt register
#define ETH_IR (_W5500_IO_BASE_ + (0x0015 << 8) + (WIZCHIP_CREG_BLOCK << 3))

// interrupt mask register
#define ETH_IMR (_W5500_IO_BASE_ + (0x0016 << 8) + (WIZCHIP_CREG_BLOCK << 3))

// socket interrupt register
#define ETH_SIR (_W5500_IO_BASE_ + (0x0017 << 8) + (WIZCHIP_CREG_BLOCK << 3))

// socket interrupt mask register
#define ETH_SIMR (_W5500_IO_BASE_ + (0x0018 << 8) + (WIZCHIP_CREG_BLOCK << 3))

// retrans timeout register
#define ETH_RTR (_W5500_IO_BASE_ + (0x0019 << 8) + (WIZCHIP_CREG_BLOCK << 3))

// retry count register
#define ETH_RCR (_W5500_IO_BASE_ + (0x001B << 8) + (WIZCHIP_CREG_BLOCK << 3))

// PPP LCP request timer
#define ETH_PTIMER (_W5500_IO_BASE_ + (0x001C << 8) + (WIZCHIP_CREG_BLOCK << 3))

// PPP LCP magic number
#define ETH_PMAGIC (_W5500_IO_BASE_ + (0x001D << 8) + (WIZCHIP_CREG_BLOCK << 3))

// PPP dest mac
#define ETH_PHAR (_W5500_IO_BASE_ + (0x001E << 8) + (WIZCHIP_CREG_BLOCK << 3))

// PPP session ID
#define ETH_PSID (_W5500_IO_BASE_ + (0x0024 << 8) + (WIZCHIP_CREG_BLOCK << 3))

// PPP MSS
#define ETH_PMRU (_W5500_IO_BASE_ + (0x0026 << 8) + (WIZCHIP_CREG_BLOCK << 3))

// unreachable IP addr for udp
#define ETH_UIPR (_W5500_IO_BASE_ + (0x0028 << 8) + (WIZCHIP_CREG_BLOCK << 3))

// unreachable port for udp
#define ETH_UPORTR (_W5500_IO_BASE_ + (0x002C << 8) + (WIZCHIP_CREG_BLOCK << 3))

// PHY status
#define ETH_PHYCFGR (_W5500_IO_BASE_ + (0x002E << 8) + (WIZCHIP_CREG_BLOCK << 3))

// version
#define ETH_VERSIONR (_W5500_IO_BASE_ + (0x0039 << 8) + (WIZCHIP_CREG_BLOCK << 3))

// socket registers

// mode
#define ETHS_MR(N) (_W5500_IO_BASE_ + (0x0000 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

// command
#define ETHS_CR(N) (_W5500_IO_BASE_ + (0x0001 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

// interrupt
#define ETHS_IR(N) (_W5500_IO_BASE_ + (0x0002 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

// status
#define ETHS_SR(N) (_W5500_IO_BASE_ + (0x0003 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

// port
#define ETHS_PORT(N) (_W5500_IO_BASE_ + (0x0004 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

// dest mac
#define ETHS_DHAR(N) (_W5500_IO_BASE_ + (0x0006 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

// dest IP
#define ETHS_DIPR(N) (_W5500_IO_BASE_ + (0x000C << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

// dest port
#define ETHS_DPORT(N) (_W5500_IO_BASE_ + (0x0010 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

// MSS
#define ETHS_MSSR(N) (_W5500_IO_BASE_ + (0x0012 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

// Type of service
#define ETHS_TOS(N) (_W5500_IO_BASE_ + (0x0015 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

// TTL
#define ETHS_TTL(N) (_W5500_IO_BASE_ + (0x0016 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

// RX buffer size
#define ETHS_RXBUF_SIZE(N) (_W5500_IO_BASE_ + (0x001E << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

// TX buffer size
#define ETHS_TXBUF_SIZE(N) (_W5500_IO_BASE_ + (0x001F << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

// TX free size
#define ETHS_TX_FSR(N) (_W5500_IO_BASE_ + (0x0020 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

// TX read ptr
#define ETHS_TX_RD(N) (_W5500_IO_BASE_ + (0x0022 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

// TX write ptr
#define ETHS_TX_WR(N) (_W5500_IO_BASE_ + (0x0024 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

// RX data size
#define ETHS_RX_RSR(N) (_W5500_IO_BASE_ + (0x0026 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

// RX read ptr
#define ETHS_RX_RD(N) (_W5500_IO_BASE_ + (0x0028 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

// RX write ptr
#define ETHS_RX_WR(N) (_W5500_IO_BASE_ + (0x002A << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

// socket interrupt mask
#define ETHS_IMR(N) (_W5500_IO_BASE_ + (0x002C << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

// socket fragment field
#define ETHS_FRAG(N) (_W5500_IO_BASE_ + (0x002D << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

// keep alive timer
#define ETHS_KPALVTR(N) (_W5500_IO_BASE_ + (0x002F << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

#define ETH_MR_RST 0x80

#define ETH_IR_SENDOK 0x10
#define ETH_IR_TIMEOUT 0x08
#define ETH_IR_RECV 0x04

#define ETH_CR_OPEN 0x01
#define ETH_CR_CLOSE 0x10
#define ETH_CR_SEND 0x20
#define ETH_CR_SEND_MAC 0x21
#define ETH_CR_RECV 0x40

#define ETH_SOCK_CLOSED 0x00
#define ETH_SOCK_UDP 0x22

#define ETH_MR_UDP 0x02
