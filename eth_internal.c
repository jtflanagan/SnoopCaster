#include "eth_internal.h"
#include <stdio.h>

static critical_section_t eth_cri_sec;
static uint eth_dma_tx;
static uint eth_dma_rx;
static dma_channel_config eth_dma_channel_config_tx;
static dma_channel_config eth_dma_channel_config_rx;
static uint8_t eth_shar[9] = {0, 0, 0, 0x00, 0x08, 0xDC, 0x12, 0x34, 0x56};
static uint8_t eth_sipr[7] = {0, 0, 0, 192, 168, 1, 253};
static uint8_t eth_gar[7] = {0, 0, 0, 192, 168, 1, 1};
static uint8_t eth_subr[7] = {0, 0, 0, 255, 255, 255, 0};

static uint16_t tx_write_ptr;
static uint16_t tx_read_ptr;

enum CMD_STATE {
  ETH_IDLE,
  ETH_TX_DMA,
  ETH_TX_SEND,
  ETH_TX_WAIT,
  ETH_RX_DMA,
  ETH_RX_RECV,
  ETH_RX_WAIT,
};

static int eth_cmd_state = ETH_IDLE;
static int prev_eth_cmd_state = ETH_IDLE;

void print_eth_cmd_state() {
  switch (eth_cmd_state) {
  case ETH_IDLE:
    printf("ETH_IDLE\n");
    return;
  case ETH_TX_DMA:
    printf("ETH_TX_DMA\n");
    return;
  case ETH_TX_SEND:
    printf("ETH_TX_SEND\n");
    return;
  case ETH_TX_WAIT:
    printf("ETH_TX_WAIT\n");
    return;
  case ETH_RX_DMA:
    printf("ETH_RX_DMA\n");
    return;
  case ETH_RX_RECV:
    printf("ETH_RX_RECV\n");
    return;
  case ETH_RX_WAIT:
    printf("ETH_RX_WAIT\n");
    return;
  default:
    printf("unknown wiz_cmd_state\n");
  }
}

static inline void eth_select(void) {
  gpio_put(ETH_PIN_CS, 0);
}

static inline void eth_deselect(void) {
  gpio_put(ETH_PIN_CS, 1);
}



void eth_init() {

  // initialize spi
  critical_section_init(&eth_cri_sec);
  spi_init(ETH_SPI_PORT, 62500 * 1000);
  gpio_set_function(ETH_PIN_SCK, GPIO_FUNC_SPI);
  gpio_set_function(ETH_PIN_MOSI, GPIO_FUNC_SPI);
  gpio_set_function(ETH_PIN_MISO, GPIO_FUNC_SPI);
  bi_decl(bi_3pins_with_func(ETH_PIN_MISO, ETH_PIN_MOSI, ETH_PIN_SCK, GPIO_FUNC_SPI));
  gpio_init(ETH_PIN_CS);
  gpio_set_dir(ETH_PIN_CS, GPIO_OUT);
  gpio_put(ETH_PIN_CS, 1);
  bi_decl(bi_1pin_with_name(ETH_PIN_CS, "W5500 CHIP SELECT"));

  // initialize spi dma
  eth_dma_tx = dma_claim_unused_channel(true);
  eth_dma_rx = dma_claim_unused_channel(true);

  eth_dma_channel_config_tx = dma_channel_get_default_config(eth_dma_tx);
  channel_config_set_transfer_data_size(&eth_dma_channel_config_tx, DMA_SIZE_8);
  channel_config_set_dreq(&eth_dma_channel_config_tx, DREQ_SPI0_TX);

  eth_dma_channel_config_rx = dma_channel_get_default_config(eth_dma_rx);
  channel_config_set_transfer_data_size(&eth_dma_channel_config_rx, DMA_SIZE_8);
  channel_config_set_dreq(&eth_dma_channel_config_rx, DREQ_SPI0_RX);
  channel_config_set_read_increment(&eth_dma_channel_config_rx, false);
  channel_config_set_write_increment(&eth_dma_channel_config_rx, true);

  // reset module
  gpio_set_dir(ETH_PIN_RST, GPIO_OUT);
  gpio_put(ETH_PIN_RST, 0);
  sleep_ms(100);
  gpio_put(ETH_PIN_RST, 1);
  sleep_ms(100);
  bi_decl(bi_1pin_with_name(ETH_PIN_RST, "w5500 RESET"));

  // initialize module
  eth_deselect();
  do {
    uint8_t link_status = eth_read8(ETH_PHYCFGR);
    printf("link status: %u\n",link_status);
    if (link_status & 0x01) { // link on
      break;
    }
  } while (1);

  // check module
  uint8_t version = eth_read8(ETH_VERSIONR);
  if (version != 0x04) {
    printf("ACCESS ERR: VERSION != 0x04, read value = 0x02x\n",version);
    while (1);
  }

  /* eth_write8(ETH_MR, ETH_MR_RST); */
  /* for (int i = 0; i < 256; ++i) { */
  /*   eth_read8(ETH_MR); */
  /* } */
  // initialize network
  /* eth_write(ETH_SHAR, eth_shar, 9, true); */
  /* eth_write(ETH_GAR, eth_gar, 7, true); */
  /* eth_write(ETH_SUBR, eth_subr, 7, true); */
  /* eth_write(ETH_SIPR, eth_sipr, 7, true); */

  eth_write8(ETHS_TXBUF_SIZE(0),16);
  eth_write8(ETHS_RXBUF_SIZE(0),16);
  for (int i = 1; i < 8; ++i) {
    eth_write8(ETHS_TXBUF_SIZE(i),0);
    eth_write8(ETHS_RXBUF_SIZE(i),0);
  }

  eth_write(ETH_SUBR, eth_subr, 7, true);
  eth_write(ETH_SIPR, eth_sipr, 7, true);
  eth_write(ETH_SHAR, eth_shar, 9, true);
  eth_write(ETH_GAR, eth_gar, 7, true);


  // initialize interrupts
  gpio_set_dir(ETH_PIN_INT, GPIO_IN);
  eth_write16(ETH_INTLEVEL, 0);
  // disable non-socket interrupts
  //eth_write8(ETH_IMR, 0xff);
  //eth_write8(ETH_SIR, 0xff);
  // enable only socket 0 interrupts
  eth_write8(ETH_SIMR, 1);
  // enable only sendok|timeout|recv on socket 0
  eth_write8(ETHS_IMR(0), ETH_IR_SENDOK | ETH_IR_TIMEOUT | ETH_IR_RECV);
  // clear any socket 0 interrupts
  eth_write8(ETHS_IR(0), 0xff);

  tx_read_ptr = eth_read16(ETHS_RX_RD(0));
  tx_write_ptr = eth_read16(ETHS_TX_WR(0));
  /* uint8_t dest_mac[9] = { 0x00, 0x00, 0x00, 0xe4, 0x5f, 0x01, 0xbf, 0x86, 0x85 }; */
  /* eth_write(ETHS_DHAR(0), dest_mac, 9, true); */
}

static inline void spi_read(uint8_t* buf, uint16_t len) {
  uint8_t dummy_data = 0xff;
  channel_config_set_read_increment(&eth_dma_channel_config_tx, false);
  channel_config_set_write_increment(&eth_dma_channel_config_tx, false);
  dma_channel_configure(eth_dma_tx, &eth_dma_channel_config_tx,
			&spi_get_hw(ETH_SPI_PORT)->dr,
			&dummy_data,
			len,
			false);
  channel_config_set_read_increment(&eth_dma_channel_config_rx, false);
  channel_config_set_write_increment(&eth_dma_channel_config_rx, true);
  dma_channel_configure(eth_dma_rx, &eth_dma_channel_config_rx,
			buf,
			&spi_get_hw(ETH_SPI_PORT)->dr,
			len,
			false);
  dma_start_channel_mask((1u << eth_dma_tx) | (1u << eth_dma_rx));
}

static inline void spi_write(uint8_t* buf, uint16_t len) {
  uint8_t dummy_data;
  channel_config_set_read_increment(&eth_dma_channel_config_tx, true);
  channel_config_set_write_increment(&eth_dma_channel_config_tx, false);
  dma_channel_configure(eth_dma_tx, &eth_dma_channel_config_tx,
			&spi_get_hw(ETH_SPI_PORT)->dr,
			buf,
			len,
			false);
  channel_config_set_read_increment(&eth_dma_channel_config_rx, false);
  channel_config_set_write_increment(&eth_dma_channel_config_rx, false);
  dma_channel_configure(eth_dma_rx, &eth_dma_channel_config_rx,
			&dummy_data,
			&spi_get_hw(ETH_SPI_PORT)->dr,
			len,
			false);
  dma_start_channel_mask((1u << eth_dma_tx) | (1u << eth_dma_rx));
}

void eth_read(uint32_t addr, uint8_t* buf, uint16_t len, 
	      bool blocking) {
  uint8_t spi_data[3];

  critical_section_enter_blocking(&eth_cri_sec);
  eth_select();

  addr |= (_W5500_SPI_READ_ | _W5500_SPI_VDM_OP_);

  spi_data[0] = (addr & 0x00ff0000) >> 16;
  spi_data[1] = (addr & 0x0000ff00) >> 8;
  spi_data[2] = (addr & 0x000000ff);
  spi_write(spi_data, 3);
  dma_channel_wait_for_finish_blocking(eth_dma_rx);
  spi_read(buf, len);

  if (blocking) {
    dma_channel_wait_for_finish_blocking(eth_dma_rx);
    eth_deselect();
  } else {
    eth_cmd_state = ETH_RX_DMA;
  }
  critical_section_exit(&eth_cri_sec);
}

// this should be called with the first 3 bytes of buf reserved to put the
// address, in order to do the write as a single burst operation
void eth_write(uint32_t addr, uint8_t* buf, uint16_t len, bool blocking) {

  critical_section_enter_blocking(&eth_cri_sec);
  eth_select();
  
  addr |= (_W5500_SPI_WRITE_ | _W5500_SPI_VDM_OP_);

  buf[0] = (addr & 0x00ff0000) >> 16;
  buf[1] = (addr & 0x0000ff00) >> 8;
  buf[2] = (addr & 0x000000ff);
  spi_write(buf, len);

  if (blocking) {
    dma_channel_wait_for_finish_blocking(eth_dma_rx);
    eth_deselect();
  } else {
    eth_cmd_state = ETH_TX_DMA;
  }
  critical_section_exit(&eth_cri_sec);
}

bool is_wizchip_busy() {
  /* if (prev_eth_cmd_state != eth_cmd_state) { */
  /*   prev_eth_cmd_state = eth_cmd_state; */
  /*   print_eth_cmd_state(); */
  /* } */
  if (eth_cmd_state == ETH_IDLE) {
    return false;
  }
  if (dma_channel_is_busy(eth_dma_rx) || dma_channel_is_busy(eth_dma_tx)) {
    //printf("busy\n");
    return true;
  }
  if (eth_cmd_state == ETH_TX_DMA) {
    eth_deselect();
    eth_write16(ETHS_TX_WR(0), tx_write_ptr);
    eth_write8(ETHS_CR(0), ETH_CR_SEND);
    eth_cmd_state = ETH_TX_SEND;
  } else if (eth_cmd_state == ETH_RX_DMA) {
    printf("completed rx dma\n");
    eth_deselect();
    eth_write16(ETHS_RX_RD(0), tx_read_ptr);
    eth_write8(ETHS_CR(0), ETH_CR_RECV);
    eth_cmd_state = ETH_RX_RECV;
  }
  if (eth_cmd_state == ETH_TX_SEND ||
      eth_cmd_state == ETH_RX_RECV) {
    uint8_t tmp = eth_read8(ETHS_CR(0));
    //printf("checking cmd state: %u\n", tmp);
    if (tmp == 0) {
      if (eth_cmd_state == ETH_TX_SEND) {
	eth_cmd_state = ETH_TX_WAIT;
      } else {
	eth_cmd_state = ETH_RX_WAIT;
      }
    }
  }
  if (gpio_get(ETH_PIN_INT) == 0) {
    //printf("checking interrupt\n");
    uint8_t tmp = eth_read8(ETHS_IR(0));
    //printf("interrupt %d\n",tmp);
    eth_write8(ETHS_IR(0),tmp);
    if (tmp & ETH_IR_SENDOK) {
      if (eth_cmd_state != ETH_TX_WAIT) {
	printf("got SENDOK with no active write\n");
	while(1);
      }
      eth_cmd_state = ETH_IDLE;
      return false;
    }
    if (tmp & ETH_IR_RECV) {
      if (eth_cmd_state != ETH_RX_DMA) {
    	printf("got RECV with no active read\n");
    	while(1);
      }
      eth_cmd_state = ETH_IDLE;
      return false;
    }
    if (tmp & ETH_IR_TIMEOUT) {
      printf("socket timeout\n");
      uint8_t mac_buf[6];
      eth_read(ETHS_DHAR(0), mac_buf, 6, true);
      while(1);
    }
  }
  /* } else { */
  /*   printf("no interrupt\n"); */
  /* } */
  uint8_t sock_state = eth_read8(ETHS_SR(0));
  if (sock_state == ETH_SOCK_CLOSED) {
    printf("socket closed\n");
    while(1);
  }
  return true;
}

// two-byte register reads must be done (at least) twice,
// to ensure that they were not interrupted by an update
// on the module side
uint16_t eth_read16(uint32_t reg) {
  uint8_t buf[2];
  uint16_t ret = 0;
  uint16_t check_ret = 0;
  do {
    eth_read(reg, buf, 2, true);
    ret = ((uint16_t)(buf[0] << 8)) + buf[1];
    eth_read(reg, buf, 2, true);
    check_ret = ((uint16_t)(buf[0] << 8)) + buf[1];
  } while (ret != check_ret);
  return ret;
}

void eth_write16(uint32_t reg, uint16_t val) {
  uint8_t buf[5];
  buf[3] = (uint8_t)(val >> 8);
  buf[4] = (uint8_t)val;
  eth_write(reg, buf, 5, true);
}

uint8_t eth_read8(uint32_t reg) {
  uint8_t ret;
  eth_read(reg, &ret, 1, true);
  return ret;
}

void eth_write8(uint32_t reg, uint8_t val) {
  uint8_t buf[4];
  buf[3] = val;
  eth_write(reg, buf, 4, true);
}

void eth_read_packet(uint8_t* buf, uint16_t len) {

}

void eth_write_packet(uint8_t* buf, uint16_t len) {
  //printf("writing packet %d\n",len);

  tx_write_ptr = eth_read16(ETHS_TX_WR(0));
  //printf("tx_write_ptr: %u\n",tx_write_ptr);
  uint32_t addr = ((uint32_t)tx_write_ptr << 8) + (WIZCHIP_TXBUF_BLOCK(0) << 3);

  /* for (int i = 0; i < len; ++i) { */
  /*   printf("%02x ", buf[i]); */
  /* } */
  /* printf("\n"); */
  eth_write(addr, buf, len, false);
  tx_write_ptr += (len - 3);
  eth_cmd_state = ETH_TX_DMA;
  /* eth_write16(ETHS_TX_WR(0), tx_write_ptr); */
  /* eth_write8(ETHS_CR(0), ETH_CR_SEND); */
  /* eth_cmd_state = ETH_TX_SEND; */
}
