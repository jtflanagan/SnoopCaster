#include "eth.h"
#include "buffers.h"
#include <stdio.h>
#include <string.h>


static critical_section_t eth_cri_sec;
static uint eth_dma_tx;
static uint eth_dma_rx;
static dma_channel_config eth_dma_channel_config_tx;
static dma_channel_config eth_dma_channel_config_rx;
// these are all declared with 3 bytes space at the start, for passing into
// the eth_write function which puts the module dest address there
static uint8_t eth_shar[9] = {0, 0, 0, 0x00, 0x08, 0xDC, 0x12, 0x34, 0x56};
static uint8_t eth_sipr[7] = {0, 0, 0, 192, 168, 1, 253};
static uint8_t eth_gar[7] = {0, 0, 0, 192, 168, 1, 1};
static uint8_t eth_subr[7] = {0, 0, 0, 255, 255, 255, 0};

struct ring_entry {
  uint16_t begin;
  uint16_t length;
};

static struct ring_entry tx_ring[8];
static uint32_t ring_end = 0;
static uint32_t ring_begin = 0;
static uint8_t msg_buf[1475];
static uint8_t* mbe;
static uint8_t rx_header_buf[8];
static uint8_t rx_buf[1475];
static uint8_t* rxe;
static bool dma_writing = false;
static bool dma_reading = false;
static bool tx_idle = true;
static uint16_t rx_pending_bytes = 0;
static uint16_t rx_len = 0;
static uint16_t tx_ptr = 0;
static uint16_t rx_ptr = 0;
static uint32_t bus_count = 0;
//static uint64_t dma_begin_time;

static uint32_t vals[256];
static uint32_t prev_address = 0;
static uint8_t host_ip[7] = {0, 0, 0, 192, 168, 1, 107};
static uint16_t host_port = 5000;
static uint64_t next_connect_attempt = 0;
static uint16_t listen_port = 5000;
static uint32_t next_tx_seqno = 0;
static uint32_t last_rx_seqno = 0;

bool tx_ring_full() {
  return (((ring_end + 1) % 8) == ring_begin);
}

static inline void eth_select(void) {
  gpio_put(ETH_PIN_CS, 0);
}

static inline void eth_deselect(void) {
  gpio_put(ETH_PIN_CS, 1);
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
  }
  critical_section_exit(&eth_cri_sec);
}

void load_tx_bytes() {
  dma_writing = true;
  uint16_t len = mbe - msg_buf;
  tx_ring[ring_end].length += len - 3;
  /* printf("load_tx_bytes %d %d %d %d %d\n", */
  /* 	 len, ring_end, tx_ring[ring_end].begin, tx_ring[ring_end].length, tx_ptr); */
  uint32_t addr = ((uint32_t)tx_ptr << 8) + (WIZCHIP_TXBUF_BLOCK(0) << 3);
  //dma_begin_time = time_us_64();
  eth_write(addr, msg_buf, len, true);
  tx_ptr += len - 3;
}

void process_echo() {
  printf("echo %d\n",rx_len);
  for (uint16_t i = rxe - rx_buf; i < rx_len; ++i) {
    *mbe++ = *rxe++;
  }
  load_tx_bytes();
}

void process_rx() {
  rxe = rx_buf;
  last_rx_seqno = *rxe++;
  last_rx_seqno += ((uint32_t)*rxe++) << 8;
  last_rx_seqno += ((uint32_t)*rxe++) << 16;
  last_rx_seqno += ((uint32_t)*rxe++) << 24;
  uint8_t cmd_type = *rxe++;
  mbe = msg_buf + 3;
  *mbe++ = last_rx_seqno & 0xff;
  *mbe++ = (last_rx_seqno >> 8) & 0xff;
  *mbe++ = (last_rx_seqno >> 16) & 0xff;
  *mbe++ = (last_rx_seqno >> 24) & 0xff;
  *mbe++ = cmd_type;
  printf("process_rx %d %d\n",last_rx_seqno, cmd_type);
  switch (cmd_type) {
  case 1:
    process_echo(); break;
  default:
    printf("unrecognized cmd type %d\n",cmd_type);
  }
  eth_write8(ETHS_RX_RD(0), rx_ptr);
  rx_len = 0;
}

bool is_dma_busy() {
  if (dma_writing || dma_reading) {
    if (dma_channel_is_busy(eth_dma_rx) || dma_channel_is_busy(eth_dma_tx)) {
      //printf("busy\n");
      return true;
    }
    // deselect to end the operation
    eth_deselect();
    if (dma_reading) {
      // read complete
      dma_reading = false;
    } else {
      // write complete
      dma_writing = false;
    }
  }
  return false;
}

void terminate_current_packet() {
  //printf("terminating packet\n");
  uint16_t cur_packet_end =
    tx_ring[ring_end].begin + tx_ring[ring_end].length;
  ring_end = (ring_end + 1) % 8;
  tx_ring[ring_end].begin = cur_packet_end;
  tx_ring[ring_end].length = 0;
}

void send_tx_packet() {
  //printf("sending tx packet %d\n", ring_begin);
  uint16_t pkt_end = tx_ring[ring_begin].begin + tx_ring[ring_begin].length;
  // adjust tx write pointer, hit send, and wait for command complete
  eth_write16(ETHS_TX_WR(0), pkt_end);
  eth_write8(ETHS_CR(0), ETH_CR_SEND);
  while (eth_read8(ETHS_CR(0)));
  tx_idle = false;
  ring_begin = (ring_begin + 1) % 8;
}

void start_rx_packet() {
  printf("start_rx_packet %d\n", rx_pending_bytes);
  //dma_reading = true;
  uint32_t addr = ((uint32_t)rx_ptr << 8) + (WIZCHIP_RXBUF_BLOCK(0) << 3);
  eth_read(addr, rx_header_buf, 8, true);
  rx_ptr += 8;
  rx_pending_bytes -= 8;
  eth_write16(ETHS_RX_RD(0),rx_ptr);
  // hit read, and wait for command complete
  eth_write8(ETHS_CR(0), ETH_CR_RECV);
  while(eth_read8(ETHS_CR(0)));
  rx_len = rx_header_buf[6];
  rx_len = (rx_len << 8) + rx_header_buf[7];
  addr = ((uint32_t)rx_ptr << 8) + (WIZCHIP_RXBUF_BLOCK(0) << 3);
  eth_read(addr, rx_buf, rx_len, true);
  rx_ptr += rx_len;
  rx_pending_bytes -= rx_len;
  eth_write16(ETHS_RX_RD(0),rx_ptr);
}

bool check_interrupts() {
  /* if (rx_pending_bytes > 0 && !rx_len) { */
  /*   // we have pending bytes from a previous RX and we have responded */
  /*   // to the last one.  Start another RX read (and return true because */
  /*   // this will start DMA and we're done on this pass) */
  /*   start_rx_packet(); */
  /*   return true; */
  /* } */
  if (gpio_get(ETH_PIN_INT) == 1) {
    // no interrupt, can check for other tasks
    //printf("no interrupt\n");
    return false;
  } else {
    //printf("interrupt\n");
  }
  uint8_t tmp = eth_read8(ETHS_IR(0));
  if (tmp & ETH_IR_TIMEOUT) {
    printf("timeout\n");
    while(1);
  }
  if (tmp & ETH_IR_RECV) {
    printf("IR_RECV\n");
    // module wants to tell us about recv bytes available
    eth_write8(ETHS_IR(0), ETH_IR_RECV);
    rx_pending_bytes = eth_read16(ETHS_RX_RSR(0));
    start_rx_packet();
  }
  if (tmp & ETH_IR_SENDOK) {
    //printf("IR_SENDOK\n");
    // last packet tx completed, we are ready to start the next when we can
    tx_idle = true;
  }
  // clear whatever interrupts happened
  eth_write8(ETHS_IR(0),tmp);
  // interrupts acknowledged, can check for other tasks
  return false;
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

static inline void time_loop(const char* buf, uint64_t b) {
  //return;
  uint64_t e = time_us_64();
  int32_t diff = e - b;
  printf("loop: %s %d\n",buf,diff);
  //printf("wtf\n");
}

void eth_init() {
  // initialize spi
  uint eth_lock = spin_lock_claim_unused(true);
  printf ("eth lock %d\n",eth_lock);
  critical_section_init_with_lock_num(&eth_cri_sec, eth_lock);
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
  eth_write8(ETH_IMR, 0xff);
  eth_write8(ETH_SIR, 0xff);
  // enable only socket 0 interrupts
  eth_write8(ETH_SIMR, 1);
  // enable only sendok|timeout|recv on socket 0
  eth_write8(ETHS_IMR(0), ETH_IR_SENDOK | ETH_IR_TIMEOUT | ETH_IR_RECV);
  // clear any socket 0 interrupts
  eth_write8(ETHS_IR(0), 0xff);

  tx_ptr = eth_read16(ETHS_RX_RD(0));
  rx_ptr = eth_read16(ETHS_TX_WR(0));
  tx_ring[ring_end].begin = tx_ptr;
  tx_ring[ring_end].length = 0;
}



void eth_loop() {
  while (1) {
    uint8_t sock_state = eth_read8(ETHS_SR(0));
    if (sock_state == ETH_SOCK_UDP) {
      printf("sock connected\n");
      break;
    }
    if (sock_state == ETH_SOCK_CLOSED) {
      uint64_t now = time_us_64();
      if (now < next_connect_attempt) {
	continue;
      }
      next_connect_attempt = now + 5*1000*1000;
      // close socket
      eth_write8(ETHS_CR(0), ETH_CR_CLOSE);
      while (eth_read8(ETHS_CR(0)));
      eth_write8(ETHS_IR(0),0xff);
      while(eth_read8(ETHS_SR(0) != ETH_SOCK_CLOSED));
      // open socket with UDP
      eth_write8(ETHS_MR(0), ETH_SOCK_UDP);
      uint8_t port_buf[5];
      port_buf[3] = (listen_port & 0xff00) >> 8;
      port_buf[4] = (listen_port & 0xff);
      eth_write(ETHS_PORT(0), port_buf, 5, true);
      eth_write8(ETHS_CR(0), ETH_CR_OPEN);
      while (eth_read8(ETHS_CR(0)));
      while (eth_read8(ETHS_SR(0) == ETH_SOCK_CLOSED));	     
      eth_write(ETHS_DIPR(0), host_ip, 7, true);
      port_buf[3] = (host_port & 0xff00) >> 8;
      port_buf[4] = (host_port & 0xff);
      eth_write(ETHS_DPORT(0), port_buf, 5, true);
      uint8_t buf[32];
      eth_read(ETH_SHAR, buf, 6, true);
      printf("mac %02x %02x %02x %02x %02x %02x\n",
	     buf[0],buf[1],buf[2],
	     buf[3],buf[4],buf[5]);
      eth_read(ETH_SIPR, buf, 4, true);
      printf("ip %02x %02x %02x %02x\n",
	     buf[0],buf[1],buf[2],buf[3]);
      eth_read(ETH_GAR, buf, 4, true);
      printf("gate %02x %02x %02x %02x\n",
	     buf[0],buf[1],buf[2],buf[3]);
      eth_read(ETH_SUBR, buf, 4, true);
      printf("subnet %02x %02x %02x %02x\n",
	     buf[0],buf[1],buf[2],buf[3]);
      printf("socket created\n");
      break;
    }
  }
  while(1) {
    // if there is active dma, nothing else can happen besides trying to pull
    // some bus data
    if (is_dma_busy()) {
      /* uint queue_size = queue_get_level_unsafe(&raw_bus_queue); */
      /* uint count = queue_size; */
      /* if (count > 8 - bus_count) { */
      /* 	count = 8 - bus_count; */
      /* } */
      /* for (int i = 0; i < count; ++i) { */
      bool have_bus_data = 
	queue_try_remove(&raw_bus_queue, &vals + bus_count * 32);
      if (have_bus_data) {
	++bus_count;
      }
      /* } */
      //printf("dma_busy\n");
      continue;
    }
    // if we process an interrupt, nothing else can happen
    if (check_interrupts()) {
      //printf("interrupt\n");
      continue;
    }
    if (rx_len) {
      printf("rx_len %d\n", rx_len);
      if (tx_ring[ring_end].length > 0) {
	if (tx_ring_full()) {
	  // if we have a pending response but tx ring is full, we can't do
	  // anything till there is space
	  continue;
	}
	// terminate the current packet so the response can be next
	terminate_current_packet();
      }
      // there is room in the tx ring for the response, queue it
      process_rx();
      continue;
    }
    // if tx is idle and there is a ready packet, push it out
    if (tx_idle) {
      //printf("tx_idle\n");
      // if there is more than one packet, push out the next packet
      if (ring_begin != ring_end) {
	//printf("ring buffer has multiple packets\n");
	send_tx_packet();
	continue;
      }
      // if there is only one packet and it is not empty, terminate it
      // and push it out
      if (tx_ring[ring_end].length > 1472-26*4) {
	//printf("ring buffer has one packet\n");
	terminate_current_packet();
	send_tx_packet();
	continue;
      }
      // otherwise there's nothing to send, check to add
    }
    if (bus_count < 4) {
      uint queue_size = queue_get_level_unsafe(&raw_bus_queue);
      uint count = queue_size;
      if (count > 4 - bus_count) {
	count = 4 - bus_count;
      }
      for (int i = 0; i < count; ++i) {
	//printf("try %d %d %d\n",i, count, queue_size);
	bool have_bus_data = 
	  queue_try_remove(&raw_bus_queue, &vals + bus_count * 32);
	if (have_bus_data) {
	  ++bus_count;
	} else {
	  break;
	}
      } 
    }
    if (bus_count == 0) {
      continue;
    }
    //printf("done checking\n");
    // if the ring is completely full (full ring and last packet
    // does not have enough room for another chunk), then we have to wait
    if (tx_ring_full() && tx_ring[ring_end].length > 1472 - 26*bus_count) {
      printf("ring buffer completely full\n");
      continue;
    }
    mbe = msg_buf + 3;
    if (tx_ring[ring_end].length == 0) {
      // packet is empty, need header
      ++next_tx_seqno;
      *mbe++ = next_tx_seqno & 0xff;
      *mbe++ = (next_tx_seqno >> 8) & 0xff;
      *mbe++ = (next_tx_seqno >> 16) & 0xff;
      *mbe++ = (next_tx_seqno >> 24) & 0xff;
      // 0 is packet type of "bus data"
      *mbe++ = 0;
    }
    uint16_t space_left = 1472-tx_ring[ring_end].length;
    uint32_t k = 0;
    while (k < bus_count && space_left >= 26*4) {
      uint8_t* b = mbe;
      for (uint32_t j = 0; j < 4; ++j) { 
	uint8_t* rw_flags = mbe++;
	uint8_t* seq_flags = mbe++;
	uint8_t* data_p = mbe;
	mbe += 8;
	*rw_flags = 0;
	*seq_flags = 0;
	for (uint8_t i = 0; i < 8; ++i) {
	  uint8_t data = vals[j*8+i] & 0xff;
	  uint8_t rw = (vals[j*8+i] >> 9) & 0x1;
	  uint8_t address = (vals[j*8+i] >> 10) & 0xffff;
	  *rw_flags |= rw << i;
	  *data_p++ = data;
	  if (address != prev_address + 1) {
	    *seq_flags |= 1 << i;
	    uint8_t address_lo = address & 0xff;
	    *mbe++ = address_lo;
	    uint8_t address_hi = (address >> 8) & 0xff;
	    *mbe++ = address_hi;
	  }
	  prev_address = address;
	}
      }
      k++;
      space_left -= (mbe - b);      
    }
    //printf("%d %d\n",k,space_left);
    load_tx_bytes();
    if (space_left < 26*4) {
      terminate_current_packet();
    }
    if (k < bus_count) {
      memmove(&vals, &vals + k*32, (bus_count-k)*128); 
    }
    bus_count -= k;
  }
} 
