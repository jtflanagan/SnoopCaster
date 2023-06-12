#include "eth.h"
#include "eth_internal.h"
#include "buffers.h"
#include <stdio.h>

static uint8_t tx_bufs[2][2048];
static uint8_t active_tx_buf_flag = 0;
static uint8_t* active_tx_buf = tx_bufs[0];
static uint8_t* txe = tx_bufs[0];
static uint32_t vals[64];
static uint32_t prev_address = 0;
static uint8_t host_ip[7] = {0, 0, 0, 192, 168, 1, 107};
static uint16_t host_port = 5000;
static uint64_t next_connect_attempt = 0;
static uint16_t listen_port = 5000;
static bool tx_msg_pending = false;
static bool tx_buf_full = false;
static uint32_t next_tx_seqno = 0;
static uint32_t last_rx_seqno = 0;


static void check_tx() {
  if (!tx_msg_pending) {
    return;
  }

  //printf("sending packet\n");
  uint32_t tx_size = txe - active_tx_buf;
  //printf("tx size:%u\n",tx_size);
  eth_write_packet(active_tx_buf, tx_size);
  //printf("sent packet %d\n", active_tx_buf_flag);
  active_tx_buf_flag = !active_tx_buf_flag;
  active_tx_buf = tx_bufs[active_tx_buf_flag];
  txe = active_tx_buf;
  tx_buf_full = false;
  tx_msg_pending = false;
}

static void prepare_tx_data() {
  if (tx_buf_full) {
    //printf("buffer full\n");
    return;
  }
  bool have_bus_data = queue_try_remove(&raw_bus_queue, &vals);
  if (!have_bus_data) {
    //printf("no bus data\n");
    return;
  }
  if (!tx_msg_pending) {
    //printf("starting packet\n");
    tx_msg_pending = true;
    ++next_tx_seqno;
    txe += 3; // leave space for module address
    *txe++ = next_tx_seqno & 0xff;
    *txe++ = (next_tx_seqno >> 8) & 0xff;
    *txe++ = (next_tx_seqno >> 16) & 0xff;
    *txe++ = (next_tx_seqno >> 24) & 0xff;
    *txe++ = last_rx_seqno & 0xff;
    *txe++ = (last_rx_seqno >> 8) & 0xff;
    *txe++ = (last_rx_seqno >> 16) & 0xff;
    *txe++ = (last_rx_seqno >> 24) & 0xff;
    *txe++ = 0;
    //printf("seqs %u %u %llp %llp\n",next_tx_seqno, last_rx_seqno, txe, active_tx_buf);
  }
  // not implementing non-bus data yet
  //printf("adding bus data\n");
  do {
    for (int j = 0; j < 8; ++j) {
      uint8_t* rw_flags = txe++;
      uint8_t* seq_flags = txe++;
      uint8_t* data_p = txe;
      txe += 8;
      for (uint8_t i = 0; i < 8; ++i) {
	uint8_t data = vals[j*8 + i] & 0xff;
	uint8_t rw = (vals[j*8 + i] >> 9) & 0x1;
	uint8_t address = (vals[j*8 + i] >> 10) & 0xffff;
	//printf("%u %u\n",data, address);
	*rw_flags |= rw << i;
	*data_p++ = data;
	if (address != prev_address + 1) {
	  *seq_flags |= 1 << i;
	  uint8_t address_lo = address & 0xff;
	  *txe++ = address_lo;
	  uint8_t address_hi = (address >> 8) & 0xff;
	  *txe++ = address_hi;
	}
	prev_address = address;
      }
    }
    if ((txe - active_tx_buf) < 1460 - 208) { 
      // not enough room for another bus data chunk on this packet, 
      // set tx_buf_full and break out
      tx_buf_full = true;
      break;
    }
    have_bus_data = queue_try_remove(&raw_bus_queue, &vals);
  } while (have_bus_data);
  /* printf("txe:"); */
  /* for (int i = 0; i < (txe - active_tx_buf); ++i) { */
  /*   printf("%02x ", active_tx_buf[i]); */
  /* } */
  /* printf("\n"); */
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
    prepare_tx_data();
    if (is_wizchip_busy()) {
      // if the chip is busy, nothing else to do
      continue;
    }
    uint8_t sock_state = eth_read8(ETHS_SR(0));
    if (sock_state == ETH_SOCK_CLOSED) {
      printf("socket closed\n");
      while(1);
    }

    check_tx();
  }
}
