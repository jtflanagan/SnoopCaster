#include "buffers.h"

int bus_spinlock;
queue_t raw_bus_queue;
uint8_t bus_buffers[8][2048];
