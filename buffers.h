#pragma once

#include <pico/util/queue.h>
#include <pico/sync.h>

extern int bus_spinlock;
extern queue_t raw_bus_queue;
extern uint8_t bus_buffers[8][2048];
