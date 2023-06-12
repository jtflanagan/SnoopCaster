#pragma once

#include <pico/util/queue.h>
#include <pico/sync.h>

extern int bus_spinlock;
extern queue_t raw_bus_queue;
