#pragma once
#include <nuttx/wqueue.h>
#include <stdint.h>

typedef void (*wq_cb_t)(void *arg);

struct wq_item {
  struct work_s work;
  wq_cb_t cb;
  void *arg;
  uint32_t period_ms;
};

int wq_schedule_periodic(struct wq_item *item, wq_cb_t cb, void *arg, uint32_t period_ms);
void wq_cancel(struct wq_item *item);