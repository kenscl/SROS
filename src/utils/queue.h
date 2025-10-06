#pragma once
#include <stddef.h>

//template <typename T> class Queue {
//  private:
//  T ** buffer;
//  size_t size;
//  public:
//  volatile uint8_t head;
//  volatile T * head_obj;
//  volatile uint8_t tail;
//  void init(T ** buffer, size_t size) {
//    this->buffer = buffer;
//    this->size = size;
//    this->head = 0;
//    this->tail = 0;
//  }
//
//  int enqueue(T * element) {
//    if (is_full()) return -1;
//    buffer[head] = element;
//    head = (head + 1) % size;
//    return 0;
//  }
//
//  int dequeue(T * ret) {
//    if (is_empty()) return -1;
//    ret = buffer[tail];
//    tail = (tail + 1) % size;
//    return 0;
//  }
//
//  int is_empty() { return head == tail; };
//  int is_full() { return (head + 1) % size == tail; };
/};
