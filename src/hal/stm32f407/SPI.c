#include "../../communication/SPI.h"
#include "../../globals.h"

#include "../../hal/hw_specific.h"
#include "../../hw_init.h"
#include "../../krnl/thread.h"
#include "../../utils/queue.h"
#include "stm32f407xx.h"
#include <stdint.h>

// SPI queue
  T ** buffer;
  size_t size;
  public:
  volatile uint8_t head;
  volatile T * head_obj;
  volatile uint8_t tail;
  void init(T ** buffer, size_t size) {
    this->buffer = buffer;
    this->size = size;
    this->head = 0;
    this->tail = 0;
  }

  int enqueue(T * element) {
    if (is_full()) return -1;
    buffer[head] = element;
    head = (head + 1) % size;
    return 0;
  }

  int dequeue(T * ret) {
    if (is_empty()) return -1;
    ret = buffer[tail];
    tail = (tail + 1) % size;
    return 0;
  }

  int is_empty() { return head == tail; };
  int is_full() { return (head + 1) % size == tail; };

// interrupt stuff
uint8_t dma_done = 1;
void dma2_stream3_handler() {
    if (DMA2->LISR & DMA_LISR_TCIF3) {
        DMA2->LIFCR |= DMA_LIFCR_CTCIF3;
    }
    if (DMA2->LISR & DMA_LISR_HTIF3) {
        DMA2->LIFCR |= DMA_LIFCR_CHTIF3;
    }
}

void dma2_stream0_handler() {
    volatile uint32_t state = DMA2->LISR;
    if (DMA2->LISR & DMA_LISR_TCIF0) {
        DMA2->LIFCR |= DMA_LIFCR_CTCIF3;
        DMA2->LIFCR |= DMA_LIFCR_CHTIF3;
        DMA2->LIFCR |= DMA_LIFCR_CTCIF0;
        DMA2->LIFCR |= DMA_LIFCR_CHTIF0;
        DMA2_Stream3->CR &= ~DMA_SxCR_EN;
        DMA2_Stream0->CR &= ~DMA_SxCR_EN;
        OS_SPI_manager.queue.head_obj->cs_high();
        OS_SPI_manager.queue.head_obj->state = SPI_state::done;
        //// os_printf("res: %d \n", current->rx_buffer[1]);
        // dma_done = 1;
    }
}

void DMA2_init() {
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    //// tx
    // DMA2_Stream3->CR = 0;
    // DMA2_Stream3->PAR = (uint32_t)&SPI1->DR;
    // DMA2_Stream3->CR |= (0b011 << DMA_SxCR_CHSEL_Pos);
    // DMA2_Stream3->CR |= (1 << DMA_SxCR_MINC_Pos);
    // DMA2_Stream3->CR |= (0b01 << DMA_SxCR_DIR_Pos);
    // DMA2_Stream3->CR |= (1 << DMA_SxCR_TCIE_Pos);
    // DMA2_Stream3->CR |= (1 << DMA_SxCR_PL_Pos);

    //// rx
    // DMA2_Stream0->CR = 0;
    // DMA2_Stream0->PAR = (uint32_t)&SPI1->DR;
    // DMA2_Stream0->CR |= (0b11 << DMA_SxCR_CHSEL_Pos);
    // DMA2_Stream0->CR |= (1 << DMA_SxCR_MINC_Pos);
    // DMA2_Stream0->CR &= ~(0b011 << DMA_SxCR_DIR_Pos);
    // DMA2_Stream0->CR |= (1 << DMA_SxCR_TCIE_Pos);
    // DMA2_Stream0->CR |= (1 << DMA_SxCR_PL_Pos);
    // DMA2_Stream0->FCR |= DMA_SxFCR_DMDIS;

    //// NVIC_EnableIRQ(DMA2_Stream3_IRQn);
    // NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    //// NVIC_SetPriority(DMA2_Stream3_IRQn, 11);
    // NVIC_SetPriority(DMA2_Stream0_IRQn, 10);
}

volatile SPI_transmition *SPI_buffer[SPI_buffer_lenght];
void SPI_manager::init() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    GPIOA->MODER |= 0b01 << (4 * 2); // general purpose output
    GPIOA->MODER |= 0b01 << (1 * 2); // general purpose output
    SPI1->CR1 &= ~SPI_CR1_SPE;

    GPIOA->MODER |= (2 << (5 * 2)) | (2 << (6 * 2)) | (2 << (7 * 2));   // af mode
    GPIOA->OSPEEDR |= (3 << (5 * 2)) | (3 << (6 * 2)) | (3 << (7 * 2)); // High speed
    GPIOA->AFR[0] |= (5 << (5 * 4)) | (5 << (6 * 4)) | (5 << (7 * 4));  // af 5
    SPI1->CR1 = 0;

    SPI1->CR1 = SPI_CR1_MSTR  // Master
                | SPI_CR1_SSM // Software CS management
                | SPI_CR1_SSI // Set nss high
                | (3 << 3);   // fPCLK/8

    SPI1->CR1 |= SPI_CR1_SPE;
    SPI1->CR2 = SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;

    DMA2_init();

    this->queue.init(SPI_buffer, SPI_buffer_lenght);
}

int SPI_manager::submit(SPI_transmition *tx) {
    tx->state = SPI_state::pending;
    return this->queue.enqueue(tx);
}

void SPI_manager::handle() {
    //// called during transmition or error
    // if (this->queue.head_obj->state == SPI_state::busy) {
    return;
}
else if (this->queue.head_obj->state == SPI_state::error) {
    //// error routine
    //}

    //// called when SPI is inactive
    // volatile SPI_transmition *next;
    if (this->queue.head_obj->state == SPI_state::pending) {
        next = this->queue.head_obj;
        next->state = SPI_state::busy;
    } else if (this->queue.head_obj->state == SPI_state::done) {
        int ret = this->queue.dequeue(next);
        if (ret == -1)
            return;
    }

    //// start next transmition
    // DMA2_Stream3->CR &= ~DMA_SxCR_EN;
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    uint8_t dma_done = 0;

    DMA2_Stream3->M0AR = (uint32_t)next->tx_buffer;
    DMA2_Stream3->NDTR = next->size;

    DMA2_Stream0->M0AR = (uint32_t)next->rx_buffer;
    DMA2_Stream0->NDTR = next->size;

    next->cs_low();
    DMA2_Stream3->CR |= DMA_SxCR_EN;
    DMA2_Stream0->CR |= DMA_SxCR_EN;
    return;
}
