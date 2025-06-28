#include "../../communication/SPI.h"

#include <cstdint>
#include <stdint.h>
#include "../../hw_init.h"
#include "../../krnl/thread.h"
#include "../../hal/hw_specific.h"
#include "stm32f407xx.h"

#define SPI_QUEUE_LENGTH 20
SPI_information * queue[SPI_QUEUE_LENGTH] = {};
SPI_information * current = 0;
uint16_t queue_head = 0;
SPI_information * empty;
uint8_t dma_done = 1;
uint8_t enqueue(SPI_information *info) {
  for (int i = queue_head; i < SPI_QUEUE_LENGTH + queue_head; ++i) {
    if (i < SPI_QUEUE_LENGTH) {
      if (queue[i]->size == 0) {
        queue[i] = info;
        return 1;
      }
    } else {
      if (queue[i - SPI_QUEUE_LENGTH]->size == 0) {
        queue[i - SPI_QUEUE_LENGTH] = info;
        return 1;
      }
    }
  }
  return 0;
}

SPI_information *dequeue() {
  if (queue[queue_head]->size != 0) {
    SPI_information *ret = queue[queue_head];
    queue[queue_head] = empty;
    if (queue_head < SPI_QUEUE_LENGTH - 1) {
      queue_head++;
    } else {
      queue_head = 0;
    }
    return ret;
  } else
    return nullptr;
}

#pragma GCC push_options
#pragma GCC optimize ("O0")
void SPI_send_next() {
    if (queue[queue_head]->size != 0) {
      if (current->state == Done) {
	   current = dequeue();

	   if (current->state == Done) {
	     current = empty;
	     return;
	   }
      }
    } else {
	if (current->state == Done) {
	    current = empty;
	    return;
	}
    }
}
#pragma GCC pop_options

volatile void SPI_thread() {
    while (1) {
      os_interrupt_disable();
      volatile uint32_t state = DMA2->LISR;
      volatile uint32_t state2 = DMA2->HISR;
      if (dma_done)
	SPI_send_next();
      os_interrupt_enable();
      yield();
    }
}

// SPI
uint8_t SPI_handle(SPI_information *info) {
  return enqueue(info);
}

void CS_A_H() {
    GPIOA->BSRR = GPIO_BSRR_BS_4;
}

void CS_A_L() {
    GPIOA->BSRR = GPIO_BSRR_BR_4;
}


void CS_M_H() {
    GPIOA->BSRR = GPIO_BSRR_BS_1;
}

void CS_M_L() {
    GPIOA->BSRR = GPIO_BSRR_BR_1;
}

void DMA2_init() {
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    // tx
    DMA2_Stream3->CR = 0;
    DMA2_Stream3->PAR = (uint32_t) &SPI1->DR;
    DMA2_Stream3->CR |= (0b011 << DMA_SxCR_CHSEL_Pos);
    DMA2_Stream3->CR |= (1 << DMA_SxCR_MINC_Pos);
    DMA2_Stream3->CR |= (0b01 << DMA_SxCR_DIR_Pos);
    DMA2_Stream3->CR |= (1 << DMA_SxCR_TCIE_Pos);
    DMA2_Stream3->CR |= (1 << DMA_SxCR_PL_Pos);

    //rx 
    DMA2_Stream0->CR = 0;
    DMA2_Stream0->PAR = (uint32_t) &SPI1->DR;
    DMA2_Stream0->CR |= (0b11 << DMA_SxCR_CHSEL_Pos);
    DMA2_Stream0->CR |= (1 << DMA_SxCR_MINC_Pos);
    DMA2_Stream0->CR &= ~(0b011 << DMA_SxCR_DIR_Pos);
    DMA2_Stream0->CR |= (1 << DMA_SxCR_TCIE_Pos);
    DMA2_Stream0->CR |= (1 << DMA_SxCR_PL_Pos);
    DMA2_Stream0->FCR |= DMA_SxFCR_DMDIS;

    //NVIC_EnableIRQ(DMA2_Stream3_IRQn);
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    //NVIC_SetPriority(DMA2_Stream3_IRQn, 11);
    NVIC_SetPriority(DMA2_Stream0_IRQn, 10);
}

static SPI_information dmy = {
    .state = Done,
    .size = 0,
    .rx_buffer = nullptr,
    .tx_buffer = nullptr,
    .target = Accelerometer,
    .adress = 0
};

void SPI_init() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; 
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; 

    GPIOA->MODER |= 0b01 << (4 * 2); // general purpose output 
    GPIOA->MODER |= 0b01 << (1 * 2); // general purpose output 
    CS_A_H();
    CS_M_H();
    SPI1->CR1 &= ~SPI_CR1_SPE;  
    
    GPIOA->MODER |= (2 << (5 * 2)) | (2 << (6 * 2)) | (2 << (7 * 2)); // af mode
    GPIOA->OSPEEDR |= (3 << (5 * 2)) | (3 << (6 * 2)) | (3 << (7 * 2)); // High speed
    GPIOA->AFR[0]  |= (5 << (5 * 4)) | (5 << (6 * 4)) | (5 << (7 * 4)); // af 5
    SPI1->CR1 = 0;

    SPI1->CR1 = SPI_CR1_MSTR     // Master 
              | SPI_CR1_SSM      // Software CS management
              | SPI_CR1_SSI      // Set nss high
              | (3 << 3);    // fPCLK/8 

    SPI1->CR1 |= SPI_CR1_SPE;  
    SPI1->CR2 = SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN; 

    // dummy
    empty = &dmy;
    current = &dmy;
    queue_head = 0;
    for (int i = 0; i < SPI_QUEUE_LENGTH; ++i) {
      queue[i] = &dmy;
    }
    
    DMA2_init();
}



extern "C" {
    void dma2_stream3_handler(){
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
	    CS_A_H();
	    CS_M_H();
	    current->state = Done;
	    current = empty;
	    //os_printf("res: %d \n", current->rx_buffer[1]);
	    dma_done = 1;
            SPI_send_next();
            }
    }
}

