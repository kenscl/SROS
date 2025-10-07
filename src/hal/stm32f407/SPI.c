#include "../../communication/SPI.h"
#include "../../communication/usart.h"
#include "../../globals.h"

#include "../../hw_init.h"
#include "stm32f407xx.h"
#include <stdint.h>

// cs-lines
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

// SPI queue
volatile SPI_transmition *SPI_buffer[SPI_buffer_lenght];
volatile size_t queue_size;
volatile uint8_t SPI_queue_head;
volatile struct SPI_transmition *SPI_current;
volatile uint8_t SPI_queue_tail;

volatile struct SPI_transmition dmy;

void queue_init(volatile SPI_transmition **buffer, size_t size) {
    queue_size = size;
    SPI_queue_head = 0;
    SPI_queue_tail = 0;

    dmy.size = 0;
    dmy.state = done;
    dmy.rx_buffer = 0;
    dmy.tx_buffer = 0;
    dmy.cs_high = 0;
    dmy.cs_low = 0;
    SPI_current = &dmy;
    for (int i = 0; i < SPI_buffer_lenght; ++i) {
        SPI_buffer[i] = &dmy;
    }
}

int is_empty() {
    return SPI_queue_head == SPI_queue_tail;
}

int is_full() {
    return (SPI_queue_head + 1) % queue_size == SPI_queue_tail;
}

int enqueue(volatile struct SPI_transmition *element) {
    if (is_full())
        return -1;
    SPI_buffer[SPI_queue_head] = element;
    SPI_queue_head = (SPI_queue_head + 1) % queue_size;
    return 0;
}

int dequeue(volatile struct SPI_transmition **ret) {
    if (is_empty())
        return -1;
    *ret = SPI_buffer[SPI_queue_tail];
    SPI_current = SPI_buffer[SPI_queue_tail];
    SPI_queue_tail = (SPI_queue_tail + 1) % queue_size;
    return 0;
}

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
        SPI_current->cs_high();
        SPI_current->state = done;
        os_printf("res: %d \n", SPI_current->rx_buffer[1]);
    }
}

void DMA2_init() {
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    // tx
    DMA2_Stream3->CR = 0;
    DMA2_Stream3->PAR = (uint32_t)&SPI1->DR;
    DMA2_Stream3->CR |= (0b011 << DMA_SxCR_CHSEL_Pos);
    DMA2_Stream3->CR |= (1 << DMA_SxCR_MINC_Pos);
    DMA2_Stream3->CR |= (0b01 << DMA_SxCR_DIR_Pos);
    DMA2_Stream3->CR |= (1 << DMA_SxCR_TCIE_Pos);
    DMA2_Stream3->CR |= (1 << DMA_SxCR_PL_Pos);

    // rx
    DMA2_Stream0->CR = 0;
    DMA2_Stream0->PAR = (uint32_t)&SPI1->DR;
    DMA2_Stream0->CR |= (0b11 << DMA_SxCR_CHSEL_Pos);
    DMA2_Stream0->CR |= (1 << DMA_SxCR_MINC_Pos);
    DMA2_Stream0->CR &= ~(0b011 << DMA_SxCR_DIR_Pos);
    DMA2_Stream0->CR |= (1 << DMA_SxCR_TCIE_Pos);
    DMA2_Stream0->CR |= (1 << DMA_SxCR_PL_Pos);
    DMA2_Stream0->FCR |= DMA_SxFCR_DMDIS;

    // NVIC_EnableIRQ(DMA2_Stream3_IRQn);
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    // NVIC_SetPriority(DMA2_Stream3_IRQn, 11);
    NVIC_SetPriority(DMA2_Stream0_IRQn, 10);
}

void SPI_init() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    GPIOA->MODER &= ~(0x3 << (4 * 2)); // Clear mode bits for pin 4
    GPIOA->MODER |= (0x1 << (4 * 2));
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

    queue_init(SPI_buffer, SPI_buffer_lenght);
}

int SPI_submit(volatile struct SPI_transmition *tx) {
    tx->state = pending;
    return enqueue(tx);
}

void SPI_handle() {
    // called during transmition or error
    if (SPI_current->state == busy) {
        return;
    } else if (SPI_current->state == error) {
        // error routine
    }

    // called when SPI is inactive
    volatile SPI_transmition *next;
    if (SPI_current->state == pending) {
        next = SPI_current;
        next->state = busy;
    } else if (SPI_current->state == done) {
        int ret = dequeue(&next);
        if (ret == -1)
            return;
    }

    // start next transmition
    DMA2_Stream3->CR &= ~DMA_SxCR_EN;
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

void SPI_thread() {
    SPI_init();
    while (1) {
        SPI_handle();
        sleep(1 * MILLISECONDS);
    }
}
