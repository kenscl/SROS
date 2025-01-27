#ifndef USART
#define USART
    void clock_init_usart1();
    void enable_usart();
//void usart_write(USART_TypeDef *usart, char c);

    // actual print fucntions
    void os_putchar(char c);
    void os_putstr(const char * s);
    void os_putint(int i);
    void os_printf(const char* format, ... );
#endif
