# SROS (Simple Realtime Operating System) 

SROS is a simple rtos for the stm32f103.
It features preemptive real-time scheduling and blocking printf.

## Compiling

First you will have to create the build folder: 
``` bash
mkdir build && cd build
``` 
And then you'll have to run CMake with the selected compilation target passed in: 
For STM32F407G:
``` bash
cmake -DPLATFORM_CONFIG_FILE=platform/stm32f407.cmake ..
```

For STM32F103:
``` bash
cmake -DPLATFORM_CONFIG_FILE=platform/stm32f103.cmake ..
```

You might also want to add the `compile_commands.json` file depending on your ide:
``` bash
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DPLATFORM_CONFIG_FILE=platform/stm32f407.cmake ..
```
or 
``` bash
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DPLATFORM_CONFIG_FILE=platform/stm32f103.cmake ..
```
You can then build the project with:
``` bash
make
```

## Flashing
The make script already includes a flashing script:
``` bash
make flash
```

## Creating threads

A thread can be declared in any header file, the implementation of witch can be
done either in the corresponding .cpp or .c file or in any other file which
includes the header (the implementation may also be done in the header).

To define the thread simply use a method that never returns, for example:

``` c
volatile void template_thread () {
  // variable initialisation
  while (1) {
	  // your code
  }
}
```

The thread then needs to be registered with the os in the main.cpp file from
line 73 onwards:

``` c
register_thread_auto(&template_thread);
```

The default characteristics may also be specified in globals.h

A more custome way to create threads is:

``` c
os_pcb *register_thread_auto(void (*thread_handler)(), uint32_t stack_size, uint8_t priority, char* name);
```

A thread can also `yield()` to allow another thread of the same priority to run.
To let a thread sleep the `void sleep(uint64_t time)` method may be used with
time in milliseconds.

Notice that the `SECONDS`, `MINUTES`, `HOURS` and `DAYS` simplify longer sleep duration's.

## Memory allocation

To allocate memory the `void *os_alloc(size_t size)` method may be used, with `size` in byte.
The resulting allocation will be 64 byte aligned.
The method returns 0 if allocation is not possible.
Use `os_free(void * pointer)` to free the allocated pointer.

## printf

The operating system provides the `os_printf(char * format, ...)` method to
print through UART (GPIOA 9 and 10).
