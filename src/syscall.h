#include <sys/stat.h>
#include <sys/times.h>
#include "usart.h"
#include <cerrno>

extern "C" {
    void _exit(int exit_code);
    int _close(int file);
    int _execve(char *name, char **argv, char **env);
    int _fork();
    int _fstat(int file, struct stat *st);
    int _getpid();
    int _isatty(int file);
    int _kill(int pid, int sig);
    int _link(char *old, char *new);
    int _lseek(int file, int ptr, int dir);
    int _open(const char *name, int flags, int mode);
    int _read(int file, char *ptr, int len);
    char * stack_ptr asm("sp");
    caddr_t _sbrk(int incr);
    int _stat(char *file, struct stat *st);
    int _times(struct tms *buf);
    int _unlink(char *name);
    int _wait(int *status);
    int _write(int file, char *ptr, int len);
}


