/*
 * syscalls.c - Newlib system call stubs for bare-metal STM32
 *
 * These stubs suppress linker warnings about unimplemented
 * _close, _lseek, _read, _write, _getpid, and _kill.
 * _sbrk is implemented for heap memory allocation.
 */

#include <errno.h>
#include <stdint.h>
#include <sys/types.h>

extern uint32_t _end;
extern uint32_t _estack;

/* Suppress "not implemented" warnings from newlib linker */

int _close(int file)
{
    (void)file;
    errno = EBADF;
    return -1;
}

int _lseek(int file, int ptr, int dir)
{
    (void)file;
    (void)ptr;
    (void)dir;
    errno = EBADF;
    return -1;
}

int _read(int file, char *ptr, int len)
{
    (void)file;
    (void)ptr;
    (void)len;
    errno = EBADF;
    return -1;
}

int _write(int file, const char *ptr, int len)
{
    (void)file;
    (void)ptr;
    (void)len;
    errno = EBADF;
    return -1;
}

int _getpid(void)
{
    return 1;
}

int _kill(int pid, int sig)
{
    (void)pid;
    (void)sig;
    errno = EINVAL;
    return -1;
}

caddr_t _sbrk(int incr)
{
    static uint8_t *heap = NULL;
    uint8_t *prev_heap;
    uint32_t stack_ptr;

    /* Get current stack pointer from MSP register */
    __asm volatile ("MRS %0, msp" : "=r" (stack_ptr));

    if (heap == NULL) {
        heap = (uint8_t *)&_end;
    }
    prev_heap = heap;

    /* Check if heap would overlap with stack */
    if ((uint32_t)heap + incr > (uint32_t)stack_ptr) {
        errno = ENOMEM;
        return (caddr_t)-1;
    }

    heap += incr;
    return (caddr_t)prev_heap;
}
