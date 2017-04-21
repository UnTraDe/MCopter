#ifndef SEMAPHORE_H_
#define SEMAPHORE_H_

#include <stdint.h>

#define SEMAPHORE_SUCCESS	0
#define SEMAPHORE_FAILED	1

typedef volatile uint8_t Semaphore;

uint8_t Semaphore_TryLock(Semaphore* semaphore);
void Semaphore_Unlock(Semaphore* semaphore);

#endif