#include "Semaphore.h"
#include "stm32f4xx_hal.h"

#define SEMAPHORE_FREE		0
#define SEMAPHORE_CLAIMED	1

uint8_t Semaphore_TryLock(Semaphore* semaphore)
{
	if (__LDREXB(semaphore) == SEMAPHORE_FREE)
		return __STREXB(SEMAPHORE_CLAIMED, semaphore);
	
	return SEMAPHORE_FAILED;
}

void Semaphore_Unlock(Semaphore* semaphore)
{
	*semaphore = 0;
}
