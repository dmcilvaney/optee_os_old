#include <fsl_caam/fsl_sec.h>
#include <io.h>
#include <kernel/mutex.h>
#include <kernel/panic.h>
#include <mm/core_mmu.h>
#include <mm/core_memprot.h>
#include <platform_config.h>
#include <rng_support.h>
#include <trace.h>

#define CAAM_RANDOM_BUFFER_SIZE 0x100

static unsigned int rng_lock = SPINLOCK_UNLOCK;
static uint8_t random_buffer[CAAM_RANDOM_BUFFER_SIZE];
static volatile size_t available = 0;

uint8_t hw_get_random_byte(void)
{
	int ret = TEE_SUCCESS;
	uint8_t random_value;

	cpu_spin_lock(&rng_lock);
	if(available <= 0) {
		FMSG("Refilling RNG buffer");
		ret = fsl_get_random_bytes(random_buffer,
				 CAAM_RANDOM_BUFFER_SIZE);
		if (ret) {
			EMSG("Failed to get random bytes: %x", ret);
			panic();
		}
		available = CAAM_RANDOM_BUFFER_SIZE;
	}

	available -= 1;
	random_value = random_buffer[available];
	cpu_spin_unlock(&rng_lock);

	return random_value;
}
