#include <fsl_caam/fsl_sec.h>
#include <io.h>
#include <kernel/panic.h>
#include <mm/core_mmu.h>
#include <mm/core_memprot.h>
#include <platform_config.h>
#include <rng_support.h>
#include <trace.h>

#define CAAM_RANDOM_BUFFER_SIZE 0x100

uint8_t hw_get_random_byte(void)
{
    static uint8_t random_buffer[CAAM_RANDOM_BUFFER_SIZE];
    static size_t available = 0;
    int ret = TEE_SUCCESS;

    if(available <= 0) {
        DMSG("Refilling RNG buffer");
        ret = fsl_get_random_bytes(random_buffer, CAAM_RANDOM_BUFFER_SIZE);
        if (ret) {
            EMSG("Failed to get random bytes: %x", ret);
            panic();
        }
        available = CAAM_RANDOM_BUFFER_SIZE;
    }

    available -= 1;
    DMSG("Generating random number: %x", random_buffer[available]);
    return random_buffer[available];
}
