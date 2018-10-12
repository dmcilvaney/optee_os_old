/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) Microsoft Corporation. All rights reserved.
 */
#ifndef __IMX_CDI__
#define __IMX_CDI__

#include <tee_api_types.h>

#define SPL_IDENTITY_SEED_ADDR CAAM_SECURE_MEMORY_PAGE2
#define SPL_IDENTITY_SEED_LENGTH TEE_SHA256_HASH_SIZE

/*
 * This structe is filled in by SPL. SPL is responsible for deriving a
 * unique device identity from a secure source. SPL will still
 * generate an identity even if the board is not configured in secure 
 * mode, but that value should not be used to write an RPMB key.
 */

#define SPL_IDENTITY_SEED_MAGIC 0x43434449
#define SPL_IDENTITY_SEED_VERSION 1

struct spl_identity_seed {
	uint32_t magic;
	uint32_t version;
	uint8_t hashed_device_id[SPL_IDENTITY_SEED_LENGTH];
	uint8_t reserved[SPL_IDENTITY_SEED_LENGTH];
	bool id_valid;
};

#endif /* __IMX_CDI__ */