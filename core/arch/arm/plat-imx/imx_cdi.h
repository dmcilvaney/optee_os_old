/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) Microsoft Corporation. All rights reserved.
 */
#ifndef __IMX_CDI__
#define __IMX_CDI__

#include <tee_api_types.h>

/*
 * The CDI values are placed in secure memory since it is important to
 * protect the CDI + OP-TEE measurement value from future access.
 * Once OP-TEE has consumed this value it should be zeroed out.
 */
#define CYRES_IDENTITY_SEED_ADDR CAAM_SECURE_MEMORY_PAGE2
#define RIOT_DIGEST_LENGTH TEE_SHA256_HASH_SIZE

/*
 * This structe is filled in by SPL. SPL is responsible for deriving a
 * unique device identity from a secure source. It also measures the
 * OP-TEE binary and combines it with the identity for use in creating
 * a certificate chain. SPL will still generate an identity even if the
 * board is not configured in secure mode, but that value should not be
 * used to write an RPMB key.
 */

#define CYRES_IDENTITY_SEED_MAGIC 0x43434449 // CCDI
#define CYRES_IDENTITY_SEED_VERSION 1

struct cyres_identity_seed {
	uint32_t magic;
	uint32_t version;
	uint8_t hashed_cdi[RIOT_DIGEST_LENGTH];
	uint8_t hashed_cdi_optee[RIOT_DIGEST_LENGTH];
	bool cdi_valid;
};

TEE_Result imx_was_cdi_successful(void);

#endif /* __IMX_CDI__ */