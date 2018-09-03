/*
 * Diffie-Hellman group 5 operations
 * Copyright (c) 2009, Jouni Malinen <j@w1.fi>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Alternatively, this software may be distributed under the terms of BSD
 * license.
 *
 * See README and COPYING for more details.
 */

#include "crypto/includes.h"

#include "crypto/common.h"
#include "crypto/dh_groups.h"
#include "crypto/dh_group5.h"

void * ICACHE_FLASH_ATTR
dh5_init(struct wpabuf **priv, struct wpabuf **publ)
{
	*publ = dh_init(dh_groups_get(5), priv);
	if (*publ == 0)
		return NULL;
	return (void *) 1;
}


struct wpabuf * ICACHE_FLASH_ATTR
dh5_derive_shared(void *ctx, const struct wpabuf *peer_public,
				  const struct wpabuf *own_private)
{
	return dh_derive_shared(peer_public, own_private, dh_groups_get(5));
}


void ICACHE_FLASH_ATTR
dh5_free(void *ctx)
{
}
