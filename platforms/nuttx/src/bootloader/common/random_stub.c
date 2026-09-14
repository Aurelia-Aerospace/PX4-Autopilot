/* Stub: bootloader only does Ed25519 verification — never calls px4_get_secure_random.
 * Provided to satisfy the linker when sw_crypto (crypto_backend) is linked. */
#include <stddef.h>
#include <stdint.h>

size_t px4_get_secure_random(uint8_t *out, size_t outlen)
{
	(void)out;
	(void)outlen;
	return 0;
}
