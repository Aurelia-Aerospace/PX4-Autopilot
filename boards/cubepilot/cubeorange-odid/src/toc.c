/****************************************************************************
 * Firmware table of contents for secure boot signature verification.
 * The linker places this struct at BOOT_DELAY_ADDRESS+8 (0x1a8 from app
 * start), where the bootloader's find_toc() looks for it.
 ****************************************************************************/

#include <image_toc.h>

#define SIGNATURE_SIZE 64

extern uint32_t _vectors[];
extern const int *_boot_signature;

IMAGE_MAIN_TOC(2) = {
	{TOC_START_MAGIC, TOC_VERSION},
	{
		{"BOOT", _vectors, (const void *)&_boot_signature,
		 0, 1, 0, 0, TOC_FLAG1_BOOT | TOC_FLAG1_VTORS | TOC_FLAG1_CHECK_SIGNATURE},
		{"SIG1", (const void *)&_boot_signature,
		 (const void *)((const uint8_t *)&_boot_signature + SIGNATURE_SIZE),
		 0, 0, 0, 0, 0},
	},
	TOC_END_MAGIC
};
