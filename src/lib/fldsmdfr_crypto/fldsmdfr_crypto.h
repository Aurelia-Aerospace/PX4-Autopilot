#pragma once

class FLDSMDFR_CRYPTO {
	public:
		FLDSMDFR_CRYPTO();
		~FLDSMDFR_CRYPTO() = default;
		int unlock(uint8_t *plain_text, const uint8_t key[32], const uint8_t nonce[24], const uint8_t mac[16],const uint8_t *cipher_text, size_t text_size);
	private:
};
