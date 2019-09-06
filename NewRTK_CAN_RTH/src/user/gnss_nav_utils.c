
#include "gnss_nav_utils.h"

uint32_t getbitu(const uint8_t *buff, int pos, int len)
{
	uint32_t bits = 0;
	int i;
	for (i = pos; i < pos + len; i++)
		bits = (bits << 1) + ((buff[i / 8] >> (7 - i % 8)) & 1u);

	return bits;
}


int32_t getbits(const uint8_t *buff, int pos, int len)
{
	uint32_t bits = getbitu(buff, pos, len);

	if (len <= 0 || 32 <= len || !(bits & (1u << (len - 1))))
		return (int32_t)bits;

	return (int32_t)(bits | (~0u << len)); /* extend sign */
}


void *nav_memcpy(void *dest, const void *src, uint32_t count)
{
	int8_t *dst8 = (int8_t *)dest;
	int8_t *src8 = (int8_t *)src;

	if (count == 0 || dst8 == src8) {
		return dest;
	}

	while (count--) {
		*dst8++ = *src8++;
	}

	return dest;
}