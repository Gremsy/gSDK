#include <stdint.h>
#include "utils.h"

/* detect CPU endian */
char cpu_le() {
	const uint32_t cpu_le_test = 0x12345678;
	return ((cpu_le_test & 0x000000FF) == 0x78) ? 1 : 0;
}

uint32_t be_u32(const uint32_t v) {
	if (cpu_le())
    {
		return	((v & 0xFF000000) >> 24) |
			((v & 0x00FF0000) >>  8) |
			((v & 0x0000FF00) <<  8) |
			((v & 0x000000FF) << 24);
    }
	return v;
}

uint32_t le_u32(const uint32_t v) {
        if (!cpu_le())
                return  ((v & 0xFF000000) >> 24) |
                        ((v & 0x00FF0000) >>  8) |
                        ((v & 0x0000FF00) <<  8) |
                        ((v & 0x000000FF) << 24);
        return v;
}
uint8_t gen_cs(const uint32_t v)
{
        return (uint8_t)(((v & 0xFF000000) >> 24) ^
                    ((v & 0x00FF0000) >> 16) ^
                    ((v & 0x0000FF00) >> 8) ^
                    ((v & 0x000000FF) >> 0));
}
void printStatus(FILE *fd, int condition){
	if(condition)
		fprintf(fd, "Error!\n");
	else
		fprintf(fd, "OK\n");
}

std::string HEX_TO_STRING[16] = { "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "a", "b", "c", "d", "e", "f" };


std::string halfbyte_to_hexstring(uint8_t b) {
    if (b < 16) {
        return HEX_TO_STRING[b];
    } else {
        return "None";
    }
}

std::string byte_to_hexstring(uint8_t b) {
    uint8_t bh = b / 16;
    uint8_t bl = b % 16;
    return halfbyte_to_hexstring(bh) + halfbyte_to_hexstring(bl);
}


std::string uint16_to_hexstring(uint16_t b) {
    uint8_t bh = b / 256;
    uint8_t bl = b % 256;
    return byte_to_hexstring(bh) + byte_to_hexstring(bl);
}


std::string uint32_to_hexstring(uint32_t b) {
    uint16_t bh = b / 65536;
    uint16_t bl = b % 65536;
    return uint16_to_hexstring(bh) + uint16_to_hexstring(bl);
}


uint32_t stringhex_to_byte(const std::string& hex) {
    int len = hex.length();
    uint32_t sum = 0;
    for (int i = 0; i < len; i++) {
        if (hex[i] >= '0' && hex[i] <= '9') {
            sum += (uint32_t)((hex[i] - '0') * std::pow(16, len - i - 1));
        } else if (hex[i] >= 'A' && hex[i] <= 'F') {
            sum += (uint32_t)((hex[i] - '7') * std::pow(16, len - i - 1));
        } else if (hex[i] >= 'a' && hex[i] <= 'f') {
            sum += (uint32_t)((hex[i] - 'W') * std::pow(16, len - i - 1));
        }
    }
    return sum;
}