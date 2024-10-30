#ifndef _H_UTILS
#define _H_UTILS

#include <stdint.h>
#include <stdio.h>
#include <string>
#include <math.h>

char     cpu_le();
uint32_t be_u32(const uint32_t v);
uint32_t le_u32(const uint32_t v);
uint8_t gen_cs(const uint32_t v);

void printStatus(FILE *fd, int condition);

std::string halfbyte_to_hexstring(uint8_t b);

std::string byte_to_hexstring(uint8_t b);

std::string uint16_to_hexstring(uint16_t b);

std::string uint32_to_hexstring(uint32_t b);

uint32_t stringhex_to_byte(const std::string& hex);
#endif
