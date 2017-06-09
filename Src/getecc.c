// Generate ECC from 64 bit word: {address[31:0],data[31:0]}
// Uses HSIAO 72_64 H Matrix 
#include <stdint.h>

unsigned char getecc(uint32_t adr, uint32_t data)
{
	
static const unsigned int H[12] = {
  0x08099264,
  0xC8080992,
  0x38C80809,
  0x0738C808,
  0xFF0738C8,
  0x64FF0738,
  0x9264FF07,
  0x099264FF,
  0x08099264,
  0xC8080992,
  0x38C80809,
  0x0738C808
};

unsigned int* ptr_H;
int i;   
unsigned int res;
unsigned int ecc;

   ecc = 0;
   ptr_H = (unsigned int*)(&H);
   i = 8;
   do
   {
      res = *ptr_H;
      res &= data;
      res ^= *(ptr_H + 4) & adr;
      ptr_H ++;
      res ^= res << 16;
      res ^= res << 8;
      res ^= res << 4;
      res ^= res << 2;
      res ^= res << 1;
      res >>= 31;
      i--;
      res <<= i;
      ecc |= res;
   }
   while(i);
   return ecc;
} 				
