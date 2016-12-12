
#ifndef NUMBER_H_
#define NUMBER_H_

#include <cstdlib>
#include <stdint.h>
#include <iostream>

namespace sawyer
{

/* Little-endian byte array to number conversion routines. */
  void utob(void *dest, size_t dest_len, uint64_t src);

  void utob(void *dest, size_t dest_len, uint32_t src);

  void utob(void *dest, size_t dest_len, uint16_t src);

  void itob(void *dest, size_t dest_len, int64_t src);

  void itob(void *dest, size_t dest_len, int32_t src);

  void itob(void *dest, size_t dest_len, int16_t src);

/* void toBytes(void* dest, size_t dest_len, float src, float scale); */
  void ftob(void *dest, size_t dest_len, double src, double scale);

/* Number to little-endian byte array conversion routines 
 * Need to provide all, since size of the int param matters. */
  uint64_t btou(void *src, size_t src_len);

  int64_t btoi(void *src, size_t src_len);

  double btof(void *src, size_t src_len, double scale);

}; // namespace sawyer

#endif // NUMBER_H_

