#ifndef audioraw_h
#define audioraw_h

//------------------------------------------------------------------------------
// Data block for 12-bit ADC mode.
struct block_t {
  unsigned short data[32];
};
//------------------------------------------------------------------------------
// Data block for PC use
struct adcdata_t {
  unsigned short count;    // count of data values
  unsigned short overrun;  // count of overruns since last block
  union {
    unsigned char  u8[64];
    unsigned short u16[32];
  } data;
};
#endif  // audioraw_h
