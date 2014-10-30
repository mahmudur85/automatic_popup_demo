#ifndef _sbit_h_
#define _sbit_h_

//BOOL
#define TRUE	0x01
#define FALSE	0x00

struct bits {
  uint8_t b0:1;
  uint8_t b1:1;
  uint8_t b2:1;
  uint8_t b3:1;
  uint8_t b4:1;
  uint8_t b5:1;
  uint8_t b6:1;
  uint8_t b7:1;
} __attribute__((__packed__));


#define SBIT(port,pin) ((*(volatile struct bits*)&port).b##pin)

#define sbi(address,bit); (address |= (1<<bit));
#define cbi(address,bit); (address &=~(1<<bit));

#endif
