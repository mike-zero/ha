#define DEFAULT_ID 254  // to be used until configured online

#define BUF_LEN 512

#define STX 0x02
#define ETX 0x03

#define SERIAL_STATE_READY 0   // waiting for STX
#define SERIAL_STATE_ADDR_H 1  // got STX, waiting for a high nibble of address
#define SERIAL_STATE_ADDR_L 2  // waiting for a low nibble of address
#define SERIAL_STATE_FROM_H 3  // got addr, waiting for a high nibble of from address
#define SERIAL_STATE_FROM_L 4  // waiting for a low nibble of from address
#define SERIAL_STATE_MSG_H 5   // waiting for a high nibble of message byte
#define SERIAL_STATE_MSG_L 6   // waiting for a low nibble of message byte

#ifdef SERIAL_ENABLE
  #define GPIO_SERIAL_MASK 0b0000000000000011  // digital pins 0, 1
#else
  #define GPIO_SERIAL_MASK 0
#endif

#ifdef ETH_RESET_PIN
  #define GPIO_ETH_RESET_MASK (1 << ETH_RESET_PIN)
#else
  #define GPIO_ETH_RESET_MASK 0
#endif

#ifdef ETHERNET_ENABLE
  #define GPIO_ETH_MASK 0b0011110000000100 | GPIO_ETH_RESET_MASK // digital pins 2, 10-13 and reset pin (if defined)
#else
  #define GPIO_ETH_MASK 0
#endif

#ifdef LED
  #if ((1 << LED) & (GPIO_SERIAL_MASK | GPIO_ETH_MASK)) == 0 // LED pin is not used by another device
    #define GPIO_LED_MASK (1 << LED)
  #else
    #undef LED
    #define GPIO_LED_MASK 0
  #endif
#else
  #define GPIO_LED_MASK 0
#endif

#ifdef RS485_TX_EN
  #define GPIO_RS485_MASK (1 << RS485_TX_EN)
#else
  #define GPIO_RS485_MASK 0
#endif

#define GPIO_MASK (0b0011111111111111 & ~(GPIO_SERIAL_MASK | GPIO_ETH_MASK | GPIO_LED_MASK | GPIO_RS485_MASK)) // TODO: set all usefull bits, then clear used by Eth, LED, Serial, interrupts, etc.

