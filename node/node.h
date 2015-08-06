#define BUF_LEN 64
#define BUF_NUM 4

#define CH_COUNT 2
#define CH_SERIAL 0
#define CH_ETH 1

#ifdef SERIAL_ENABLE
#define GPIO_SERIAL_MASK 0b0000000000000011  // digital pins 0, 1
#else
#define GPIO_SERIAL_MASK 0
#endif

#ifdef ETHERNET_ENABLE
#define GPIO_ETH_MASK 0b0011110000000100  // digital pins 2, 10-13
#else
#define GPIO_ETH_MASK 0
#define LED 13  // uncomment this line to show command processing
#endif

#ifdef LED
#define GPIO_LED_MASK 0b0010000000000000  // digital pin 13
#else
#define GPIO_LED_MASK 0
#endif

#ifdef ETH_RESET_PIN
#define GPIO_ETH_RESET_MASK 0b0000001000000000  // digital pin 9
#else
#define GPIO_ETH_RESET_MASK 0
#endif

#define GPIO_MASK 0b0011111111111111 & GPIO_SERIAL_MASK & ~GPIO_ETH_MASK & ~GPIO_ETH_RESET_MASK & ~GPIO_LED_MASK  // TODO: set all usefull bits, then clear used by Eth, LED, Serial, interrupts, etc.

