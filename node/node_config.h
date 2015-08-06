#define NODE_ID 1

#ifdef ETHERNET_ENABLE
//#define ETH_RESET_PIN 9 // uncomment this line to be able to reset ethernet module manually
uint8_t mac[6] = {0x02,0x00,0x00,0x00,0x00,NODE_ID};
IPAddress myIP(192, 168, 137, 20 + NODE_ID);
#endif

