#ifndef USER_TCP_H
#define USER_TCP_H

#include "stm32f4xx.h"
#include "err.h"
#include "tcp.h"

#define CONFIG_PORT 3355
#define OPEN_PORT	  3356


void tcpSend(uint8_t array[], uint8_t size);
void keySendInit(void);
void sysConfigInit(void);
void changeSysConfig(uint8_t id, uint8_t param);
err_t changeConfig(struct pbuf *p);
err_t eventconnect(struct tcp_pcb *pcb, struct pbuf *p, err_t err);


#endif /* USER_TCP_H */
