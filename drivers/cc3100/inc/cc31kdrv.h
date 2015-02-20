// --------------------------------------------------------------------------------------
// Module     : CC31KDRV
// Author     : N. El-Fata
// --------------------------------------------------------------------------------------
// Pulse Innovation, Inc. www.pulseinnovation.com
// Copyright (c) 2014, Pulse Innovation, Inc. All rights reserved.
// --------------------------------------------------------------------------------------

#ifndef CC31KDRV_H
#define CC31KDRV_H

typedef void (*irq_handler_t)(void *pVal);
typedef unsigned int Fd_t;

Fd_t cc31k_open(char *ifName, unsigned long flags);
int cc31k_close(Fd_t fd);
int cc31k_read(Fd_t fd, unsigned char *pBuff, int len);
int cc31k_write(Fd_t fd, unsigned char *pBuff, int len);
void cc31k_enable(void);
void cc31k_disable(void);
int cc31k_registerIrqHandler(irq_handler_t handler, void *pVal);


#endif // CC31KDRV_H

