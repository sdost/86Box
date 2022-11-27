/*
 * 86Box	A hypervisor and IBM PC system emulator that specializes in
 *		running old operating systems and software designed for IBM
 *		PC systems and compatibles from 1981 through fairly recent
 *		system designs based on the PCI bus.
 *
 *		This file is part of the 86Box distribution.
 *
 *		RetroWave OPL3 bridge.
 *
 *		 Version: 1.0.0
 *
 * Version:	@(#)snd_opl_retrowave.c	1.0.0	2021/12/22
 *
 * Authors:	Fred N. van Kempen, <decwiz@yahoo.com>
 *		Miran Grca, <mgrca8@gmail.com>
 *		Alexey Khokholov (Nuke.YKT)
 *
 *		Copyright 2017-2020 Fred N. van Kempen.
 *		Copyright 2016-2020 Miran Grca.
 *		Copyright 2013-2018 Alexey Khokholov (Nuke.YKT)
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <86box/86box.h>
#include <86box/timer.h>
#include <86box/sound.h>
#include <86box/snd_opl_retrowave.h>
#include "RetroWave/RetroWaveLib/RetroWave_86Box.h"


#define WRBUF_SIZE	1024
#define WRBUF_DELAY	1
#define RSM_FRAC	10


struct chip;

typedef struct chip {
    uint8_t opl3_port;
} retrowave_t;


uint16_t
retrowave_write_addr(void *priv, uint16_t port, uint8_t val)
{
    retrowave_t *dev = (retrowave_t *)priv;
//                      printf("writeaddr: 0x%08x 0x%02x\n", port, val);

    switch (port & 3) {
        case 0:
            dev->opl3_port = 0;
            return val;
        case 2:
            dev->opl3_port = 1;
            if (val == 0x05)
                return 0x100 | val;
            else
                return val;
    }
}


void
retrowave_write_reg(void *priv, uint16_t reg, uint8_t val)
{
    retrowave_t *dev = (retrowave_t *)priv;
//                      printf("writereg: 0x%08x 0x%02x\n", reg, val);

                        uint8_t real_reg = reg & 0xff;
                        uint8_t real_val = val;

    if (dev->opl3_port) {
        retrowave_opl3_queue_port1(&retrowave_global_context, real_reg, real_val);
    } else {
        retrowave_opl3_queue_port0(&retrowave_global_context, real_reg, real_val);
    }
}


void *
retrowave_86box_module_init()
{
    retrowave_t *dev;
    uint8_t i;

    dev = (retrowave_t *)malloc(sizeof(retrowave_t));
    memset(dev, 0x00, sizeof(retrowave_t));

    dev->opl3_port = 0;

    retrowave_init_86box("ttyOPL");
    retrowave_opl3_reset(&retrowave_global_context);

    return(dev);
}


void
retrowave_close(void *priv)
{
    retrowave_t *dev = (retrowave_t *)priv;

    free(dev);
}


void
retrowave_generate_stream(void *priv, int32_t *sndptr, uint32_t num)
{
    retrowave_t *dev = (retrowave_t *)priv;
    uint32_t i;

    retrowave_flush(&retrowave_global_context);
}
