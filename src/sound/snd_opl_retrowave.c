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
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define HAVE_STDARG_H
#include <86box/86box.h>
#include <86box/snd_opl_retrowave.h>
#include <86box/timer.h>
#include <86box/sound.h>
#include <86box/device.h>
#include <86box/snd_opl.h>
#include "RetroWave/RetroWaveLib/RetroWave_86Box.h"

#define WRBUF_SIZE  1024
#define WRBUF_DELAY 1
#define RSM_FRAC    10

struct chip;

typedef struct chip {
    uint8_t opl3_port;
} retrowave_t;

typedef struct {
    retrowave_t opl;
    int8_t      flags, pad;

    uint16_t port;
    uint8_t  status, timer_ctrl;
    uint16_t timer_count[2],
        timer_cur_count[2];

    pc_timer_t timers[2];

    int     pos;
    int32_t buffer[SOUNDBUFLEN * 2];
} retrowave_drv_t;

enum {
    FLAG_CYCLES = 0x02,
    FLAG_OPL3   = 0x01
};

enum {
    STAT_TMR_OVER  = 0x60,
    STAT_TMR1_OVER = 0x40,
    STAT_TMR2_OVER = 0x20,
    STAT_TMR_ANY   = 0x80
};

enum {
    CTRL_RESET      = 0x80,
    CTRL_TMR_MASK   = 0x60,
    CTRL_TMR1_MASK  = 0x40,
    CTRL_TMR2_MASK  = 0x20,
    CTRL_TMR2_START = 0x02,
    CTRL_TMR1_START = 0x01
};

#ifdef ENABLE_OPL_LOG
int retrowave_do_log = ENABLE_OPL_LOG;

static void
retrowave_log(const char *fmt, ...)
{
    va_list ap;

    if (retrowave_do_log) {
        va_start(ap, fmt);
        pclog_ex(fmt, ap);
        va_end(ap);
    }
}
#else
#    define retrowave_log(fmt, ...)
#endif

uint16_t
retrowave_write_addr(void *priv, uint16_t port, uint8_t val)
{
    retrowave_t *dev = (retrowave_t *) priv;
    retrowave_log("writeaddr: 0x%08x 0x%02x\n", port, val);

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
    retrowave_t *dev = (retrowave_t *) priv;
    retrowave_log("writereg: 0x%08x 0x%02x\n", reg, val);

    uint8_t real_reg = reg & 0xff;
    uint8_t real_val = val;

    if (dev->opl3_port) {
        retrowave_opl3_queue_port1(&retrowave_global_context, real_reg, real_val);
    } else {
        retrowave_opl3_queue_port0(&retrowave_global_context, real_reg, real_val);
    }
}

void
retrowave_init(retrowave_t *dev)
{
    uint8_t i;

    memset(dev, 0x00, sizeof(retrowave_t));

    dev->opl3_port = 0;

    retrowave_init_86box("ttyOPL");
    retrowave_opl3_reset(&retrowave_global_context);
}

void
retrowave_generate_stream(retrowave_t *dev)
{
    retrowave_flush(&retrowave_global_context);
}

static void
retowave_timer_tick(retrowave_drv_t *dev, int tmr)
{
    dev->timer_cur_count[tmr] = (dev->timer_cur_count[tmr] + 1) & 0xff;

    retrowave_log("Ticking timer %i, count now %02X...\n", tmr, dev->timer_cur_count[tmr]);

    if (dev->timer_cur_count[tmr] == 0x00) {
        dev->status |= ((STAT_TMR1_OVER >> tmr) & ~dev->timer_ctrl);
        dev->timer_cur_count[tmr] = dev->timer_count[tmr];

        retrowave_log("Count wrapped around to zero, reloading timer %i (%02X), status = %02X...\n", tmr, (STAT_TMR1_OVER >> tmr), dev->status);
    }

    timer_on_auto(&dev->timers[tmr], (tmr == 1) ? 320.0 : 80.0);
}

static void
retrowave_timer_control(retrowave_drv_t *dev, int tmr, int start)
{
    timer_on_auto(&dev->timers[tmr], 0.0);

    if (start) {
        retrowave_log("Loading timer %i count: %02X = %02X\n", tmr, dev->timer_cur_count[tmr], dev->timer_count[tmr]);
        dev->timer_cur_count[tmr] = dev->timer_count[tmr];
        if (dev->flags & FLAG_OPL3)
            timer_tick(dev, tmr); /* Per the YMF 262 datasheet, OPL3 starts counting immediately, unlike OPL2. */
        else
            timer_on_auto(&dev->timers[tmr], (tmr == 1) ? 320.0 : 80.0);
    } else {
        retrowave_log("Timer %i stopped\n", tmr);
        if (tmr == 1) {
            dev->status &= ~STAT_TMR2_OVER;
        } else
            dev->status &= ~STAT_TMR1_OVER;
    }
}

static void
retrowave_timer_1(void *priv)
{
    retrowave_drv_t *dev = (retrowave_drv_t *) priv;

    retrowave_timer_tick(dev, 0);
}

static void
retrowave_timer_2(void *priv)
{
    retrowave_drv_t *dev = (retrowave_drv_t *) priv;

    retrowave_timer_tick(dev, 1);
}

void
retrowave_drv_set_do_cycles(device_t *dev, int8_t do_cycles)
{
    if (do_cycles)
        dev->flags |= FLAG_CYCLES;
    else
        dev->flags &= ~FLAG_CYCLES;
}

static void
retrowave_drv_init(const device_t *info, int is_opl3)
{
    retrowave_drv_t *dev = (retrowave_drv_t *) calloc(1, sizeof(retrowave_drv_t));
    dev->flags           = FLAG_CYCLES;
    if (info->local == FM_YMF262)
        dev->flags |= FLAG_OPL3;
    else
        dev->status = 0x06;

    /* Create a RetroWave object. */
    retrowave_init(&dev->opl);

    timer_add(&dev->timers[0], retrowave_timer_1, dev, 0);
    timer_add(&dev->timers[1], retrowave_timer_2, dev, 0);

    return dev;
}

void
retrowave_drv_close(void *priv)
{
    retrowave_drv_t *dev = (retrowave_drv_t *) priv;
    free(dev);
}

uint8_t
retrowave_drv_read(uint16_t port, void *priv)
{
    retrowave_drv_t *dev = (retrowave_drv_t *) priv;

    if (dev->flags & FLAG_CYCLES)
        cycles -= ((int) (isa_timing * 8));

    retrowave_drv_update(dev);

    uint8_t ret = 0xff;

    if ((port & 0x0003) == 0x0000) {
        ret = dev->status;
        if (dev->status & STAT_TMR_OVER)
            ret |= STAT_TMR_ANY;
    }

    retrowave_log("OPL statret = %02x, status = %02x\n", ret, dev->status);

    return ret;
}

void
retrowave_drv_write(uint16_t port, uint8_t val, void *priv)
{
    retrowave_drv_t *dev = (retrowave_drv_t *) priv;

    retrowave_drv_update(dev);

    if ((port & 0x0001) == 0x0001) {
        retrowave_write_reg_buffered(&dev->opl, dev->port, val);

        switch (dev->port) {
            case 0x02: /* Timer 1 */
                dev->timer_count[0] = val;
                retrowave_log("Timer 0 count now: %i\n", dev->timer_count[0]);
                break;

            case 0x03: /* Timer 2 */
                dev->timer_count[1] = val;
                retrowave_log("Timer 1 count now: %i\n", dev->timer_count[1]);
                break;

            case 0x04: /* Timer control */
                if (val & CTRL_RESET) {
                    retrowave_log("Resetting timer status...\n");
                    dev->status &= ~STAT_TMR_OVER;
                } else {
                    dev->timer_ctrl = val;
                    retrowave_timer_control(dev, 0, val & CTRL_TMR1_START);
                    retrowave_timer_control(dev, 1, val & CTRL_TMR2_START);
                    retrowave_log("Status mask now %02X (val = %02X)\n", (val & ~CTRL_TMR_MASK) & CTRL_TMR_MASK, val);
                }
                break;
        }
    } else {
        dev->port = retrowave_write_addr(&dev->opl, port, val) & 0x01ff;

        if (!(dev->flags & FLAG_OPL3))
            dev->port &= 0x00ff;
    }
}

void
retrowave_drv_update(void *priv)
{
    retrowave_drv_t *dev = (retrowave_drv_t *) priv;

    if (dev->pos >= sound_pos_global)
        return;

    retrowave_generate_stream(&dev->opl);

    for (; dev->pos < sound_pos_global; dev->pos++) {
        dev->buffer[dev->pos * 2] /= 2;
        dev->buffer[(dev->pos * 2) + 1] /= 2;
    }
}

static void
retrowave_drv_reset_buffer(void *priv)
{
    retrowave_drv_t *dev = (retrowave_drv_t *) priv;

    dev->pos = 0;
}

const device_t ymf262_retrowave_device = {
    .name          = "Retrowave OPL3 (Serial)",
    .internal_name = "ymf262_retrowave",
    .flags         = 0,
    .local         = FM_YMF262,
    .init          = retrowave_drv_init,
    .close         = retrowave_drv_close,
    .reset         = NULL,
    { .available = NULL },
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};

const fm_drv_t retrowave_opl_drv = {
    &retrowave_drv_read,
    &retrowave_drv_write,
    &retrowave_drv_update,
    &retrowave_drv_reset_buffer,
    &retrowave_drv_set_do_cycles,
    NULL,
};
