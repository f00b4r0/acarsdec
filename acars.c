/*
 *  Copyright (c) 2015 Thierry Leconte
 *  Copyright (c) 2024 Thibaut VARENE
 *
 *   
 *   This code is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU Library General Public License version 2
 *   published by the Free Software Foundation.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Library General Public License for more details.
 *
 *
 */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <pthread.h>
#include "acarsdec.h"
#include "output.h"
#include "statsd.h"

// ACARS is LSb first, 7-bit ASCII range. MSb is odd parity bit.

// include parity MSb
#define SYN 0x16
#define SOH 0x01
#define STX 0x02
#define ETX 0x83	// 0x03|0x80
#define ETB 0x97	// 0x17|0x80
#define DEL 0x7f

/* message queue */
static pthread_mutex_t blkq_mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t blkq_wcd = PTHREAD_COND_INITIALIZER;
static msgblk_t *blkq_h = NULL;
static pthread_t blkth_id;

static int acars_shutdown = 1;

#include "syndrom.h"

static int fixprerr(msgblk_t *blk, const unsigned short crc, int *pr, int pn)
{
	int i;

	if (pn > 0) {
		/* try to recursievly fix parity error */
		for (i = 0; i < 8; i++) {
			if (fixprerr(blk, crc ^ syndrom[i + 8 * (blk->len - *pr + 1)], pr + 1, pn - 1)) {
				blk->txt[*pr] ^= (1 << i);
				return 1;
			}
		}
		return 0;
	} else {
		/* end of recursion : no more parity error */
		if (crc == 0)
			return 1;

		/* test remainding error in crc */
		for (i = 0; i < 2 * 8; i++)
			if (syndrom[i] == crc) {
				return 1;
			}
		return 0;
	}
}

static int fixdberr(msgblk_t *blk, const unsigned short crc)
{
	int i, j, k;

	/* test remainding error in crc */
	for (i = 0; i < 2 * 8; i++)
		if (syndrom[i] == crc) {
			return 1;
		}

	/* test double error in bytes */
	for (k = 0; k < blk->len; k++) {
		int bo = 8 * (blk->len - k + 1);
		for (i = 0; i < 8; i++)
			for (j = 0; j < 8; j++) {
				if (i == j)
					continue;
				if ((crc ^ syndrom[i + bo] ^ syndrom[j + bo]) == 0) {
					blk->txt[k] ^= (1 << i);
					blk->txt[k] ^= (1 << j);
					return 1;
				}
			}
	}
	return 0;
}

#define MAXPERR 3
static void *blk_thread(void *arg)
{
	do {
		msgblk_t *blk;
		int i, pn, chn;
		unsigned short crc;
		int pr[MAXPERR];

		vprerr("blk_starting\n");

		/* pop a message */
		pthread_mutex_lock(&blkq_mtx);
		while ((blkq_h == NULL) && !acars_shutdown)
			pthread_cond_wait(&blkq_wcd, &blkq_mtx);

		if (acars_shutdown) {
			pthread_mutex_unlock(&blkq_mtx);
			break;
		}

		blk = blkq_h;
		blkq_h = blk->prev;
		pthread_mutex_unlock(&blkq_mtx);

		chn = blk->chn;

		vprerr("get message #%d\n", chn+1);

		if (R.statsd)
			statsd_inc_per_channel(chn, "decoder.msg.count");

		/* handle message */
		if (blk->len < 13) {
			vprerr("#%d too short\n", chn+1);
			if (R.statsd)
				statsd_inc_per_channel(chn, "decoder.errors.too_short");
			free(blk);
			continue;
		}

		/* force STX/ETX */
		blk->txt[12] &= (ETX | STX);
		blk->txt[12] |= (ETX & STX);

		/* parity check */
		pn = 0;
		for (i = 0; i < blk->len; i++) {
			if (parity8(blk->txt[i]) == 0) {
				if (pn < MAXPERR) {
					pr[pn] = i;
				}
				pn++;
			}
		}
		if (pn > MAXPERR) {
			vprerr("#%d too many parity errors: %d\n", chn+1, pn);
			if (R.statsd)
				statsd_inc_per_channel(chn, "decoder.errors.parity_excess");
			free(blk);
			continue;
		}

		/* crc check */
		crc = 0;
		for (i = 0; i < blk->len; i++)
			crc = update_crc16(crc, blk->txt[i]);
		crc = update_crc16(crc, blk->crc[0]);
		crc = update_crc16(crc, blk->crc[1]);

		/* try to fix errors: parity or crc */
		if (pn) {
			vprerr("#%d parity error(s): %d\n", chn+1, pn);
			if (fixprerr(blk, crc, pr, pn) == 0) {
				vprerr("#%d not able to fix errors\n", chn+1);
				free(blk);
				continue;
			}
			vprerr("#%d errors fixed\n", chn+1);
		}
		else if (crc) {
			vprerr("#%d crc error\n", chn+1);
			if (R.statsd)
				statsd_inc_per_channel(chn, "decoder.errors.crc");
			if (fixdberr(blk, crc) == 0) {
				vprerr("#%d not able to fix errors\n", chn+1);
				free(blk);
				continue;
			}
			vprerr("#%d errors fixed\n", chn+1);
		}
		blk->err = pn;

		/* redo parity checking and removing */
		pn = 0;
		for (i = 0; i < blk->len; i++) {
			if (parity8(blk->txt[i]) == 0) {
				pn++;
			}
			blk->txt[i] &= 0x7f;
		}
		if (pn) {
			fprintf(stderr, "#%d parity check problem\n", chn+1);
			free(blk);
			continue;
		}

		if (R.statsd) {
			char pfx[16];
			statsd_metric_t metrics[] = {
				{ .type = STATSD_UCOUNTER, .name = "decoder.msg.good", .value.u = 1 },
				{ .type = STATSD_LGAUGE, .name = "decoder.msg.errs", .value.l = blk->err },
				{ .type = STATSD_FGAUGE, .name = "decoder.msg.lvl", .value.f = blk->lvl },
				{ .type = STATSD_LGAUGE, .name = "decoder.msg.len", .value.l = blk->len },
			};
			// use the frequency if available, else the channel number
			snprintf(pfx, sizeof(pfx), "%u.", R.channels[chn].Fr ? R.channels[chn].Fr : chn+1);
			statsd_update(pfx, metrics, ARRAY_SIZE(metrics));
		}

		outputmsg(blk);

		free(blk);

	} while (1);
	return NULL;
}

static void resetAcars(channel_t *ch)
{
	ch->Acarsstate = PREKEY;
	ch->MskDf = 0;
	ch->nbits = 8;
	ch->count = 0;
}

int initAcars(channel_t *ch)
{
	if (acars_shutdown) {
		acars_shutdown = 0;
		pthread_create(&blkth_id, NULL, blk_thread, NULL);
	}

	resetAcars(ch);
	ch->blk = NULL;

	return 0;
}

void decodeAcars(channel_t *ch)
{
	unsigned char r = ch->outbits;
	//vprerr("#%d r: %x, count: %d, st: %d\n", ch->chn+1, r, ch->count, ch->Acarsstate);

	ch->nbits = 8;	// by default we'll read another byte next

	switch (ch->Acarsstate) {
	case PREKEY:
		if (ch->count >= 12 && 0xFF != r) {	// we have our first non-0xFF byte after a sequence - XXX REVISIT: expect at least 16: adjust count depending on how fast the MSK PLL locks
			unsigned char q = ~r;	// avoid type promotion in calling ffs(~r)
			int l = ffs(q);		// find the first (LSb) 0 in r

			vprerr("#%d synced, count: %d, r: %x, fz: %d, lvl: %5.1f\n", ch->chn+1, ch->count, r, l, 10 * log10(ch->MskLvl));
			ch->count = 0;
			ch->Acarsstate = SYNC;

			// after the 0xFF sequence we expect a possibly shifted '+'|0x80, aka 0xAB: 10101011
			if (l < 3) {
				/*
				 if the first zero is in position 1 or 2, assume we have already eaten into the '+', attempt sync on next symbol.
				 NB we could check if we could reconstruct a '+' by shifting left, but that's too much effort for too little gain ;P
				 3 possible cases: 0x.5 (0xAB>>1): shift 7 more bits; 0x.A (0xAB>>2): shift 6 more bits; else error.
				 can't check entire byte as high nibble contains bits from next one. error unhandled, will be caught by SYNC:
				 ch->nbits = ((r & 0xF) == 0x5) ? 7 : 6;
				 in fact, this can be rewritten without branch simply by considering the value of the first bit:
				 */
				ch->nbits = 6 | (r & 0x01);	// 6 or 7
				ch->count = 1;	// skip '+', check '*'
			}
			else if (l > 3)
				ch->nbits = l-3;	// shift enough bits to leave 2 '1' before the first zero
			else	// it looks like we've stopped dead on '+', jump into next state
				goto synced;
		}
		else {	// otherwise eat bytes until we get a sequence of more than 16 consecutive 0xFF or 0x00
			switch (r) {
			case 0x00:
				if (--ch->count <= -10) {
					// we really are hearing 0xFF, only inverted
					vprerr("#%d inverting polarity\n", ch->chn+1);
					ch->MskS ^= 2;	// inverted polarity
					ch->count = -ch->count;
				}
				break;
			case 0xFF:
				ch->count++;
				break;
			default:
				ch->count = 0;
				break;
			}
		}

		return;

	case SYNC:
synced:
		switch (ch->count) {
		case 0:	// expect '+' with parity bit set
			if (unlikely(('+'|0x80) != r)) {
				vprerr("#%d didn't get '+': %x\n", ch->chn+1, r);
				goto fail;
			}
			break;
		case 1:	// expect '*'
			if (unlikely('*' != r)) {
				vprerr("#%d didn't get '*': %x\n", ch->chn+1, r);
				goto fail;
			}
			break;
		case 3:	// expect SYN
			ch->Acarsstate = SOH1;
			// fallthrough
		case 2:	// expect SYN
			if (unlikely(SYN != r)) {
				vprerr("#%d didn't get SYN: %x\n", ch->chn+1, r);
				goto fail;
			}
			break;
		default:	// cannot happen
			goto fail;
		}
		ch->count++;
		return;

	case SOH1:
		if (likely(r == SOH)) {
			if (ch->blk == NULL) {
				ch->blk = malloc(sizeof(*ch->blk));
				if (unlikely(ch->blk == NULL)) {
					perror(NULL);
					break;	// fail
				}
			}
			gettimeofday(&(ch->blk->tv), NULL);
			ch->Acarsstate = TXT;
			ch->blk->chn = ch->chn;
			ch->blk->len = 0;
			ch->blk->err = 0;
			return;
		}
		vprerr("#%d didn't get SOH: %x\n", ch->chn+1, r);
		break;	// else fail

	case TXT:
		if (unlikely(ch->blk->len > 240)) {
			vprerr("#%d too long\n", ch->chn + 1);
			break;	// fail
		}
		ch->blk->txt[ch->blk->len++] = r;
		if (r == ETX || r == ETB) {
			ch->Acarsstate = CRC1;
			return;
		}
		if (unlikely(r == DEL && ch->blk->len > 20)) {
			vprerr("#%d miss txt end\n", ch->chn + 1);
			ch->blk->len -= 3;
			ch->blk->crc[0] = ch->blk->txt[ch->blk->len];
			ch->blk->crc[1] = ch->blk->txt[ch->blk->len + 1];
			ch->Acarsstate = END;
			goto putmsg_lbl;
		}
		if (unlikely(parity8(r) == 0)) {
			if (++ch->blk->err > MAXPERR + 1) {
				vprerr("#%d too many parity errors\n", ch->chn + 1);
				break;	// fail
			}
		}
		return;

	case CRC1:
		ch->blk->crc[0] = r;
		ch->Acarsstate = CRC2;
		return;

	case CRC2:
		ch->blk->crc[1] = r;
		ch->Acarsstate = END;
		return;

	case END:
		if (unlikely(r != DEL))
			vprerr("#%d didn't get DEL: %x\n", ch->chn+1, r);	// ignored

putmsg_lbl:
		ch->blk->lvl = 10 * log10(ch->MskLvl);

		vprerr("put message #%d\n", ch->chn + 1);

		pthread_mutex_lock(&blkq_mtx);
		ch->blk->prev = blkq_h;
		blkq_h = ch->blk;
		pthread_cond_signal(&blkq_wcd);
		pthread_mutex_unlock(&blkq_mtx);

		ch->blk = NULL;
		break;	// reset
	}

fail:
	resetAcars(ch);
	return;
}

int deinitAcars(void)
{
	pthread_mutex_lock(&blkq_mtx);
	acars_shutdown = 1;
	pthread_mutex_unlock(&blkq_mtx);
	pthread_cond_signal(&blkq_wcd);

	pthread_join(blkth_id, NULL);

	return 0;
}
