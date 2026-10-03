/*
 *  Copyright (c) 2026 Thibaut VARENE
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
 *   You should have received a copy of the GNU Library General Public
 *   License along with this library; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include "acarsdec.h"
#include "lib.h"

#define ERRPFX	"ERROR: IQFILE: "
#define MAXNSAMPLES 4096


static FILE *iqfp;
static enum iqfile_format { IQ_U8, IQ_S16 } iqfmt;

static int usage(void)
{
	fprintf(stderr,
		"iqfile input accepts as parameter:\n"
		" - a filename (optionally prefixed by 'file=') followed by a coma,\n"
		"   and the optional 'format=' argument; where\n"
		"   'format=' is one of 'U8', 'S16' (default: U8);\n"
		"examples:\n"
		"   'file=data.raw' to process unsigned 8-bit raw IQ data\n"
		"   'data.raw,format=S16' to process signed 16-bit raw IQ data\n"
		"\n"
		"NOTE: IQ sample rate must be a multiple of %d Hz.\n"
		"The rate multiplier must be specified via '-m'\n",
		INTRATE);

	return 1;
}

int initIqfile(char *optarg)
{
	char *format = NULL, *fname = NULL;
	struct params_s iqp[] = {
		{ .name = "format", .valp = &format, },
		{ .name = "file", .valp = &fname, },
	};
	char *retp, *sep;
	float scale = 127.5F;	// U8 default

	do {
		retp = parse_params(&optarg, iqp, ARRAY_SIZE(iqp));
		if (retp) {
			sep = strchr(retp, '=');
			// backward compat: if we find a lone token, assume it's the filename if not already set
			if (!sep && !fname)
				fname = retp;
			else {
				fprintf(stderr, ERRPFX "invalid parameter '%s'\n", retp);
				return -1;

			}
		}
	} while (retp);

	if (!fname) {
		fprintf(stderr, ERRPFX "missing filename\n");
		return -1;
	}

	if (!strcmp("help", fname))
		return usage();

	if (!R.rateMult){
		fprintf(stderr, ERRPFX "missing rate multiplier\n");
		return -1;
	}

	if (!R.Fc) {
		fprintf(stderr, ERRPFX "missing center frequency\n");
		return -1;
	}

	if (format) {
		if (!strcmp("U8", format)) {
			iqfmt = IQ_U8;
			scale = 127.5F;
		}
		else if (!strcmp("S16", format)) {
			iqfmt = IQ_S16;
			scale = 32768.0F;
		}
		else {
			fprintf(stderr, ERRPFX "invalid format value '%s'\n", format);
			return -1;
		}
	}

	iqfp = fopen(fname, "r");
	if (!iqfp) {
		fprintf(stderr, ERRPFX "could not open %s: %s\n", fname, strerror(errno));
		return 1;
	}

	return channels_init_sdr(R.Fc, R.rateMult, scale);
}


static void process_samples(const void *buf, size_t nread)
{
	const unsigned int mult = R.rateMult;
	float complex phasors[mult];

	if (unlikely(nread % 2)) {
		fprintf(stderr, ERRPFX "incomplete read\n");
		return;
	}

	while (nread) {
		unsigned int ind, lim = unlikely(nread / 2 < mult) ? nread / 2 : mult;
		float i, q;

		switch (iqfmt) {
			case IQ_U8:
				for (ind = 0; ind < lim; ind++) {
					i = (float)(*(uint8_t *)buf++) - 127.5F;
					q = (float)(*(uint8_t *)buf++) - 127.5F;
					phasors[ind] = i + q * I;
				}
				break;
			case IQ_S16:
				for (ind = 0; ind < lim; ind++) {
					i = (float)(*(int16_t *)buf++);
					q = (float)(*(int16_t *)buf++);
					phasors[ind] = i + q * I;
				}
				break;
			default:
				return;
		}

		channels_mix_phasors(phasors, lim, mult);
		nread -= lim * 2;
	}
}

int runIqfileSample(void)
{
	size_t nread, ssize, bufsize = MAXNSAMPLES * 2;
	void *buf;

	switch (iqfmt) {
		case IQ_U8:
			ssize = sizeof(uint8_t);
			break;
		case IQ_S16:
			ssize = sizeof(int16_t);
			break;
		default:
			return -1;
	}

	buf = malloc(bufsize * ssize);
	if (!buf) {
		perror(ERRPFX "runIqfileSample()");
		return -1;
	}

	do {
		nread = fread(buf, ssize, bufsize, iqfp);
		if (!nread)
			goto out;

		process_samples(buf, nread);
	} while (R.running);

out:
	free(buf);
	fclose(iqfp);
	return 0;
}
