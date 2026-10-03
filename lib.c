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
 *   You should have received a copy of the GNU Library General Public
 *   License along with this library; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <complex.h>
#include <sysexits.h>
#include <err.h>

#include "acarsdec.h"
#include "lib.h"
#include "msk.h"
#if defined(DEBUG) && defined(WITH_SNDFILE)
 #include "soundfile.h"
#endif

/**
 * Interval defines how often the arbitrary rate remixer renormalizes the
 * channel state. This prevents the complex multiplies from drifting in
 * magnitude over time. As a power of two, it allows use of an bit op rather
 * than a modulo.
 */
#define MIX_RENORM_INTERVAL 1024U

/**
 * Return the minimum sample rate  suitable to cover the target frequency range.
 * @param minFc lowest frequency in the range
 * @param maxFc highest frequency in the range
 * @return suitable sample rate
 */
unsigned int min_samplerate(unsigned int minFc, unsigned int maxFc)
{
	return ((maxFc - minFc) + 4 * INTRATE);
}

/**
 * Return the minimum sample rate multiplier suitable to cover the target frequency range.
 * @param minFc lowest frequency in the range
 * @param maxFc highest frequency in the range
 * @return suitable multiplier
 */
unsigned int min_multiplier(unsigned int minFc, unsigned int maxFc)
{
	// default min multiplier - see find_centerfreq() for computation margins applied
	return (min_samplerate(minFc, maxFc) + INTRATE) / INTRATE;
}

unsigned int find_centerfreq_rate(unsigned int minFc, unsigned int maxFc, unsigned int input_rate)
{
	if (!minFc) {
		fprintf(stderr, "ERROR: need a least one input frequency\n");
		return 0;
	}

	if ((maxFc - minFc) > input_rate - 4 * INTRATE) {
		fprintf(stderr, "ERROR: input frequencies too far apart\n");
		return 0;
	}

	if (R.Fc)
		return R.Fc;

	return (maxFc + minFc) / 2;
}

unsigned int find_centerfreq(unsigned int minFc, unsigned int maxFc, unsigned int multiplier)
{
	return (find_centerfreq_rate(minFc, maxFc, multiplier * INTRATE));
}

/**
 * Allocate and init the internal oscillators.
 * @param Fc the chosen center frequency
 * @param multiplier the chosen oversampling multiplier
 * @param scale the phasor scale (e.g. 1.0F for complex float32 phasors; 32768.0F for int16 phasors, ...)
 */
int channels_init_sdr(unsigned int Fc, unsigned int multiplier, float scale)
{
	unsigned int n, ind;
	float correctionPhase;

	for (n = 0; n < R.nbch; n++) {
		channel_t *ch = &R.channels[n];

		ch->oscillator = malloc(multiplier * sizeof(*ch->oscillator));
		ch->dm_buffer = malloc(DMBUFSZ * sizeof(*ch->dm_buffer));
		if (ch->oscillator == NULL || ch->dm_buffer == NULL) {
			perror(NULL);
			return 1;
		}

		/* precompute a scaled, oversampled local INTRATE oscillator per channel
		 mixing this oscillator with the received full-scale oversampled signal
		 will provide a normalized signal at the channel frequency */
		correctionPhase = (signed)(ch->Fr - Fc) / (float)(INTRATE * multiplier) * (float)(2 * M_PI);
		vprerr("#%d: Fc = %uHz, Fr = %uHz, phase = % f (%+dHz)\n",
		       n+1, Fc, ch->Fr, correctionPhase, (signed)(ch->Fr - Fc));
		for (ind = 0; ind < multiplier; ind++)
			ch->oscillator[ind] = cexpf(correctionPhase * ind * -I) / multiplier / scale;
	}

	return 0;
}

/**
 * Allocate and init the internal oscillators - resampling version.
 * @param Fc the chosen center frequency
 * @param input_rate the chosen sampling rate
 * @param scale the phasor scale (e.g. 1.0F for complex float32 phasors; 32768.0F for int16 phasors, ...)
 * @note falls back to channels_init_sdr() if input_rate is an integer multiple of INTRATE
 */
int channels_init_sdr_resample(unsigned int Fc, unsigned int input_rate, float scale)
{
	unsigned int n;
	float scale_factor;

	if (!input_rate)
		return 1;

	if (!(input_rate % INTRATE))
		return channels_init_sdr(Fc, input_rate / INTRATE, scale);

	scale_factor = ((float)INTRATE / (float)input_rate) / scale;

	for (n = 0; n < R.nbch; n++) {
		channel_t *ch = &R.channels[n];
		float correctionPhase;

		ch->dm_buffer = malloc(DMBUFSZ * sizeof(*ch->dm_buffer));
		if (ch->dm_buffer == NULL) {
			perror(NULL);
			return 1;
		}

		correctionPhase = (signed)(ch->Fr - Fc) / (float)input_rate * (float)(2 * M_PI);
		vprerr("#%d: Fc = %uHz, Fr = %uHz, phase = % f (%+dHz)\n",
		       n + 1, Fc, ch->Fr, correctionPhase, (signed)(ch->Fr - Fc));

		ch->mix_phase = scale_factor;
		ch->mix_step = cexpf(correctionPhase * -I);
		ch->mix_count = 0;
	}

	return 0;
}

/**
 * Compute the magnitude of the oversampled signal and update the channel buffer.
 * @param D an R.nbch-wide array of oversampled I/Q pairs
 * @note zeroes input
 */
static void channels_push_and_demod_sample(float complex *restrict D)
{
	static unsigned int counter = 0;
	const unsigned int nbch = R.nbch;
	unsigned int n;

	// each dm_buffer sample is made of a the magnitude of a rateMult-oversampled I/Q pair.
	// dm-buffer is rateMult-downsampled
	for (n = 0; n < nbch; n++) {
		R.channels[n].dm_buffer[counter] = cabsf(D[n]);
		D[n] = 0;
	}

	if (++counter >= DMBUFSZ) {
		for (n = 0; n < nbch; n++)
			demodMSK(&R.channels[n], DMBUFSZ);
		counter = 0;

#if defined(DEBUG) && defined(WITH_SNDFILE)
		SndWrite(DMBUFSZ);
#endif
	}
}

/**
 * Mix an array of oversampled phasors with each channel local oscillator to retrieve the signal at that channel frequency.
 * Invoke channels_push_and_demod_sample() when enough data has been accumulated.
 *
 * @param phasors an array of oversampled phasors
 * @param len the length of the phasors array (need not be a multiple of oversampling multiplier)
 * @param multiplier the oversampling multiplier
 * @note can process an arbitrary number of phasors, perf benefits are obtained if #len is always an exact multiple of #multiplier,
 * or if #len is larger than #multiplier *2
*
 * Theory of operation: together with channels_push_and_demod_sample():
 * For each channel, mix the oversampled full-scale phasor with channel downscaled oversampled
 * local oscillator and sum (integrate) the result rateMult times: this gives us a normalized complex signal
 * at the local oscillator freq. Then compute the magnitude of the resulting signal,
 * which is the magnitude of the signal received at that local oscillator freq
 */
void channels_mix_phasors(const float complex *restrict phasors, unsigned int len, const unsigned int multiplier)
{
	static float complex *restrict D = NULL;
	static unsigned int ind = 0;
	const unsigned int nbch = R.nbch;
	unsigned int n, k = 0;
	float complex d, *restrict oscillator;

	if (unlikely(!len))
		return;

	if (unlikely(!D)) {
		D = calloc(nbch, sizeof(*D));
		if (!D)
			err(EX_OSERR, NULL);
	}

	// realign to multiplier stride if necessary
	if (unlikely(ind)) {
		for (k = 0; ind < multiplier && k < len; k++, ind++) {
			for (n = 0; n < nbch; n++)
				D[n] += phasors[k] * R.channels[n].oscillator[ind];
		}
		if (likely(multiplier == ind)) {
			channels_push_and_demod_sample(D);
			ind = 0;
		}
	}

	len -= k;
	phasors += k;

	// here either (ind==0 and len>=0) or len==0

	// then use a vectorized loop for the remainder of the buffer
	while (len) {
		unsigned int lim = unlikely(len < multiplier) ? len : multiplier;	// process multiplier-sized chunks
		for (n = 0; n < nbch; n++) {
			oscillator = R.channels[n].oscillator;
			for (d = 0, k = 0; k < lim; k++)		// vectorizable
				d += phasors[k] * oscillator[k];
			D[n] = d;	// update static variable outside of loop
		}
		if (likely(multiplier == lim))
			channels_push_and_demod_sample(D);
		else	// partial write
			ind = lim;
		len -= lim;
		phasors += lim;
	}
}

/**
 * This function handles sample rates that are not clean integer multiples 
 * of 12 kHz, specifically for the AirSpy R2 which has rates of 2.5MS/s and
 * 10 MS/s. It breaks out the mixer phase and 12 khZ output timing 
 * explicitly. Similar to the channels_mix_phasors function, once enough
 * data has been processed, the samples are sent to the push and demod function.
 *
 * The mix_phase is periodically renormalized to prevent small floating
 * point errors from building up and destroying the signal.  
 *
 * In plain terms, it takes the channel to baseband, one input sample at
 * a time. While doing that, enough shifted samples are added to produce
 * a single output sample at the decoder rate, then passed to the demod, which
 * stores magnitude and eventually feeds the MSK demod.
 *
 * @param phasors An array of input I/Q samples from the SDR
 * @param len The number of complex samples in the array.
 * @param input_rate The sample rate in samples per second.
 * @note this implementation is an order of magnitude more CPU-intensive than
 * the regular `channels_mix_phasors()` implementation and should only be used
 * for inputs that do not support sample rates which are multiples of INTRATE.
 * @note this function, like its init() counterpart, falls back to channels_mix_phasors()
 * if the provided samplerate is an integer multiple of INTRATE.
 */
void channels_mix_phasors_resample(const float complex *restrict phasors, unsigned int len, unsigned int input_rate)
{
	static float complex *restrict D = NULL;
	static unsigned int phase_acc = 0;
	const unsigned int nbch = R.nbch;
	unsigned int i, n;

	if (unlikely(!len))
		return;

	if (!(input_rate % INTRATE))
		return channels_mix_phasors(phasors, len, input_rate / INTRATE);

	if (unlikely(!D)) {
		// Allocate one accumlator per channel
		D = calloc(nbch, sizeof(*D));
		if (!D)
			err(EX_OSERR, NULL);
	}

	for (i = 0; i < len; i++) {
		// Process one i/q sample at the device sample rate.
		float complex sample = phasors[i];

		for (n = 0; n < nbch; n++) {
			channel_t *ch = &R.channels[n];
			float complex mixed;
			float phase_mag;

			// Shift this channel from its RF offset down to baseband
			// and add the result into the current output accumulator.
			mixed = sample * ch->mix_phase;

#if defined(DEBUG)
			// The below can be used to debug if the renormalization isn't working
			// correctly or if the data is bad (the sample is NaN or inf).
			if (!isfinite(crealf(mixed)) || !isfinite(cimagf(mixed))) {
				vprerr("WARNING: MIX: resetting non-finite mixed sample on channel #%u\n", n + 1);
				ch->mix_phase = cabsf(ch->mix_phase) > 0.0F ? ch->mix_phase / cabsf(ch->mix_phase) * ((float)INTRATE / (float)input_rate) : ((float)INTRATE / (float)input_rate);
				ch->mix_count = 0;
				mixed = 0.0F;
			}
#endif
			D[n] += mixed;

			// Advance the oscillator so the next sample uses the correct
			// phase for this channels frequency offset.
			ch->mix_phase *= ch->mix_step;

			// Every MIX_RENORM_INTERVAL, renormalize the phase magnitude, preventing
			// drift towards zero/infinity. Floating point roundoff can slowly change
			// the magnitude of mix_phase. This keeps the oscillator stable. Prior to
			// implementing this check, after several minutes of runtime, the mix_phase
			// could/would trend towards zero/inf.
			if ((++ch->mix_count & (MIX_RENORM_INTERVAL - 1U)) == 0) {
				phase_mag = cabsf(ch->mix_phase);
				ch->mix_phase /= phase_mag;
				ch->mix_phase *= ((float)INTRATE / (float)input_rate);
			}
		}

		// Convert the arbitrary rate into a steady 12kHz output,
		// and when enough has accumulated, emit a decoder-rate
		// sample to the demodulator.
		phase_acc += INTRATE;
		if (phase_acc >= input_rate) {
			phase_acc -= input_rate;
			channels_push_and_demod_sample(D);
		}
	}
}
