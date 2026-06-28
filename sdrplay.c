/*
 *  Copyright (c) 2015 Thierry Leconte
 *  Copyright (c) 2025 Franco Venturi
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
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "acarsdec.h"
#include "lib.h"
#include <sdrplay_api.h>


// SDRPlay API 3.14
#ifndef SDRPLAY_RSPdxR2_ID
#define SDRPLAY_RSPdxR2_ID (SDRPLAY_RSPdx_ID)
#endif

// choose 252 as the 12k (INTRATE) multiplier because 12k * 252 = 3024Msps
// (which is reasonable)
// the specific reason for 252 is that the SDRplay API RX callback size
// (numSamples) is 1008, which is an integer multiple of 252, and that is
// supposed to improve performance with acarsdec
#define SDRPLAY_MULT 252U

#define ERRPFX	"ERROR: SDRplay: "
#define WARNPFX	"WARNING: SDRplay: "

static unsigned int Fc;
static sdrplay_api_DeviceT device;

int initSdrplay(char *optarg)
{
	int r;
	sdrplay_api_DeviceT devices[4];
	sdrplay_api_ErrT err;

	if (!optarg)
		return 1;	// cannot happen with getopt()

	char *serialNumber = NULL;
	int rspSequenceNumber = -1;
	if (strlen(optarg) == 10 && strspn(optarg, "0123456789ABCDEF") == 10) {
		serialNumber = optarg;
	} else if (strlen(optarg) == 1 && strspn(optarg, "0123456789") == 1) {
		rspSequenceNumber = atoi(optarg);
	} else {
		fprintf(stderr, ERRPFX "invalid RSP serial number or sequence number (0..3): %s\n", optarg);
		return 2;
	}

	if (!R.rateMult)
		R.rateMult = SDRPLAY_MULT;
	unsigned int sr = INTRATE * R.rateMult;
	int decimation = 1;
	while (sr < 2000000 && decimation <= 32) {
		sr *= 2;
		decimation *= 2;
	}

	if (!R.gRdB)
		R.gRdB = -100;	// AGC

	if (!R.lnaState)
		R.lnaState = 2;

	Fc = find_centerfreq(R.minFc, R.maxFc, R.rateMult);
	if (Fc == 0)
		return 5;

	r = channels_init_sdr(Fc, R.rateMult, 32768.0F);
	if (r)
		return r;

	/* open SDRplay API and check version */
	err = sdrplay_api_Open();
	if (err != sdrplay_api_Success) {
		fprintf(stderr, ERRPFX "Open failed: %s\n", sdrplay_api_GetErrorString(err));
		return -1;
	}
	float ver;
	err = sdrplay_api_ApiVersion(&ver);
	if (err != sdrplay_api_Success) {
		fprintf(stderr, ERRPFX "ApiVersion failed: %s\n", sdrplay_api_GetErrorString(err));
		sdrplay_api_Close();
		return -2;
	}	   
	if (ver != SDRPLAY_API_VERSION) {
		fprintf(stderr, ERRPFX "API version mismatch - expected=%.2f found=%.2f\n", SDRPLAY_API_VERSION, ver);
		sdrplay_api_Close();
		return -3;
	}

	/* select device */
	err = sdrplay_api_LockDeviceApi();
	if (err != sdrplay_api_Success) {
		fprintf(stderr, ERRPFX "LockDeviceApi failed: %s\n", sdrplay_api_GetErrorString(err));
		sdrplay_api_Close();
		return -4;
	}
	unsigned int ndevices = sizeof(devices) / sizeof(devices[0]);
	err = sdrplay_api_GetDevices(devices, &ndevices, ndevices);
	if (err != sdrplay_api_Success) {
		fprintf(stderr, ERRPFX "GetDevices failed: %s\n", sdrplay_api_GetErrorString(err));
		sdrplay_api_UnlockDeviceApi();
		sdrplay_api_Close();
		return -5;
	}
	int deviceIndex = -1;
	if (serialNumber != NULL) {
		for (unsigned int i = 0; i < ndevices; i++) {
			if (devices[i].valid &&
			   (strcmp(devices[i].SerNo, serialNumber) == 0)) {
				deviceIndex = i;
				break;
			}
		}
	} else if (rspSequenceNumber >= 0) {
		if (ndevices > rspSequenceNumber)
			deviceIndex = rspSequenceNumber;
	}
	if (deviceIndex == -1) {
		fprintf(stderr, ERRPFX "RSP not found or not available\n");
		sdrplay_api_UnlockDeviceApi();
		sdrplay_api_Close();
		return -6;
	}
	device = devices[deviceIndex];

	/* for RSPduo make sure single tuner mode is available */
	if (device.hwVer == SDRPLAY_RSPduo_ID) {
		if ((device.rspDuoMode & sdrplay_api_RspDuoMode_Single_Tuner) != sdrplay_api_RspDuoMode_Single_Tuner) {
			fprintf(stderr, ERRPFX "RSPduo single tuner mode not available\n");
			sdrplay_api_UnlockDeviceApi();
			sdrplay_api_Close();
			return -7;
		}
		device.rspDuoMode = sdrplay_api_RspDuoMode_Single_Tuner;
		if (R.antenna != NULL) {
			if (strcmp(R.antenna, "Tuner 1 50 ohm") == 0 || strcmp(R.antenna, "High Z") == 0) {
				device.tuner = sdrplay_api_Tuner_A;
			} else if (strcmp(R.antenna, "Tuner 2 50 ohm") == 0) {
				device.tuner = sdrplay_api_Tuner_B;
			} else {
				device.tuner = sdrplay_api_Tuner_A;
			}
		}
		device.rspDuoSampleFreq = 0;
	}

	err = sdrplay_api_SelectDevice(&device);
	if (err != sdrplay_api_Success) {
		fprintf(stderr, ERRPFX "SelectDevice failed: %s\n", sdrplay_api_GetErrorString(err));
		sdrplay_api_UnlockDeviceApi();
		sdrplay_api_Close();
		return -8;
	}

	err = sdrplay_api_UnlockDeviceApi();
	if (err != sdrplay_api_Success) {
		fprintf(stderr, ERRPFX "UnlockDeviceApi failed: %s\n", sdrplay_api_GetErrorString(err));
		sdrplay_api_ReleaseDevice(&device);
		sdrplay_api_Close();
		return -9;
	}

	/* select device settings */
	sdrplay_api_DeviceParamsT *device_params;
	err = sdrplay_api_GetDeviceParams(device.dev, &device_params);
	if (err != sdrplay_api_Success) {
		fprintf(stderr, ERRPFX "GetDeviceParams failed: %s\n", sdrplay_api_GetErrorString(err));
		sdrplay_api_ReleaseDevice(&device);
		sdrplay_api_Close();
		return -10;
	}

	device_params->devParams->mode = sdrplay_api_BULK;

	sdrplay_api_RxChannelParamsT *rx_channel_params = device_params->rxChannelA ;
	device_params->devParams->fsFreq.fsHz = (double)sr;
	rx_channel_params->ctrlParams.decimation.enable = decimation > 1;
	rx_channel_params->ctrlParams.decimation.decimationFactor = decimation;
	rx_channel_params->tunerParams.ifType = sdrplay_api_IF_Zero;
	if (R.gRdB == -100) {
		rx_channel_params->ctrlParams.agc.enable = sdrplay_api_AGC_100HZ;
	} else {
		rx_channel_params->ctrlParams.agc.enable = sdrplay_api_AGC_DISABLE;
		rx_channel_params->tunerParams.gain.gRdB = R.gRdB;
	}
	rx_channel_params->tunerParams.gain.LNAstate = R.lnaState;
	device_params->devParams->ppm = (double)(R.ppm);
	rx_channel_params->tunerParams.rfFreq.rfHz = Fc;
	if (R.antenna != NULL) {
		int antennaOK = 0;
		if (device.hwVer == SDRPLAY_RSP2_ID) {
			if (strcmp(R.antenna, "Antenna A") == 0) {
				antennaOK = 1;
				rx_channel_params->rsp2TunerParams.antennaSel = sdrplay_api_Rsp2_ANTENNA_A;
				rx_channel_params->rsp2TunerParams.amPortSel = sdrplay_api_Rsp2_AMPORT_2;
			} else if (strcmp(R.antenna, "Antenna B") == 0) {
				antennaOK = 1;
				rx_channel_params->rsp2TunerParams.antennaSel = sdrplay_api_Rsp2_ANTENNA_B;
				rx_channel_params->rsp2TunerParams.amPortSel = sdrplay_api_Rsp2_AMPORT_2;
			} else if (strcmp(R.antenna, "Hi-Z") == 0) {
				antennaOK = 1;
				rx_channel_params->rsp2TunerParams.antennaSel = sdrplay_api_Rsp2_ANTENNA_A;
				rx_channel_params->rsp2TunerParams.amPortSel = sdrplay_api_Rsp2_AMPORT_1;
			}
		} else if (device.hwVer == SDRPLAY_RSPduo_ID) {
			if (strcmp(R.antenna, "High Z") == 0) {
				antennaOK = 1;
				rx_channel_params->rspDuoTunerParams.tuner1AmPortSel = sdrplay_api_RspDuo_AMPORT_1;
			} else {
				antennaOK = 1;
				rx_channel_params->rspDuoTunerParams.tuner1AmPortSel = sdrplay_api_RspDuo_AMPORT_2;
			}
		} else if (device.hwVer == SDRPLAY_RSPdx_ID || device.hwVer == SDRPLAY_RSPdxR2_ID) {
			if (strcmp(R.antenna, "Antenna A") == 0) {
				antennaOK = 1;
				device_params->devParams->rspDxParams.antennaSel = sdrplay_api_RspDx_ANTENNA_A;
			} else if (strcmp(R.antenna, "Antenna B") == 0) {
				antennaOK = 1;
				device_params->devParams->rspDxParams.antennaSel = sdrplay_api_RspDx_ANTENNA_B;
			} else if (strcmp(R.antenna, "Antenna C") == 0) {
				antennaOK = 1;
				device_params->devParams->rspDxParams.antennaSel = sdrplay_api_RspDx_ANTENNA_C;
			}
		}
		if (!antennaOK) {
			fprintf(stderr, ERRPFX "invalid antenna: %s\n", R.antenna);
			sdrplay_api_ReleaseDevice(&device);
			sdrplay_api_Close();
			return -11;
		}
	}

	if (R.bias) {
		if (device.hwVer == SDRPLAY_RSP1A_ID || device.hwVer == SDRPLAY_RSP1B_ID) {
			rx_channel_params->rsp1aTunerParams.biasTEnable = R.bias;
		} else if (device.hwVer == SDRPLAY_RSP2_ID) {
			rx_channel_params->rsp2TunerParams.biasTEnable = R.bias;
		} else if (device.hwVer == SDRPLAY_RSPduo_ID) {
			rx_channel_params->rspDuoTunerParams.biasTEnable = R.bias;
		} else if (device.hwVer == SDRPLAY_RSPdx_ID || device.hwVer == SDRPLAY_RSPdxR2_ID) {
			device_params->devParams->rspDxParams.biasTEnable = R.bias;
		}
	}

	if (R.gRdB == -100)
		fprintf(stderr, "SDRplay device selects freq %d and sets autogain and LNA state %d\n", Fc, R.lnaState);
	else
		fprintf(stderr, "SDRplay device selects freq %d and sets IF gain reduction %d and LNA state %d\n",
			Fc, R.gRdB, R.lnaState);

	return 0;
}

static void sdrplayRXCallback(short *xi,
			      short *xq,
			      sdrplay_api_StreamCbParamsT *params,
			      unsigned int numSamples,
			      unsigned int reset,
			      void *cbContext);
void sdrplayEventCallback(sdrplay_api_EventT eventId,
			  sdrplay_api_TunerSelectT tuner,
			  sdrplay_api_EventParamsT *params,
			  void *cbContext);

int runSdrplaySample(void)
{
	sdrplay_api_ErrT err;

	sdrplay_api_CallbackFnsT callbackFns = {
		sdrplayRXCallback,
		NULL,
		sdrplayEventCallback
	};

	err = sdrplay_api_Init(device.dev, &callbackFns, NULL);
	if (err != sdrplay_api_Success) {
		fprintf(stderr, ERRPFX "Init failed: %s\n", sdrplay_api_GetErrorString(err));
		sdrplay_api_ReleaseDevice(&device);
		sdrplay_api_Close();
		return -1;
	}

	while (R.running)
		sleep(2);

	err = sdrplay_api_Uninit(device.dev);
	if (err != sdrplay_api_Success) {
		fprintf(stderr, ERRPFX "Uninit failed: %s\n", sdrplay_api_GetErrorString(err));
		sdrplay_api_ReleaseDevice(&device);
		sdrplay_api_Close();
		return -2;
	}

	return 0;
}

static unsigned int nextSampleNum = 0xffffffff;

static void sdrplayRXCallback(short *xi,
			      short *xq,
			      sdrplay_api_StreamCbParamsT *params,
			      unsigned int numSamples,
			      unsigned int reset,
			      void *cbContext)
{
	(void)reset;
	(void)cbContext;

	/* check for dropped samples */
	if (nextSampleNum != 0xffffffff && params->firstSampleNum != nextSampleNum) {
		unsigned int dropped_samples;
		if (nextSampleNum < params->firstSampleNum) {
			dropped_samples = params->firstSampleNum - nextSampleNum;
		} else {
			dropped_samples = UINT_MAX - (params->firstSampleNum - nextSampleNum) + 1;
		}
		fprintf(stderr, WARNPFX "dropped %d samples\n", dropped_samples);
	}
	nextSampleNum = params->firstSampleNum + numSamples;

	float complex phasors[R.rateMult];
	unsigned int i, j, lim;

	j = 0;
	while (numSamples) {
		/* mult-sized chunks */
		lim = numSamples < R.rateMult ? numSamples : R.rateMult;
		for (i = 0; i < lim; i++, j++) {
			float r = ((float)(xi[j]));
			float g = ((float)(xq[j]));
			phasors[i] = r + g * I;
		}
		channels_mix_phasors(phasors, lim, R.rateMult);
		numSamples -= lim;
	}
}

void sdrplayEventCallback(sdrplay_api_EventT eventId,
			  sdrplay_api_TunerSelectT tuner,
			  sdrplay_api_EventParamsT *params,
			  void *cbContext)
{
	(void)cbContext;

	if (eventId == sdrplay_api_PowerOverloadChange) {
		switch (params->powerOverloadParams.powerOverloadChangeType) {
		case sdrplay_api_Overload_Detected:
			fprintf(stderr, WARNPFX "power overload detected event\n");
			break;
		case sdrplay_api_Overload_Corrected:
			fprintf(stderr, WARNPFX "power overload correct event\n");
			break;
		}
		sdrplay_api_Update(device.dev, tuner, sdrplay_api_Update_Ctrl_OverloadMsgAck, sdrplay_api_Update_Ext1_None);
	}
	return;
}

int runSdrplayClose(void)
{
	sdrplay_api_ErrT err;

	err = sdrplay_api_ReleaseDevice(&device);
	if (err != sdrplay_api_Success) {
		fprintf(stderr, ERRPFX "ReleaseDevice failed: %s\n", sdrplay_api_GetErrorString(err));
		sdrplay_api_Close();
		return -1;
	}
	err = sdrplay_api_Close();
	if (err != sdrplay_api_Success) {
		fprintf(stderr, ERRPFX "Close failed: %s\n", sdrplay_api_GetErrorString(err));
		return -2;
	}
	return 0;
}
