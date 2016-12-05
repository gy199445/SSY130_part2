/** @file Automatically generated wavetable. Generated by gen_wavetable.m */
#ifndef WAVEFORM_H_
#define WAVEFORM_H_
#include <stdint.h>
#include "config.h"

#if AUDIO_SAMPLE_RATE == 48000
extern const int16_t waveform[353856];
#endif

#if AUDIO_SAMPLE_RATE == 24000
extern const int16_t waveform[176928];
#endif

#if AUDIO_SAMPLE_RATE == 16000
extern const int16_t waveform[117952];
#endif

#endif