#include "sources.h"
#include "config.h"
#include <math.h>
#include "backend/hw/headphone.h"
#include "util.h"
#include "waveform.h"
#include "arm_math.h"

/** @brief One period of the secret "mystery" disturbance signal */
#if AUDIO_SAMPLE_RATE == 48000
const float dist_sig[96] = {0.000000e+00, 1.827277e-01, 3.585557e-01, 5.210407e-01, 6.646149e-01, 7.849329e-01, 8.791190e-01, 9.458957e-01, 9.855875e-01, 1.000000e+00, 9.921919e-01, 9.661606e-01, 9.264733e-01, 8.778768e-01, 8.249215e-01, 7.716310e-01, 7.212445e-01, 6.760503e-01, 6.373189e-01, 6.053363e-01, 5.795259e-01, 5.586413e-01, 5.410056e-01, 5.247697e-01, 5.081618e-01, 4.897029e-01, 4.683675e-01, 4.436777e-01, 4.157226e-01, 3.851086e-01, 3.528493e-01, 3.202119e-01, 2.885402e-01, 2.590770e-01, 2.328066e-01, 2.103351e-01, 1.918222e-01, 1.769702e-01, 1.650699e-01, 1.550957e-01, 1.458388e-01, 1.360584e-01, 1.246350e-01, 1.107062e-01, 9.376717e-02, 7.372670e-02, 5.091041e-02, 2.601219e-02, 1.670829e-16, -2.601219e-02, -5.091041e-02, -7.372670e-02, -9.376717e-02, -1.107062e-01, -1.246350e-01, -1.360584e-01, -1.458388e-01, -1.550957e-01, -1.650699e-01, -1.769702e-01, -1.918222e-01, -2.103351e-01, -2.328066e-01, -2.590770e-01, -2.885402e-01, -3.202119e-01, -3.528493e-01, -3.851086e-01, -4.157226e-01, -4.436777e-01, -4.683675e-01, -4.897029e-01, -5.081618e-01, -5.247697e-01, -5.410056e-01, -5.586413e-01, -5.795259e-01, -6.053363e-01, -6.373189e-01, -6.760503e-01, -7.212445e-01, -7.716310e-01, -8.249215e-01, -8.778768e-01, -9.264733e-01, -9.661606e-01, -9.921919e-01, -1.000000e+00, -9.855875e-01, -9.458957e-01, -8.791190e-01, -7.849329e-01, -6.646149e-01, -5.210407e-01, -3.585557e-01, -1.827277e-01};
#elif AUDIO_SAMPLE_RATE == 24000
const float dist_sig[48] = {0.000000e+00, 3.613774e-01, 6.698451e-01, 8.860373e-01, 9.933436e-01, 1.000000e+00, 9.337642e-01, 8.314132e-01, 7.269204e-01, 6.423343e-01, 5.840865e-01, 5.452631e-01, 5.121608e-01, 4.720534e-01, 4.189941e-01, 3.556261e-01, 2.908109e-01, 2.346387e-01, 1.933318e-01, 1.663689e-01, 1.469865e-01, 1.256159e-01, 9.450508e-02, 5.131105e-02, 1.683978e-16, -5.131105e-02, -9.450508e-02, -1.256159e-01, -1.469865e-01, -1.663689e-01, -1.933318e-01, -2.346387e-01, -2.908109e-01, -3.556261e-01, -4.189941e-01, -4.720534e-01, -5.121608e-01, -5.452631e-01, -5.840865e-01, -6.423343e-01, -7.269204e-01, -8.314132e-01, -9.337642e-01, -1.000000e+00, -9.933436e-01, -8.860373e-01, -6.698451e-01, -3.613774e-01};
#elif AUDIO_SAMPLE_RATE == 16000
const float dist_sig[32] = { 0.000000e+00, 5.210407e-01, 8.791190e-01, 1.000000e+00, 9.264733e-01, 7.716310e-01, 6.373189e-01, 5.586413e-01, 5.081618e-01, 4.436777e-01, 3.528493e-01, 2.590770e-01, 1.918222e-01, 1.550957e-01, 1.246350e-01, 7.372670e-02, 4.902874e-17, -7.372670e-02, -1.246350e-01, -1.550957e-01, -1.918222e-01, -2.590770e-01, -3.528493e-01, -4.436777e-01, -5.081618e-01, -5.586413e-01, -6.373189e-01, -7.716310e-01, -9.264733e-01, -1.000000e+00, -8.791190e-01, -5.210407e-01};
#else
#error disturbance signal not available at configured sample-rate
#endif

float h[8]={10,
 -8,
 6,
 -4,
 2,
 -1,
 1,
 0};
#define TEST_SAMPLES (64)
float test_y[TEST_SAMPLES]={2.02369089,
 -2.25835397,
 2.22944568,
 0.337563701,
 1.00006082,
 -1.66416447,
 -0.590034564,
 -0.278064164,
 0.422715691,
 -1.6702007,
 0.471634326,
 -1.2128472,
 0.0661900484,
 0.652355889,
 0.327059967,
 1.0826335,
 1.00607711,
 -0.650907737,
 0.257056157,
 -0.944377806,
 -1.32178852,
 0.924825933,
 4.98490753e-05,
 -0.0549189146,
 0.911127266,
 0.594583697,
 0.350201174,
 1.25025123,
 0.929789459,
 0.239763257,
 -0.690361103,
 -0.651553642,
 1.19210187,
 -1.61183039,
 -0.0244619366,
 -1.94884718,
 1.02049801,
 0.861716302,
 0.00116208348,
 -0.0708372132,
 -2.48628392,
 0.581172323,
 -2.19243492,
 -2.31928031,
 0.0799337103,
 -0.948480984,
 0.411490621,
 0.676977806,
 0.857732545,
 -0.691159125,
 0.449377623,
 0.10063335,
 0.826069998,
 0.53615708,
 0.897888426,
 -0.131937868,
 -0.147201456,
 1.00777341,
 -2.12365546,
 -0.504586406,
 -1.27059445,
 -0.382584803,
 0.648679262,
 0.825727149};
float test_x[TEST_SAMPLES]={14.8932485,
 -36.4125825,
 49.6098742,
 -35.4846256,
 33.5805224,
 -37.2492035,
 20.8040165,
 -15.8582675,
 13.4601244,
 -23.3828053,
 23.2106558,
 -29.2438718,
 20.4087478,
 -7.21076326,
 6.33646669,
 7.29170102,
 2.56950095,
 -9.34445857,
 9.55170621,
 -16.9394193,
 -0.260314187,
 11.902817,
 -9.38023172,
 7.48980224,
 4.40947625,
 0.554168547,
 2.18526666,
 10.4388343,
 0.895980308,
 1.28314079,
 -7.22703933,
 -0.528340904,
 12.9917571,
 -26.003008,
 21.7181535,
 -34.105149,
 34.4417038,
 -16.2093727,
 9.7912756,
 -5.11442585,
 -23.7706529,
 24.0264104,
 -41.0469766,
 8.4977257,
 -1.0262878,
 -11.6924303,
 14.0071515,
 -4.39772103,
 9.71108348,
 -15.6537233,
 14.3129292,
 -10.1725884,
 14.3665133,
 -4.00377257,
 11.6911611,
 -9.52909365,
 4.82707489,
 7.2190423,
 -27.5685094,
 17.9532188,
 -24.7068563,
 13.8368214,
 -1.46003582,
 7.97696244};


/** @brief The next element from wavetable based waveforms */
uint_fast32_t dist_idx = 0;
uint_fast32_t waveform_idx = 0;
uint_fast32_t test_y_idx = 0;
uint_fast32_t test_x_idx = 0;

/** @brief The current angles for sine/cosine sources */
float sin_ang = 0;
float cos_ang = 0;

/** @brief The current frequency to use for sine/cosine outputs, default to 1kHz */
float trig_freq = 1e3;

void blocks_sources_zeros(float * sample_block){
	arm_fill_f32(0.0f, sample_block, AUDIO_BLOCKSIZE);
}

void blocks_sources_ones(float * sample_block){
	arm_fill_f32(1.0f, sample_block, AUDIO_BLOCKSIZE);
}

void blocks_sources_trig_setfreq(float frequency){
	ATOMIC(trig_freq = frequency);
}

void blocks_sources_sin(float * sample_block){
	int_fast32_t i;
	float ang = sin_ang;
	volatile float freq;	//Declare volatile to ensure the math operations are not re-ordered into the atomic block
	ATOMIC(freq = trig_freq);
	const float delta_ang = M_TWOPI * freq / AUDIO_SAMPLE_RATE;
	for(i = 0; i < AUDIO_BLOCKSIZE; i++){
		sample_block[i] = arm_sin_f32(ang);
		ang += delta_ang;
		if(sin_ang > M_TWOPI){
			sin_ang -= M_TWOPI;
		}
	}
}

void blocks_sources_cos(float * sample_block){
	int_fast32_t i;
	float ang = sin_ang;
	volatile float freq;	//Declare volatile to ensure the math operations are not re-ordered into the atomic block
	ATOMIC(freq = trig_freq);
	const float delta_ang = M_TWOPI * freq / AUDIO_SAMPLE_RATE;
	for(i = 0; i < AUDIO_BLOCKSIZE; i++){
		sample_block[i] = arm_cos_f32(ang);
		ang += delta_ang;
		if(sin_ang > M_TWOPI){
			sin_ang -= M_TWOPI;
		}
	}
}

void blocks_sources_waveform(float * sample_block){
	int_fast32_t i;
	for(i = 0; i < AUDIO_BLOCKSIZE; i++){
		sample_block[i] = waveform[waveform_idx++] * (1.0f/INT16_MAX);
		if(waveform_idx > NUMEL(waveform)-1){
			waveform_idx = 0;
		}
	}
}

void blocks_sources_disturbance(float * sample_block){
	int_fast32_t i;
	for (i = 0; i < AUDIO_BLOCKSIZE; i++) {
		sample_block[i] = dist_sig[dist_idx++];
		if (dist_idx > NUMEL(dist_sig)-1) {
			dist_idx = 0;
		}
	}
}
void blocks_sources_test_x(float * sample_block){
	int_fast32_t i;
	for (i = 0; i < AUDIO_BLOCKSIZE; i++) {
		sample_block[i] = test_x[test_x_idx++];
		if (test_x_idx > NUMEL(test_x)-1) {
			test_x_idx = 0;
		}
	}
}
void blocks_sources_test_y(float * sample_block){
	int_fast32_t i;
	for (i = 0; i < AUDIO_BLOCKSIZE; i++) {
		sample_block[i] = test_y[test_y_idx++];
		if (test_y_idx > NUMEL(test_y)-1) {
			test_y_idx = 0;
		}
	}
}

void blocks_sources_microphone(float * sample_block){
	arm_copy_f32(processed_micdata, sample_block, AUDIO_BLOCKSIZE);
}

void blocks_sources_update(void){
	//Compute the relative change in angle between each sample for the requested frequency
	volatile float freq;	//Declare volatile to ensure the math operations are not re-ordered into the atomic block
	ATOMIC(freq = trig_freq);
	const float delta_ang = M_TWOPI * freq / AUDIO_SAMPLE_RATE;
	sin_ang += delta_ang * AUDIO_BLOCKSIZE;
	cos_ang += delta_ang * AUDIO_BLOCKSIZE;
}
