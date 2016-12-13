#include "lab_lms.h"
#include "backend/arm_math.h"
#include "blocks/sources.h"
#include "blocks/sinks.h"
#include "util.h"
#include "config.h"
#include "backend/systime/systime.h"
#include "backend/printfn/printfn.h"

#define TEST 			(0)
#define NORMAL 		(1)
#define TESTMODE NORMAL

#if SYSMODE == SYSMODE_LMS

//#define statements for the LMS lab
#define 	LAB_LMS_MU_INIT		(1.0e-4f)
#define		LAB_LMS_MU_CHANGE	(1.2)
#if TESTMODE == TEST
#define LAB_LMS_TAPS  (8)
#else
#define 	LAB_LMS_TAPS		(64)
#endif
// Stateful variables for the LMS lab
float lms_coeffs[LAB_LMS_TAPS];
float lms_state[LAB_LMS_TAPS + AUDIO_BLOCKSIZE - 1];
float lms_mu_state = LAB_LMS_MU_INIT;
float lms_mu       = LAB_LMS_MU_INIT;
char pname[] = "h";

// define enum type for different modes of operation
enum lms_modes {lms_updt, lms_enbl, lms_dsbl } lms_mode;
enum dist_srces {cos_src, noise_src } dist_src;
enum signal_modes {signal_off, signal_on } signal_mode;

// Function declaration
void my_lms( float * y,
	float * x,
	float * xhat,
	float * e,
	int blockSize);

void lab_lms_init(void){
	//Manually initialize the LMS filter coefficients and state to all zeros
	arm_fill_f32(0.0f, lms_coeffs, NUMEL(lms_coeffs));
	arm_fill_f32(0.0f, lms_state, NUMEL(lms_state));
  lms_mode = lms_dsbl; // start with disabled mode
  dist_src = noise_src; // start with wide band noise
	signal_mode = signal_on;


#if TESTMODE == NORMAL
	printf("Usage guide;\n"
			"Press the following keys to change the system behavior\n"
			"\t'd' - LMS filtering disabled, raw microphone data output to right speaker. Initial mode.\n"
			"\t'f' - LMS filtering applied no filter update (mu=0), error signal output to right speaker\n"
      "\t'u' - LMS filtering applied with filter update, error signal output to right speaker\n"
      "\t't' - Toggle disturbance source between cosine signal and wide band noise\n"
			"\t'r' - Reset step size to %e and turn off filter updating\n"
			"\t'+' - Increase step size mu\n"
			"\t'-' - Decrease step size mu\n"
			"\t'p' - Prints the filter coefficients h in a format useful for import in Matlab\n"
			"\t's' - Toggles music signal\n", lms_mu);
#endif
#if TESTMODE == TEST
	float y_data[AUDIO_BLOCKSIZE];
	float x_data[AUDIO_BLOCKSIZE];
	float xhat_data[AUDIO_BLOCKSIZE];
	float e_data[AUDIO_BLOCKSIZE];
		blocks_sources_test_y(y_data);
		blocks_sources_test_x(x_data);
		int i;
		for( i=0; i<1000; i++){
			blocks_sources_test_y(y_data);
			blocks_sources_test_x(x_data);
			my_lms(y_data, x_data, xhat_data, e_data, AUDIO_BLOCKSIZE);
//		  arm_lms_f32(&lms_s, y_data, x_data, xhat_data, e_data, AUDIO_BLOCKSIZE);
		}
		printf("Correct values are \n"
		 "h=[0, 1, -1, 2, -4, 6, -8, 10]\n"
	   "Estimated vector: \n");
		print_vector_f(pname, lms_coeffs, LAB_LMS_TAPS);
	//	print_vector_f(pname, y_data, AUDIO_BLOCKSIZE);
		printf("Test finished. Halting execution!\n");
		while (true){};
#endif


}

void lab_lms(void){
  //Update filter settings
  char key;
  if(board_get_usart_char(&key)){
    switch(key){
    default:
      printf("Invalid key pressed.\n");
      break;
    case 'd':
      printf("Filtering disabled, mic signal output to speaker\n");
      lms_mode = lms_dsbl;
      break;
    case 'f':
      printf("Filtering enabled, error signal output to speaker\n");
      lms_mode = lms_enbl;
      lms_mu = 0.0f;
      printf("No updating. Step size mu set to %e\n", 0.0f);
      break;
    case 'u':
      printf("Filtering and lms update, error signal output to speaker\n");
      lms_mode = lms_updt;
      lms_mu = lms_mu_state;
      printf("Step size mu set to %e\n", lms_mu);
      break;
    case 'r':
      lms_mu_state = LAB_LMS_MU_INIT;
      lms_mu = lms_mu_state;
      printf("Filtering enabled, error signal output to speaker\n");
      printf("Step size mu reset to %e\n", lms_mu);
      break;
    case '+':
      lms_mu_state *= LAB_LMS_MU_CHANGE;
      lms_mu = lms_mu_state;
      printf("Step size mu increased to %e\n", lms_mu);
      break;
    case '-':
      lms_mu_state *= 1.0f/LAB_LMS_MU_CHANGE;
      lms_mu = lms_mu_state;
      printf("Step size mu decreased to %e\n", lms_mu);
      break;
    case 't':
      if (dist_src == cos_src){
        dist_src = noise_src;
        printf("Disturbance source set to noise\n");
      }else{
        dist_src = cos_src;
        printf("Disturbance source set to cosine\n");
      }
      break;
    case 'p':
      print_vector_f(pname, lms_coeffs, LAB_LMS_TAPS);
      break;
    case 's':
      if (signal_mode == signal_off){
	signal_mode = signal_on;
	printf("Signal source turned on\n");
      }else{
	signal_mode = signal_off;
	printf("Signal source turned off\n");
      }
      break;
    }
  }

  float outdata[AUDIO_BLOCKSIZE];
  float distdata[AUDIO_BLOCKSIZE];
  float signaldata[AUDIO_BLOCKSIZE];
  blocks_sources_waveform(signaldata); // Music signal
  if (dist_src == noise_src){ // wide band noise as disturbance
    blocks_sources_disturbance(distdata);
  } else { // cosine as disturbance
    blocks_sources_cos(distdata);
  };
  arm_scale_f32(distdata, 0.2f, distdata, AUDIO_BLOCKSIZE);
  if (signal_mode == signal_on){
    arm_add_f32(distdata,signaldata,outdata,AUDIO_BLOCKSIZE); // add signal and noise
    blocks_sinks_leftout(outdata); // Send to left channel
  }else{
    blocks_sinks_leftout(distdata); // Send to left channel
  }
  //Update the LMS filter
  // float *lms_input = distdata;
  float lms_mic[AUDIO_BLOCKSIZE];
  blocks_sources_microphone(lms_mic);
  float lms_output[AUDIO_BLOCKSIZE];
  float lms_err[AUDIO_BLOCKSIZE];
  if ((lms_mode == lms_enbl) || (lms_mode == lms_updt)){
    my_lms(distdata, lms_mic, lms_output, lms_err, AUDIO_BLOCKSIZE);
    blocks_sinks_rightout(lms_err); // Send cleaned signal to right channel
  }else{
    blocks_sinks_rightout(lms_mic);
  };
}

void my_lms(
	    float * y,
	    float * x,
	    float * xhat,
	    float * e,
	    int blockSize){
	/*
	y[] = vector of input signal of length blockSize
	x[] = vector of "desired" signal of length blockSize
	xhat[] = vector of filter output of length blockSize
	e[] = vector of x-xhat the error of length blockSize
	blockSize = the length of all vectors

	The function also use the global variables:
	lms_state[] = vector of the filter state with size  blockSize+LAB_LMS_TAPS-1
	lms_mu = the step-length of the LMS filter update.
	lms_coeffs[] = the vector of the filter coefficients h stored in reverse order
	*/

  // Copy indata into lms_state starting from index numTaps-1
  // pState has length blockSize+numTaps-1
  arm_copy_f32(y, &(lms_state[LAB_LMS_TAPS-1]), blockSize);

	// Loop variables
	int i,j;

	// Loop through block of samples
	for(i=0; i<blockSize; i++) {
		// Compute xhat(i) using dot product
		arm_dot_prod_f32(lms_coeffs, &(lms_state[i]), LAB_LMS_TAPS, &(xhat[i]));
		// Compute e(i) = x(i) - xhat(i)
		e[i] = x[i] - xhat[i];
		// Update lms_coeffs
		for(j=0; j<LAB_LMS_TAPS; j++) {
			lms_coeffs[j] += 2*lms_mu*lms_state[i + j]*e[i];
		}
	}

  // Place last numTaps-1 inpuy (y) samples first in state vector lms_state
  arm_copy_f32( &y[blockSize - (LAB_LMS_TAPS-1)], lms_state, LAB_LMS_TAPS-1);
};


#endif
