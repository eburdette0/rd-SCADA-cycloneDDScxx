#include "FIR_ds.h"
#include <cmath>

//this defines buffer sizes, should be a power of two
//const int MAX_TAPS = 512;


void scaleTaps(double* Coeffs, int* inCoeffs, int M, int bits){
    double integral = 0.0;

    for (int i=0; i<M; i++){
        integral+= inCoeffs[i]/1.0;
    }
    integral/=std::pow(2,(bits-1));
    double scaling = (1.0/M)/(integral/M);

    for (int i=0; i<M; i++){
        Coeffs[i] = (inCoeffs[i])/(std::pow(2,(bits-1)))*scaling;
    }
}


FIR_circ::FIR_circ(){

};
FIR_circ::~FIR_circ(){
    
};


void FIR_circ::fillTaps(double *taps, int length){

    used_taps = length;
    for (int i = 0; i<length; i++){
        coeffs[i] = taps[i]; //write taps forwards
    }
}


void FIR_circ::filter(double * in, double * out1, int points_in){
    for (int i = 0; i<points_in; i++){
        data_in[(head+i) % MAX_TAPS] = in[i]; //write forwards
    }
    head = (head + points_in) % MAX_TAPS;

    accum = 0.0;
    for (int i=0; i<used_taps; i++){
        // Debug
        // data_indexed = data_in[(MAX_TAPS + (head-1) - i) % MAX_TAPS];
        // coeff_indexed = coeffs[i];
        // accum += data_indexed*coeff_indexed; //read data backwards, taps forwards
        accum += data_in[(MAX_TAPS + (head-1) - i) % MAX_TAPS]*coeffs[i]; //read data backwards, taps forwards
    }
    *out1= accum;

}
