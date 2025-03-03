#ifndef FIR_DS_H
#define FIR_DS_H

//this defines buffer sizes, should be a power of two
const int MAX_TAPS = 512;


void scaleTaps(double* Coeffs, int* inCoeffs, int M, int bits);

class FIR_circ
{

public:
    FIR_circ();
    ~FIR_circ();

    void fillTaps(double * taps, int length);
    void filter(double * in, double * out1, int points_in);

private:
    int used_taps = MAX_TAPS;
    double coeffs[MAX_TAPS] {};
    double data_in[MAX_TAPS] {};
    int head = 0;
    double accum =0.0;

};


#endif // FIR_DS_H