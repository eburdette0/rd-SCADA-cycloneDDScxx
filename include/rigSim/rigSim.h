#ifndef RIGSIM_H
#define RIGSIM_H

class Machine{

public:
    Machine();

    double PcPumpUp(double timestep);
    double PcVent(double timestep);
    void axialDerivs(double t, double* y, double* dydt, double v_LP);
    void RKSolver(double t0, double* y0, double v_LP, double step_size, int steps);
    double axialRam(double input, double timestep);
    double calcSealFriction();
    int setInputChannels(int Load, int Pc, int LVDT);
    int setSampleBehavior(int viscous, double u0, double a, double b, double dc);
    int updateState(double * input, double * output, bool * relayOutput, double timestep);
    int resetState();


    double PConfining = 0.0;
    double ExternalLoad = 0.0;
    double InternalLoad = 0.0;
    double ExternalDisp = 0.0;
    double InternalDisp = 0.0;
    double SampleDisp = 0.0;
    double SampleShear = 0.0;
    double SampleDiameter =25.4; //mm
    double HitGap = 1.0; //mm


private:
    int delay_loopback = 0; //samples
    double frameK = 2000; // MPa/mm
    double internalK = 1000;
    double dExternalDisp = 0.0;
    double dInternalDisp = 0.0;
    double u0 = 0.6; //no sample deformation for now
    double sample_a = 0.0;
    double sample_b = 0.0;
    double sample_dc = 0.0;
    double sealFriction = 0.001;
    double RamDirection = 0.0;


    // Define the variables
    double axialSystem[3] = {0.0, 0.0, 0.0}; //h, x_seal, v_LP
    double k_0 = 3;
    double k_1 = 2;
    double k_3 = .1;

    double P = 10;
    double mu = 0.02;
    double visc = 1e0;
    double h0=1.0;


};

#endif
