#include "rigSim.h"
#include <math.h>

#define STEPS 20
#define Y_SIZE 3

//solving with hit material, close to real behavior

Machine::Machine(){
}

double Machine::PcPumpUp(double timestep){
    PConfining += 0.1*timestep*(100.0/(100.0+PConfining))*timestep;
    return 1;
}

double Machine::PcVent(double timestep){
    PConfining += 0.1*timestep*((100.0+PConfining)/100.0);
    return 1;
}


void Machine::axialDerivs(double t, double* y, double* dydt, double v_LP){
    double deltah = y[0];
    double x_seal = y[1];
    double x_lp = y[2];
    
    double x_face = (-deltah + (k_1 / k_3) * x_seal) / (1 + (k_1 / k_3));
    double F1 = k_1*(x_seal-x_face);
    double F0 = k_0*(x_lp-x_seal);
    double dhdt = -F1*(deltah+h0)/visc;
    double dx_sealdt = 0.0;

    //error check
    if(fabs((F0-F1)) < 1){
    }

    if(fabs((F0-F1)/mu/P) < 1){
        dx_sealdt = atanh((F0-F1)/(mu)/P);
    } else {
        dx_sealdt = 1e1* ((F0-F1) > 0 ? 1 : -1);
    }
    double dx_lpdt = v_LP;

    dydt[0] = dhdt;
    dydt[1] = dx_sealdt;
    dydt[2] = dx_lpdt;
}

void Machine::RKSolver(double t0, double* y0, double v_LP, double step_size, int steps) {
    double t[STEPS] {};
    double y[STEPS][Y_SIZE]  {};
    double k1[Y_SIZE] {};
    double k2[Y_SIZE] {};
    double k3[Y_SIZE] {};
    double k4[Y_SIZE] {};
    double yReturn[Y_SIZE] {};

    // Set the initial values
    t[0] = t0;
    for (int j = 0; j < Y_SIZE; j++) {
        y[0][j] = y0[j];
    }

    // Iterate over the desired time range
    for (int i = 1; i < steps; i++) {
        // Get the previous time and solution
        double ti = t[i-1];
        double yi[Y_SIZE];
        for (int j = 0; j < Y_SIZE; j++) {
            yi[j] = y[i-1][j];
        }

        // Compute the intermediate values
        axialDerivs(ti, yi, k1, v_LP);
        for (int j = 0; j < Y_SIZE; j++) {
            yi[j] = y[i-1][j] + step_size/2 * k1[j];
        }
        axialDerivs(ti + step_size/2, yi, k2, v_LP);
        for (int j = 0; j < Y_SIZE; j++) {
            yi[j] = y[i-1][j] + step_size/2 * k2[j];
        }
        axialDerivs(ti + step_size/2, yi, k3, v_LP);
        for (int j = 0; j < Y_SIZE; j++) {
            yi[j] = y[i-1][j] + step_size * k3[j];
        }
        axialDerivs(ti + step_size, yi, k4, v_LP);

        // Compute the next time and solution
        double ti_1 = ti + step_size;
        double yi_1[Y_SIZE];
        for (int j = 0; j < Y_SIZE; j++) {
            yi_1[j] = yi[j] + step_size/6 * (k1[j] + 2*k2[j] + 2*k3[j] + k4[j]);
        }

        // Store the values in the arrays
        t[i] = ti_1;
        for (int j = 0; j < Y_SIZE; j++) {
            y[i][j] = yi_1[j];
            yReturn[j] = yi_1[j];
        }

    }

    y0[0]=yReturn[0];
    y0[1]=yReturn[1];
    y0[2]=yReturn[2];
}


double Machine::axialRam(double amplifierInput, double timestep){

    if (ExternalLoad > 0.0) {
        dExternalDisp = amplifierInput*0.3*timestep*(100.0/(100.0+ExternalLoad));
    } else {
        dExternalDisp = amplifierInput*0.3*timestep;
    }

    //step axialSystem STEPS for each input
    RKSolver(0, axialSystem, dExternalDisp/timestep/STEPS, timestep/STEPS, STEPS);

    InternalDisp = (-axialSystem[0] + (k_1 / k_3) * axialSystem[1]) / (1 + (k_1 / k_3));//x_face = (-deltah + (k_1 / k_3) * x_seal) / (1 + (k_1 / k_3));
    InternalLoad= k_1*(axialSystem[1]-InternalDisp); //F1= k_1*(x_seal-x_face);
    ExternalLoad= k_0*(axialSystem[2]-axialSystem[1]); //F0= k_0*(x_lp-x_seal);
    ExternalDisp= axialSystem[2];




    // dExternalDisp = amplifierInput*0.3*timestep*(100.0/(100.0+ExternalLoad));
    // if (amplifierInput > 0.0) {
    //     RamDirection = 1.0;
    // } else if (amplifierInput < 0.0) {
    //     RamDirection = -1.0;
    // } else {
    //     RamDirection = 0.0;
    // }

    // //if we exceed seal friction, move
    // if ((ExternalLoad + dExternalDisp*frameK - (InternalLoad+calcSealFriction())) >= 0.0){
    //     dInternalDisp = dExternalDisp;
    //     ExternalDisp += dExternalDisp;
    // } else { // else load external piston
    //     ExternalLoad += dExternalDisp*frameK;
    //     dInternalDisp = 0.0;
    //     ExternalDisp += dExternalDisp;
    // }

    // //if we aren't in contact, set internal load to confining pressure
    // if (InternalDisp-HitGap <= 0.0 ){
    //     InternalDisp += dInternalDisp;
    //     InternalLoad = PConfining;
    // } else { //else load sample
    //     InternalLoad += dInternalDisp*internalK;
    //     SampleDisp += dInternalDisp; //elastic for now
    // }


    return 1;
}

double Machine::calcSealFriction(){
    return RamDirection*PConfining*sealFriction;
}

int Machine::setInputChannels(int Load, int Pc, int LVDT){
    return 0;
}

int Machine::setSampleBehavior(int viscous, double u0, double a, double b, double dc){
    return 0;
}

int Machine::updateState(double * analogInput, double * analogOutput, bool * relayOutput, double timestep){
    if (relayOutput[0] > 0.0){
        PcPumpUp(timestep);
    }
    if (relayOutput[1] < 0.0){
        PcVent(timestep);
    }

    axialRam(analogOutput[0], timestep);

    analogInput[0] = ExternalLoad;
    analogInput[1] = PConfining;
    analogInput[3] = ExternalDisp; // matches USGS Rigs
    return 1;
}

int Machine::resetState(){
    double PConfining = 0.0;
    double ExternalLoad = 0.0;
    double InternalLoad = 0.0;
    double ExternalDisp = 0.0;
    double SampleDisp = 0.0;
    double SampleShear = 0.0;
    return 1;
}    // if (ExternalLoad > 0.0) {
    //     dExternalDisp = amplifierInput*0.3*timestep*(100.0/(100.0+ExternalLoad));
    // } else {
    //     dExternalDisp = amplifierInput*0.3*timestep;
    // }

    // //step axialSystem STEPS for each input
    // RKSolver(0, axialSystem, dExternalDisp/timestep/STEPS, timestep/STEPS, STEPS);

    // InternalDisp = (-axialSystem[0] + (k_1 / k_3) * axialSystem[1]) / (1 + (k_1 / k_3));//x_face = (-deltah + (k_1 / k_3) * x_seal) / (1 + (k_1 / k_3));
    // InternalLoad= k_1*(axialSystem[1]-InternalDisp); //F1= k_1*(x_seal-x_face);
    // ExternalLoad= k_0*(axialSystem[2]-axialSystem[1]); //F0= k_0*(x_lp-x_seal);
    // ExternalDisp= axialSystem[2];

