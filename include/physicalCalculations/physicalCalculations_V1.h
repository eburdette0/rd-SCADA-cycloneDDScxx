#ifndef PHYSICALCALCULATIONS_V1_H
#define PHYSICALCALCULATIONS_V1_H

//#include "definitions.h" //has channel count definitions
#include "macro_definitions.h"
#include <math.h>
#include <cstdint>




namespace physCalc{

    //need to be able to copy this state structure around so we can re-calculate elsewhere if needed
    //these are usually populated via json during GetData startup
    struct calculatorParams {
        double intDispEst = 0;
        double sealStiffness = 100;
        double sealDeadband = 0;
        double fractionalStiffness = 1;
        double extPistonRadius = 12.54;
        double refIntDisp = 0;
        double faultAngle = 30;
        double sampleRadius = 12.54;
        uint32_t lvdt_Ch = 4-1;
        uint32_t axialStress_Ch = 1-1;
        uint32_t confiningP_Ch = 2-1;
    };

    double shearStress(double axialStress, double confiningStress, double angle);

    double normalStress(double axialStress, double confiningStress, double angle);

    double diamAdjustedAxialStress(double axialStress, double confiningStress, double pistonToSampleRatio);

    double weightedSum(double Ch1, double Ch2, double Ch3, double w1, double w2, double w3);

    double weightedProd(double Ch1, double Ch2, double Ch3, double w1);

    // Deadband Correction for Seal Friction
    double sealFric(double extDisp, double intDisp, double sealDeadband, double sealStiffness);

    double distEst(double extDisp, double intDisp, double sealDeadband, double sealStiffness);

    // Overlap Area Corrected Diff Stress
    double ACD(double relDisp, double radius, double axialStress, double confiningStress, double fractionalStiffness);

    void aiToScaled(double* aidata, double* scaling, double* offsets, double* outCalc);
    void calcOnScaled(double* outCalc, calculatorParams calcPs);
    void aiToCalc(double* aidata, double* scaling, double* offsets, double* outCalc, calculatorParams calcPs);

    void copyScaledAi(double* outCalc, double* aiScaled);
    void copyDerivedCalcs(double* outCalc, double* calcSubset);
    void copyFullCalcs(double* outCalc, double* calcCopy);

    void setSampleGeom(double angle, double radius, calculatorParams calcPs);
    void setSealParams(double db, double sStiffness, calculatorParams calcPs);
    void setFractionalStiffness(double fStiffness, calculatorParams calcPs);
}


#endif // PHYSICALCALCULATIONS_V1_H