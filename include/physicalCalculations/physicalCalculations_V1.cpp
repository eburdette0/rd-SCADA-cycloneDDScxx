#include "physicalCalculations_V1.h"

//#include "definitions.h"
#include "macro_definitions.h"



namespace physCalc{

    void setSampleGeom(double angle, double radius, calculatorParams calcPs){
        if (angle>0){
            calcPs.faultAngle = angle;
        }
        if (radius>0){
            calcPs.sampleRadius = radius;
        }
    }
    void setSealParams(double db, double sStiffness, calculatorParams calcPs){
        if (db>0){
            calcPs.sealDeadband = db;
        }
        if (sStiffness>0){
            calcPs.sealStiffness = sStiffness;
        }
    }

    void setFractionalStiffness(double fStiffness, calculatorParams calcPs){
        if (fStiffness>0){
            calcPs.fractionalStiffness = fStiffness;
        }
     
}

    double shearStress(double axialStress, double confiningStress, double angle){
        return std::sin(angle/180*3.14159*2.0)*(axialStress-confiningStress)/2.0;
    }

    double normalStress(double axialStress, double confiningStress, double angle){
        return (axialStress+confiningStress)/2.0 - (axialStress-confiningStress)/2.0*std::cos(angle/180*3.14159*2.0);
    }

    double diamAdjustedAxialStress(double axialStress, double confiningStress, double pistonToSampleRatio){
        return confiningStress+(axialStress-confiningStress)*pistonToSampleRatio;
    }

    double weightedSum(double Ch1, double Ch2, double Ch3, double w1, double w2, double w3){
        return w1*Ch1+w2*Ch2+w3+Ch3;
    }

    double weightedProd(double Ch1, double Ch2, double Ch3, double w1){
        //add logic further up to allow for one value to be constant
        double denom = Ch3;
        if (std::abs(denom) < 1.0e-12f) {
            denom = std::copysign(1.0e-12f, denom);
        }
        return w1*Ch1*Ch2 / denom;
    }


    // Deadband Correction for Seal Friction
    double sealFric(double extDisp, double intDisp, double sealDeadband, double sealStiffness){
            double hdbDist = sealDeadband/2.0/sealStiffness; //this stiffness is the piston between the LVDT and the seal

            if (extDisp>=intDisp+hdbDist){
                return sealDeadband/2.0;
            } else if (extDisp<=intDisp-hdbDist){
                return -sealDeadband/2.0;
            } else{
                return (extDisp-intDisp)*sealStiffness;
            }

            //old functions translated from FORTRAN
            //   double dbmax = sealStiff * (drmt - fslip);
            //   double dbmin = dbmax - db;
            //   if (internalStress > dbmax) {
            //     // Moved above deadband - reset db
            //     fslip = drmt - internalStress / sealStiff; //this is a shared variable, how to address state!!
            //     return internalStress;

            //   } else if (internalStress < dbmin) {
            //     // Moved below db - reset
            //     fslip = drmt - (internalStress + db) / sealStiff; //this is a shared variable, how to address state!!
            //     return internalStress + db;

            //   } else {
            //     // Still within db
            //     return dbmax;
            //   }
    }

    double distEst(double extDisp, double intDisp, double sealDeadband, double sealStiffness){
        double hdbDist = sealDeadband/2.0/sealStiffness;

        if (extDisp>=intDisp+hdbDist){
            return extDisp-hdbDist; //new intDisp
        } else if (extDisp<=intDisp-hdbDist){
            return extDisp+hdbDist; //new intDisp
        } else{
            return intDisp;
        }
    }

    // Overlap Area Corrected Diff Stress
    double ACD(double relDisp, double radius, double axialStress, double confiningStress, double fractionalStiffness){
        double z = relDisp;
        double r = radius;
        double diffStress = axialStress - confiningStress;
        if (z >= 2.0f * r) {
            // Sliding gone too far - something wrong
            return diffStress;
        }
        if (z <= 0.0f) {
            // Not in contact
            return diffStress;
        }
        // Calculate area correction terms
        z -= diffStress / fractionalStiffness; //Machine stiffness!! possibly dangerous without a check here
        float rsq = r * r;
        float area = 3.141593f * rsq;
        float corrf1 = 0.5773503f * z * std::sqrt(rsq - 0.08333333f * z * z);
        float corrf2 = 2.0f * rsq * std::asin(0.2886751f * z / r);
        return diffStress * area / (area - corrf1 - corrf2);
    }

    void aiToScaled(double* aidata, double* scaling, double* offsets, double* outCalc){
        for (int i=0; i<INPUT_CHANNELS; i++){
            outCalc[i] = aidata[i]*scaling[i] + offsets[i];
        }
    }

    void calcOnScaled(double* outCalc, calculatorParams calcPs){
        //careful not to exceed INPUT_CHANNELS + CALC_CHANNELS - 1 (from definitions.h)
        outCalc[INPUT_CHANNELS] = outCalc[calcPs.axialStress_Ch]-outCalc[calcPs.confiningP_Ch];
        outCalc[INPUT_CHANNELS+1] = outCalc[calcPs.axialStress_Ch]-sealFric(outCalc[calcPs.lvdt_Ch], outCalc[INPUT_CHANNELS+2], calcPs.sealDeadband, calcPs.sealStiffness);
        outCalc[INPUT_CHANNELS+2] = distEst(outCalc[calcPs.lvdt_Ch], outCalc[INPUT_CHANNELS+2], calcPs.sealDeadband, calcPs.sealStiffness);
        outCalc[INPUT_CHANNELS+3] = outCalc[INPUT_CHANNELS+1]-outCalc[calcPs.confiningP_Ch];
        outCalc[INPUT_CHANNELS+4] = diamAdjustedAxialStress(outCalc[INPUT_CHANNELS+1], outCalc[calcPs.confiningP_Ch], (calcPs.extPistonRadius*calcPs.extPistonRadius)/(calcPs.sampleRadius*calcPs.sampleRadius));
        outCalc[INPUT_CHANNELS+5] = outCalc[INPUT_CHANNELS+4]-outCalc[calcPs.confiningP_Ch];

        outCalc[INPUT_CHANNELS+6] = shearStress(outCalc[INPUT_CHANNELS+4], outCalc[calcPs.confiningP_Ch], calcPs.faultAngle); //shear stress
        outCalc[INPUT_CHANNELS+7] = normalStress(outCalc[INPUT_CHANNELS+4], outCalc[calcPs.confiningP_Ch], calcPs.faultAngle); //normal stress
        outCalc[INPUT_CHANNELS+8] = ACD(outCalc[INPUT_CHANNELS+2]-calcPs.refIntDisp, calcPs.sampleRadius, outCalc[INPUT_CHANNELS+4], outCalc[calcPs.confiningP_Ch], calcPs.fractionalStiffness); //area corrected diffStress
        outCalc[INPUT_CHANNELS+9] = shearStress(outCalc[INPUT_CHANNELS+8]+outCalc[calcPs.confiningP_Ch], outCalc[calcPs.confiningP_Ch], calcPs.faultAngle); //area corrected shear stress
        outCalc[INPUT_CHANNELS+10] = normalStress(outCalc[INPUT_CHANNELS+8]+outCalc[calcPs.confiningP_Ch], outCalc[calcPs.confiningP_Ch], calcPs.faultAngle) ; //area corrected normal stress
        outCalc[INPUT_CHANNELS+11] = outCalc[INPUT_CHANNELS+6]/outCalc[INPUT_CHANNELS+7];
        outCalc[INPUT_CHANNELS+12] = outCalc[INPUT_CHANNELS+9]/outCalc[INPUT_CHANNELS+10];

    }

    void aiToCalc(double* aidata, double* scaling, double* offsets, double* outCalc, calculatorParams calcPs){
        aiToScaled(aidata, scaling, offsets, outCalc);
        calcOnScaled(outCalc, calcPs);
    }


    void copyScaledAi(double* outCalc, double* aiScaled){
        for (int i=0; i<INPUT_CHANNELS; i++){
            aiScaled[i] =  outCalc[i];
        }
    }

    void copyDerivedCalcs(double* outCalc, double* calcSubset){
        for (int i=INPUT_CHANNELS; i<(INPUT_CHANNELS+CALC_CHANNELS); i++){
            calcSubset[i] = outCalc[i];
        }
    }

    void copyFullCalcs(double* outCalc, double* calcCopy){
        for (int i=0; i<(INPUT_CHANNELS+CALC_CHANNELS); i++){
            calcCopy[i] = outCalc[i];
        }
    }
}



//old functions from microvax fortran code

// // Function to calculate special function values
// void spftn2_(short kcall) {
//   for (int i = nadch + 1; i <= nch; ++i) {
//     int isfndx = i - nadch - 1;
//     switch (sfcom.isfflg[isfndx]) {
//       case 1: // Area Corrected Diff Stress
//         {
//           float z = prime.adval[sfcom.isfch[isfndx][2] - 1];
//           float r = sfcom.sfcon[isfndx][0];logBundle
//           float diffStress = prime.adval[sfcom.isfch[isfndx][0] - 1] -
//                          prime.adval[sfcom.isfch[isfndx][1] - 1];
//           if (z >= 2.0f * r) {
//             // Sliding gone too far - something wrong
//             prime.adval[i - 1] = prime.spmax[i - 1] + 500.0f;
//             break;
//           }
//           if (z <= 0.0f) {
//             // Not in contact
//             prime.adval[i - 1] = diffStress;
//             break;
//           }
//           // Calculate area correction terms
//           z -= diffStress / sfcom.sfcon[isfndx][1];
//           float rsq = r * r;
//           float area = 3.141593f * rsq;
//           float corrf1 = 0.5773503f * z * std::sqrt(rsq - 0.08333333f * z * z);
//           float corrf2 = 2.0f * rsq * std::asin(0.2886751f * z / r);
//           prime.adval[i - 1] = diffStress * area / (area - corrf1 - corrf2);
//         }
//         break;
//       case 2: // Normal Stress
//         prime.adval[i - 1] = 0.25f * prime.adval[sfcom.isfch[isfndx][0] - 1] +
//                            prime.adval[sfcom.isfch[isfndx][1] - 1];
//         break;
//       case 3: // Shear Stress
//         prime.adval[i - 1] = 0.433013f * prime.adval[sfcom.isfch[isfndx][0] - 1];
//         break;
//       case 4: // Adjusted Axial Load
//         prime.adval[i - 1] = prime.adval[sfcom.isfch[isfndx][1] - 1] +
//                            (prime.adval[sfcom.isfch[isfndx][0] - 1] -
//                             prime.adval[sfcom.isfch[isfndx][1] - 1]) *
//                                sfcom.sfcon[isfndx][0];
//         break;
//       case 5: // Weighted Sum of 3 Chans
//         prime.adval[i - 1] = sfcom.sfcon[isfndx][0] *
//                                prime.adval[sfcom.isfch[isfndx][0] - 1] +
//                            sfcom.sfcon[isfndx][1] *
//                                prime.adval[sfcom.isfch[isfndx][1] - 1] +
//                            sfcom.sfcon[isfndx][2] *
//                                prime.adval[sfcom.isfch[isfndx][2] - 1];
//         break;
//       case 6: // Weighted Product and Ratio of 3 Chans
//         {
//           float f1 = sfcom.isfch[isfndx][0] == 0
//                          ? 1.0f
//                          : prime.adval[sfcom.isfch[isfndx][0] - 1];
//           float f2 = sfcom.isfch[isfndx][1] == 0
//                          ? 1.0f
//                          : prime.adval[sfcom.isfch[isfndx][1] - 1];
//           float f3 = sfcom.isfch[isfndx][2] == 0
//                          ? 1.0f
//                          : prime.adval[sfcom.isfch[isfndx][2] - 1];
//           float denom = f3;
//           if (std::abs(denom) < 1.0e-20f) {
//             denom = std::copysign(1.0e-20f, denom);
//           }
//           prime.adval[i - 1] = sfcom.sfcon[isfndx][0] * f1 * f2 / denom;
//         }
//         break;
//       case 7: // Deadband Correction for Seal Friction
//         {
//           float internalStress = prime.adval[sfcom.isfch[isfndx][0] - 1];
//           float drmt = prime.adval[sfcom.isfch[isfndx][1] - 1];
//           float db = sfcom.sfcon[isfndx][0];
//           float stif = sfcom.sfcon[isfndx][1];
//           float fslip = sfcom.sfcon[isfndx][2];
//           float dbmax = stif * (drmt - fslip);
//           float dbmin = dbmax - db;
//           if (internalStress > dbmax) {
//             // Moved above deadband - reset db
//             prime.adval[i - 1] = internalStress;
//             sfcom.sfcon[isfndx][2] = drmt - internalStress / stif;
//           } else if (internalStress < dbmin) {
//             // Moved below db - reset
//             prime.adval[i - 1] = internalStress + db;
//             sfcom.sfcon[isfndx][2] = drmt - (internalStress + db) / stif;
//           } else {
//             // Still within db
//             prime.adval[i - 1] = dbmax;
//           }
//         }
//         break;
//     }
//   }
// }