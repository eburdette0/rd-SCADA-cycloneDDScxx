#pragma once


class PIDcoeff
{
public:
	double kP = 0;
	double kI = 0;
	double kD = 0;
	double bias = 0;

	double outputMax = 1;
	double outputMin = -1;
	double windupMax = 0.75;
	double windupMin = -0.75;
	double deadbandMax = 0;
	double deadbandMin = 0;
};

class PID
{

public:
	PID();

	double *input;
	double *output;
	double *setpoint;
	double *manualOutput;

	void begin(double *_input,
			   double *_output,
			   double *_setpoint,
			   double *_manualOutput,
			   const double &_p = 0,
			   const double &_i = 0,
			   const double &_d = 0,
			   const double &_bias = 0,
			   const double &_minSamplePeriod_s = 0.1);

	void resetState();

	void controlModeAuto();
	void controlModeMan();
	unsigned int controlMode();
	void clampOutput();
	void unclampOutput();
	unsigned int clamped();

	void compute();

	void setOutputLimits(const double &min, const double &max);
	void setWindUpLimits(const double &min, const double &max);
	void setDeadBand(const double &min, const double &max);
	void setBias(const double &_bias);
	void setCoefficients(const double &_p, const double &_i, const double &_d, const double &_b);
	void setSampleTime(const double &_minSamplePeriod_s);

	double B();
	double P();
	double I();
	double D();

	PIDcoeff getCoeffs();

	// void debug(Stream* stream = &Serial,
	//            const char* controllerName = "controller",
	// 	       const byte& mask = 0xFF);



	double curSetpoint = 0;
	double curInput = 0;
	double newOutput;
	double newManualOutput;


private:
	double bias;

	double outputMax = 1.0;
	double outputMin = -1.0;

	double windupMax = 1.0;
	double windupMin = -1.0;

	double deadBandMax = 0.0;
	double deadBandMin = 0.0;

	double curError;
	// double curSetpoint = 0;
	// double curInput = 0;
	double dInput;

	double lastError;
	double lastSetpoint;
	double lastInput;

	double pIn;
	double iIn;
	double dIn;

	double kp = 0.0;
	double ki = 0.0;
	double kd = 0.0;

	double pOut;
	double iOut;
	double dOut;

	double outTemp;
	double outHeadroom_p;
	double outHeadroom_n;

	double iTemp;

	// double newOutput;
	// double newManualOutput;

	// must startup caller with safe Manual OP
	unsigned int mode = 0;
	unsigned int zeroOP = 1;

	double timestep;
};