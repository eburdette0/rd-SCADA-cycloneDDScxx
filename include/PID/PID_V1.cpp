#include "PID_V1.h"

// loosely based on an arduino PID library

PID::PID()
{
}

// must run controlModeAuto() and unclampOutput() to control
void PID::begin(double *_input,
				double *_output,
				double *_setpoint,
				double *_manualOuput,
				const double &_p,
				const double &_i,
				const double &_d,
				const double &_bias,
				const double &_minSamplePeriod_s)
{

	input = _input;
	output = _output;
	setpoint = _setpoint;
	manualOutput = _manualOuput;

	setCoefficients(_p, _i, _d, _bias);
	setBias(_bias);
	setSampleTime(_minSamplePeriod_s);
	// default OP limit +-1, Antiwindup +-1, and deadband=0 in header

	resetState();
	// in manual output state and clamped (safe) by default
}

void PID::controlModeAuto()
{
	if (mode != 1)
	{
		mode = 1;
		resetState();
	}
}

void PID::controlModeMan()
{
	if (mode != 0)
	{
		mode = 0;
	}
}

unsigned int PID::controlMode()
{
	return mode;
}

void PID::clampOutput()
{
	if (zeroOP != 1)
	{
		zeroOP = 1;
	}
}

void PID::unclampOutput()
{
	if (zeroOP != 0)
	{
		zeroOP = 0;
	}
}

unsigned int PID::clamped()
{
	return zeroOP;
}

void PID::resetState()
{
	// initiate controller with current input state before computing
	//(otherwise first PID compute is bad with zero starts)
	curInput = *input;
	curSetpoint = *setpoint;
	curError = curSetpoint - curInput;

	lastInput = curInput;
	lastSetpoint = curSetpoint;
	lastError = curError;

	pOut = 0;
	iOut = 0;
	dOut = 0;
}

void PID::compute()
{
	kp = pIn;
	ki = iIn * timestep;
	kd = dIn / timestep;

	// shift previous inputs to 'last' storage
	lastInput = curInput;
	lastSetpoint = curSetpoint;
	lastError = curError;

	// populate current inputs
	curInput = *input;
	curSetpoint = *setpoint;

	// calculate error and derivative
	curError = curSetpoint - curInput;
	dInput = *input - lastInput;

	// deadband error adjustment
	if (curError <= deadBandMax && curError >= deadBandMin)
	{
		curError = 0; // in deadband
	}
	else
	{
		if (curError > 0)
		{
			curError = curError - deadBandMax; // offset out the top of the deadband
		}
		else
		{
			curError = curError - deadBandMin; // offset out the bottom of the deadband
		}
	}

	pOut = kp * curError; // calculate P

	dOut = -kd * dInput; // Derrivative on measurement

	outTemp = bias + pOut + dOut; // Output without integral


	//how much headrooom left is there for integral term?
	outHeadroom_p = outputMax - outTemp;
	outHeadroom_n = outputMin - outTemp;
	if (outHeadroom_p < 0)
		outHeadroom_p = 0; // P and D terms bigger than limits
	if (outHeadroom_n > 0)
		outHeadroom_n = 0;

	//Calculate and set integral term within headroom and anti-windup limits
	iTemp = (iIn == 0) ? 0 : iOut + (ki * ((curError + lastError) / 2.0)); // Trapezoidal integration
	if (iTemp > windupMax)
		iTemp = windupMax; // Prevent integral windup
	if (iTemp < windupMin)
		iTemp = windupMin;

	if (iTemp > outHeadroom_p)
		iTemp = outHeadroom_p; // no headroom for I, don't start integrating until we get close to SP
	if (iTemp < outHeadroom_n)
		iTemp = outHeadroom_n;
	iOut = iTemp;

	//calculate and constrain total output sum
	newOutput = bias + pOut + iOut + dOut;
	if (outHeadroom_p == 0)
		newOutput = outputMax; // constrain outputs
	if (outHeadroom_n == 0)
		newOutput = outputMin;

	
	//clamp output if in clamped state
	if (zeroOP)
	{
		*output = 0.0;
		newOutput = 0.0;
	}
	else
	{
		// set ouput to the manual OP variable if manual is enabled
		if (mode == 0)
		{
			*output = *manualOutput;
			if (*manualOutput > outputMax)
				*output = outputMax;
			if (*manualOutput < outputMin)
				*output = outputMin;
			newOutput = *output;
		}
		else
		{
			*output = newOutput; //auto
		}
	}
	newManualOutput = *manualOutput; //copy value to internal
}

void PID::setOutputLimits(const double &min, const double &max)
{
	if (max > min)
	{
		outputMax = max;
		outputMin = min;
	}
}

void PID::setWindUpLimits(const double &min, const double &max)
{
	if (max > min)
	{
		windupMax = max;
		windupMin = min;
	}
}

// Band around zero error where no reaction happens. Internal error starts at 0 at the edge fo the deadband
void PID::setDeadBand(const double &min, const double &max)
{
	if (max >= min)
	{
		deadBandMax = max;
		deadBandMin = min;
	}
}

void PID::setBias(const double &_bias)
{
	bias = _bias;
}

void PID::setCoefficients(const double &_p, const double &_i, const double &_d, const double &_b)
{
	pIn = _p;
	iIn = _i;
	dIn = _d;
	bias = _b;
	resetState();
}

void PID::setSampleTime(const double &_minSamplePeriod_s)
{
	timestep = _minSamplePeriod_s;
}

double PID::P()
{
	return pOut;
}

double PID::I()
{
	return iOut;
}

double PID::D()
{
	return dOut;
}

double PID::B()
{
	return bias;
}

PIDcoeff PID::getCoeffs(){
	PIDcoeff temp;
	temp.kP = pIn;
	temp.kI = iIn;
	temp.kD = dIn;
	temp.bias = bias;
	
	temp.outputMax = outputMax;
	temp.outputMin = outputMin;
	temp.windupMax = windupMax;
	temp.windupMin = windupMin;
	temp.deadbandMax = deadBandMax;
	temp.deadbandMin = deadBandMin;

	return temp; //copy out
}
