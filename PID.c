#include <stdint.h>

#include <PID.h>


PID_Handle_t PID_create(int32_t pTerm, int32_t iTerm, int32_t dTerm){
	PID_Handle_t pid = pvPortMalloc(sizeof(PID_Handle_t));
	
	PID_reset(pid);
	
	return pid;
}

void PID_dispose(PID_Handle_t pid){
	vPortFree(pid);
}



void PID_reset(PID_Handle_t pid){
	pid->lastError = INT32_MAX;
	pid->accumulator = 0;
	
}

void PID_setOutputLimits(PID_Handle_t pid, int32_t outputMax, int32_t outputMin){
	if(outputMax < outputMin) return;
	
	pid->outMax = outputMax;
	pid->outMin = outputMin;
	pid->outSpan = outputMax - outputMin;
}

void PID_setCoefficients(PID_Handle_t pid, int32_t pTerm, int32_t iTerm, int32_t dTerm){
	pid->pTerm = pTerm;
	pid->iTerm = iTerm;
	pid->dTerm = dTerm;
}

int32_t PID_getCurrentOutput(PID_Handle_t pid){
	return pid->currentOutput;
}

int32_t PID_run(PID_Handle_t pid, int32_t targetValue, int32_t actualValue){
	//first calculate the current error
	int32_t error = actualValue - targetValue;
	
	//limit error to 16bit to prevent overflow during further calculations
	if(error > INT16_MAX) error = INT16_MAX;
	if(error < INT16_MIN) error = INT16_MIN;

	
	
	//do I calculation

	//check if we can integrate without overflowing the accumulator
	if(INT32_MAX - abs(pid->accumulator) > abs(error)){ 
		//yes => do so
		pid->accumulator += (pid->iTerm * error) >> 16;
	}else{
		//no => at least get the acumulator to its maximum value
		if(error > 0){
			pid->accumulator = INT32_MAX;
		}else{
			pid->accumulator = INT32_MIN;
		}
	}

	
	
	//do D calculation
	
	//calculate current derivative
	int32_t errorDifference = error - pid->lastError;
	
	//is the last error invalid?
	if(pid->lastError == INT32_MAX) errorDifference = 0;
	
	pid->lastError = error;
	


	//generate output value from the individual terms
	
	//add terms together
	int32_t output = 0;
	output += (pid->pTerm * error) >> 16;
	output += pid->accumulator >> 16;
	output += (pid->dTerm * errorDifference) >> 16;
	
	//limit output to 16bit to prevent overflow during further calculations
	if(output > INT16_MAX) output = INT16_MAX;
	if(output < INT16_MIN) output = INT16_MIN;
	
	//scale into the selected output range
	int32_t outputScaled = ((pid->outSpan * (output + INT16_MIN)) >> 16) pid->outMin;
	
	pid->currentOutput = outputScaled;
	return outputScaled;
}