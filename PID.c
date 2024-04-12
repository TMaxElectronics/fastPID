#include <stdint.h>
#include <math.h>

#include "FreeRTOS.h"

#include "PID.h"
#include "System.h"

static uint32_t PID_scaleOutput(PID_Handle_t * pid, int32_t output);

PID_Handle_t * PID_create(PID_Parameter_t * config){
	PID_Handle_t * pid = pvPortMalloc(sizeof(PID_Handle_t));
    
    pid->currMeasHandle = NULL;
	
    PID_setCoefficients(pid, config->pTerm, config->iTerm, config->dTerm);
    PID_setDeadband(pid, config->deadband);
    PID_setPTermFilter(pid, config->lowPass);
	PID_reset(pid);
	
	return pid;
}

void PID_dispose(PID_Handle_t * pid){
	vPortFree(pid);
}



void PID_startTransientMeasurements(PID_Handle_t * pid){
    if(pid->currMeasHandle == NULL) pid->currMeasHandle = pvPortMalloc(sizeof(PID_Measurement_t));
    
    pid->currMeasHandle->starttime_counts = xTaskGetTickCount();
    pid->currMeasHandle->state = PID_MS_RISETIME;
    
    pid->currMeasHandle->peakErrorNoise = 0;
    pid->currMeasHandle->peakValueOvershoot = 0;
}

PID_Measurement_t * PID_endTransientMeasurements(PID_Handle_t * pid){
    if(pid->currMeasHandle == NULL) return NULL;
    
    //clear measurement pointer to make sure nothing acesses it
    PID_Measurement_t * ret = pid->currMeasHandle;
    pid->currMeasHandle = NULL;
    
    //compute percentile overshoot from absolute values
    ret->overshoot_percent = (ret->peakValueOvershoot * 100) / ret->targetValue;
    ret->noise_percent = (ret->peakErrorNoise * 100) / ret->targetValue;
    
    //return it
    return ret;
}

void PID_reset(PID_Handle_t * pid){
	pid->lastError = INT32_MAX;
	pid->accumulator = 0;
	
}

void PID_setOutputLimits(PID_Handle_t * pid, int32_t outputMax, int32_t outputMin){
	if(outputMax < outputMin) return;
	
	pid->outMax = outputMax;
	pid->outMin = outputMin;
	pid->outSpan = outputMax - outputMin;
}

void PID_setPTermFilter(PID_Handle_t * pid, uint32_t filterRatio){
	pid->lpFilterRatioA = PID_LP_SUM-filterRatio;
	pid->lpFilterRatioB = filterRatio;
    
    PID_reset(pid);
}

//PID Coefficients are given in signed 16bit fixed point, with the first three bits being the digit before the decimal point. (so 32767 = 8.0)
void PID_setCoefficients(PID_Handle_t * pid, int32_t pTerm, int32_t iTerm, int32_t dTerm){
	pid->pTerm = pTerm;
	pid->iTerm = iTerm;
	pid->dTerm = dTerm;
    
    PID_reset(pid);
}

void PID_setDeadband(PID_Handle_t * pid, int32_t deadBand){
	pid->deadband = deadBand;
}

int32_t PID_getCurrentOutput(PID_Handle_t * pid){
	return pid->currentOutput;
}

int32_t PID_run(PID_Handle_t * pid, int32_t targetValue, int32_t actualValue){
	//first calculate the current error
	int32_t error = actualValue - targetValue;
	
	//limit error to 16bit to prevent overflow during further calculations
	if(error > INT16_MAX) error = INT16_MAX;
	if(error < INT16_MIN) error = INT16_MIN;
    
    //check if the error is outside the deadband
    if(abs(error) < pid->deadband){ 
        //no => just return the last calculated value and invalidate the lastError Field
        pid->lastError = 0;
        return pid->currentOutput;
    }
                        
	//do I calculation

	//check if we can integrate without overflowing the accumulator
	if(INT32_MAX - abs(pid->accumulator) > abs(error)){ 
		//yes => do so
		pid->accumulator += (pid->iTerm * error) >> 10;
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
	
    //is the output lowpass enabled?
	if(pid->lpFilterRatioB > 0){
        pid->lastLPError = ((error * pid->lpFilterRatioA) + (pid->lastLPError * pid->lpFilterRatioB)) >> PID_LP_SHIFT;
    }else{
        pid->lastLPError = error;
    }


	//generate output value from the individual terms
	
	//add terms together
	int32_t output = 0;
	output += (pid->pTerm * pid->lastLPError) >> 9;
	output += pid->accumulator;
	output += (pid->dTerm * errorDifference) >> 9;
    
    //TERM_printDebug(TERM_handle, "\rRunning PID: target=%d current=%d error=%d acc=%d out=%d!", targetValue, actualValue, error, pid->accumulator, output);
	
	//limit output to 16bit to prevent overflow during further calculations
	if(output > INT16_MAX) output = INT16_MAX;
	if(output < INT16_MIN) output = INT16_MIN;
	
	//scale into the selected output range
	int32_t outputScaled = PID_scaleOutput(pid, output);
    
    pid->currentOutput = outputScaled;
    
    //check if any measurements are active atm. Also do this thread safe
    vTaskEnterCritical();
    PID_Measurement_t * meas = pid->currMeasHandle;
    vTaskExitCritical();
    
    if(meas != NULL){
        //yes => first check which state we are in
        if(meas->state == PID_MS_RISETIME){
            //we are measuring the rise time => check if we have exceeded the 90% threshold of the target value
            //* 29491) >> 15 is equal to * 0.899993 but without floating point or division :P
            if(actualValue > ((targetValue * 29491) >> 15)){
                //yep 90% threshold was just exceeded => save time and go to next state
                meas->state = PID_MS_STEADYSTATE;
                meas->steadyCounter = 0;
                
                //calculate time                                                                                        uS per tick
                meas->risetime_us = (xTaskGetTickCount() - meas->starttime_counts) * (1000000 / configTICK_RATE_HZ);
            }
            
        }else if(meas->state == PID_MS_STEADYSTATE){
            //we are measuring time to get to steady state. Here we just make sure that we are within the 90%-110% threshold for ten samples
            //                  97% threshold                                   103% threshold
            if((actualValue > ((targetValue * 31785) >> 15)) && (actualValue < ((targetValue * 16876) >> 14))){
                //now check if we got our 10 samples
                if(++meas->steadyCounter > 25){
                    //yes => steady state reached. Switch state and save time
                    meas->state = PID_MS_NOISE;
                    
                    //calculate time                                                                                                    uS per tick
                    meas->timeForSteadyState_us = (xTaskGetTickCount() - meas->starttime_counts) * (1000000 / configTICK_RATE_HZ);
                }
            }else{
                meas->steadyCounter = 0;
            }
        }
        
        //what peak are we looking for right now
        if(meas->state == PID_MS_NOISE){
            //we are measuring noise amplitude. This is the peak error once we have reached steady state
            if(meas->peakErrorNoise < abs(error)) meas->peakErrorNoise = abs(error);
            
            //this state is the terminating one => no transition check required
        }else{
            //we're measuring overshoot
            if(meas->peakValueOvershoot < error) meas->peakValueOvershoot = error;
        }
        
        //and also remember what the target value was for percentile calculations
        //TODO perhaps add a protection against this changing during measurement? That would ruin it lol
        meas->targetValue = targetValue;
    }
	
    return pid->currentOutput;
}

static uint32_t PID_scaleOutput(PID_Handle_t * pid, int32_t output){
    
	//limit output to 16bit to prevent overflow during further calculations
	if(output > INT16_MAX) output = INT16_MAX;
	if(output < INT16_MIN) output = INT16_MIN;
    
    uint32_t outputWithNoOffset = (uint32_t) (output - INT16_MIN);
    int32_t scaledToSpan = ((pid->outSpan * outputWithNoOffset) >> 16);
	
	//scale into the selected output range
	int32_t outputScaled = scaledToSpan + pid->outMin + 1;
    
    //TERM_printDebug(TERM_handle, "Scaling value: val=%d, noOffset=%d, scaled=%d, offset=%d (span=%d min=%d)\r\n", output, outputWithNoOffset, scaledToSpan, outputScaled, pid->outSpan, pid->outMin);
    
	return outputScaled;
}