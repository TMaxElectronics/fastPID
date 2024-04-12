#ifndef PID_INC
#define PID_INC

#include <stdint.h>

#define PID_LP_SUM 32
#define PID_LP_SHIFT 5

typedef enum {PID_MS_RISETIME, PID_MS_STEADYSTATE, PID_MS_NOISE} TransState_t;

typedef struct{
    int32_t starttime_counts;
    int32_t steadyCounter;
    int32_t peakErrorNoise;
    int32_t peakValueOvershoot;
    int32_t targetValue;
    
    TransState_t state;
    
    int32_t risetime_us;
    int32_t timeForSteadyState_us;
    
    int32_t overshoot_percent;
    int32_t noise_percent;
} PID_Measurement_t;


typedef struct{
	int32_t currentOutput;
	int32_t lastError;
	int32_t lastLPError;
	int32_t accumulator;
	
	int32_t lpFilterRatioA;
	int32_t lpFilterRatioB;
    
	int32_t outMin;
	int32_t outMax;
	uint32_t outSpan;
    int32_t deadband;
	
	int32_t pTerm;
	int32_t iTerm;
	int32_t dTerm;
    
    PID_Measurement_t * currMeasHandle;
} PID_Handle_t;

typedef struct{
    int32_t pTerm;
    int32_t iTerm;
    int32_t dTerm;
    int32_t deadband;
    int32_t lowPass;
} PID_Parameter_t;
PID_Handle_t * PID_create(PID_Parameter_t * config);
void PID_reset(PID_Handle_t * pid);
void PID_startTransientMeasurements(PID_Handle_t * pid);
PID_Measurement_t * PID_endTransientMeasurements(PID_Handle_t * pid); 
void PID_dispose(PID_Handle_t * pid);
void PID_setOutputLimits(PID_Handle_t * pid, int32_t outputMax, int32_t outputMin);
void PID_setCoefficients(PID_Handle_t * pid, int32_t pTerm, int32_t iTerm, int32_t dTerm);
int32_t PID_run(PID_Handle_t * pid, int32_t targetValue, int32_t actualValue);
int32_t PID_getCurrentOutput(PID_Handle_t * pid);
void PID_setPTermFilter(PID_Handle_t * pid, uint32_t filterRatio);
void PID_setDeadband(PID_Handle_t * pid, int32_t deadBand);

#endif