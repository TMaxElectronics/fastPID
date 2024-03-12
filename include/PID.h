
typedef struct{
	int32_t currentOutput;
	int32_t lastError;
	int32_t accumulator;
	
	int32_t outMin;
	int32_t outMax;
	int32_t outSpan;
	
	int32_t pTerm;
	int32_t iTerm;
	int32_t dTerm;
} PID_Handle_t;

PID_Handle_t PID_create(int32_t pTerm, int32_t iTerm, int32_t dTerm);
void PID_reset(PID_Handle_t pid);
void PID_dispose(PID_Handle_t pid);
void PID_setOutputLimits(PID_Handle_t pid, int32_t outputMax, int32_t outputMin);
void PID_setCoefficients(PID_Handle_t pid, int32_t pTerm, int32_t iTerm, int32_t dTerm);
int32_t PID_run(PID_Handle_t pid, int32_t targetValue, int32_t actualValue);
int32_t PID_getCurrentOutput(PID_Handle_t pid);