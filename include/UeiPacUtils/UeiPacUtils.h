#ifndef __PARSE_PARAMS_H__
#define __PARSE_PARAMS_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _pd_params
{
    int device;
    int numChannels;
    unsigned int channels[128];
    float frequency;
    int trigger;
    int numSamplesPerChannel;
    int regenerate;
    char streamFileName[260];
    int numBuffers;
    int clockSource;
    int numDevices;
    int devices[32];
    int numArbParams;
    unsigned int arbParams[128];
    int verbose;
    int gain;
    char ipAddress[32];
    int numNamedParams;
    char* names[128];
    char* values[128];
} PDNA_PARAMS;

typedef enum _ueipac_var_type
{
    UEIPAC_VAR_UINT8,
    UEIPAC_VAR_UINT16,
    UEIPAC_VAR_UINT32,
    UEIPAC_VAR_DOUBLE,
    UEIPAC_VAR_FLOAT
} UEIPAC_VAR_TYPE;

typedef struct _ueipac_variables
{
    int numberOfVariables;    // number of variables stored in shared memory
    int valueType;
    int valuesOffset;
    unsigned char buffer[];
} UEIPAC_VARS;

#define SHM_VARS_KEY 0x04030201

int UeiPacAllocateVars(int numVars, int varType, char names[][128]);
int UeiPacGetValueArray(int varId, void** valArray);
int UeiPacFreeVars(int varId);

void* UeiPacOpenVars(int* numVars, int* varType, char** names, void** values);
void UeiPacCloseVars(void* vars);

void Usage();
int ParseParameters(int argc, char **argv, PDNA_PARAMS *params);
int ParseList(char *listParam, unsigned int listArray[], int *numListElements);
int ParseNamedParamList(char *listParam, char* nameArray[], char* valueArray[], int *numListElements);

#ifdef __cplusplus
}
#endif

#endif /* __PARSE_PARAMS_H__ */


