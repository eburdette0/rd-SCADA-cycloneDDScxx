#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include "UeiPacUtils.h"

void Usage(char *prog_name)
{
    printf("%s, Copyright (C) 2022, United Electronic Industries, Inc.\n", prog_name);
    printf("usage: %s [options]\n", prog_name);
    printf("\t-h : display help\n");
    printf("\t-d n: selects the device(s) to use (default: 0)\n");
    printf("\t-n n: selects the number of scans per buffer (samples per channel)\n");
    printf("\t-f n.nn : set the rate of the DAQ operation (default: 1000 Hz)\n");
    printf("\t-c \"x,y,z,...\" : select the channels to use (default: channel 0)\n");
    printf("\t-t n: configure digital trigger (only for VMAP examples)\n \t\t0 -> Software trigger\n \t\t1 -> start on raising edge\n \t\t2 -> start on falling edge\n");
    printf("\t-x n: configure clock source (only for VMAP examples)\n \t\t0 -> Internal clock\n \t\t1 -> PLL\t\t2 -> Sync clock In\n");
    printf("\t-r : turn-on regeneration (only for buffered output examples)\n");
    printf("\t-s : stream file storing acquired/generated data\n");
    printf("\t-b n: selects the number of buffers to acquire (-1 for continuous)\n");
    printf("\t-p \"x,y,z,...\": arbitrary parameters\n");
    printf("\t-i : IP address of IOM (default is '127.0.0.1')\n");
    printf("\t-v n: sets verbosity level to print more or less information\n");
    printf("\t-N \"param1=value1,param2=value2,...\" : named parameters\n");
}

int ParseParameters(int argc, char **argv, PDNA_PARAMS *params)
{
    int ret = 0;
    int i = 1;
    int j;

    while(i<argc)
    {
        if(argv[i][0] == '-')
        {
            switch(argv[i][1])
            {
            case 'd':
                i++;
                ParseList(argv[i], (unsigned int*)params->devices, &params->numDevices);
                printf("%d device(s) specified: ", params->numDevices);
                for(j=0; j<params->numDevices; j++)
                    printf("%d ", params->devices[j]);
                printf("\n");
                // keep compatibility with old parser which only accepted one device
                params->device = params->devices[0];
                break;
            case 'n':
                i++;
                params->numSamplesPerChannel = atoi(argv[i]);
                printf("Number of scans is set to %d\n", params->numSamplesPerChannel);
                break;
            case 'f':
                i++;
                params->frequency = atof(argv[i]);
                printf("Frequency is set to %f\n", params->frequency);
                break;
            case 'c':
                i++;
                ParseList(argv[i], params->channels, &params->numChannels);
                printf("%d channel(s) specified: ", params->numChannels);
                for(j=0; j<params->numChannels; j++)
                    printf("%d ", params->channels[j]);
                printf("\n");
                break;
            case 't':
                i++;
                params->trigger = atoi(argv[i]);
                printf("Trigger is set to %d\n", params->trigger);
                break;
            case 'x':
                i++;
                params->clockSource = atoi(argv[i]);
                printf("Clock source is set to %d\n", params->clockSource);
                break;
            case 'r':
                params->regenerate = 1;
                printf("Output regeneration is set to %d\n", params->regenerate);
                break;
            case 's':
                i++;
                strcpy(params->streamFileName, argv[i]);
                printf("Stream file is set to %s\n", params->streamFileName);
                break;
            case 'b':
                i++;
                params->numBuffers = atoi(argv[i]);
                printf("Number of buffers is set to %d\n", params->numBuffers);
                break;
            case 'p':
                i++;
                ParseList(argv[i], params->arbParams, &params->numArbParams);
                printf("%d arbitrary parameters(s) specified: ", params->numArbParams);
                for(j=0; j<params->numArbParams; j++)
                    printf("%d ", params->arbParams[j]);
                printf("\n");
                break;
            case 'v':
                i++;
                params->verbose = atoi(argv[i]);
                printf("Verbosity level is set to %d\n", params->verbose);
                break;
            case 'g':
                i++;
                params->gain = atoi(argv[i]);
                printf("Gain is set to %d\n", params->gain);
                break;
            case 'i':
                i++;
                strcpy(params->ipAddress, argv[i]);
                printf("IP address is set to %s\n", params->ipAddress);
                break;
            case 'N':
                i++;
                ret = ParseNamedParamList(argv[i], params->names, params->values, &params->numNamedParams);
                if(ret < 0) exit(EXIT_FAILURE);
                printf("%d named parameters(s) specified: ", params->numNamedParams);
                for(j=0; j<params->numNamedParams; j++)
                    printf("%s=%s", params->names[j], params->values[j]);
                printf("\n");
                break;
            case 'h':
            default:
                Usage(argv[0]);
                exit(0);
                break;
            }

            i++;
        }
        else
        {
            Usage(argv[0]);
            exit(0);
        }
    }

    return 0;
}


int ParseList(char *listParam, unsigned int listArray[], int *numListElements)
{
    char *p = listParam;
    char *comma, *colon;
    int i=0;

    // if there is a colon in the channel list, this is a range
    colon = strchr(listParam, ':');
    if(colon != NULL)
    {
        int first, last;

        first = atoi(listParam);
        last = atoi(colon+1);

        *numListElements = last-first+1;
        for(i=0; i < *numListElements; i++)
        {
            listArray[i]=first+i;
        }
    }
    else
    {
        i=0;

        while(p != NULL)
        {
            listArray[i] = atoi(p);
            comma = strchr(p, ',');
            if(comma == NULL)
                break;

            p = comma+1;
            i++;
        }

        *numListElements = i+1;
    }

    return 0;
}

int ParseNamedParamList(char *listParam, char* nameArray[], char* valueArray[], int *numListElements)
{
    char *p = listParam;
    char *comma, *equal;
    int i=0;


    while(p != NULL)
    {
        // Look for '='
        equal = strchr(p, '=');
        comma = strchr(p, ',');
        if(comma == NULL)
            comma = p+strlen(listParam);
        
        // sanity check
        if(comma < equal)
        {
            fprintf(stderr, "Invalid syntax for named parameters\n");
            return -1;
        }
        
        
        nameArray[i] = (char*)malloc(equal-p+1);
        valueArray[i] = (char*)malloc(comma-equal+1);
        
        strncpy(nameArray[i], p, equal-p);
        strncpy(valueArray[i], equal+1, comma-equal);
        
        // Move on to next parameter (if any)
        comma = strchr(p, ',');
        if(comma == NULL)
            break;

        p = comma+1;
        i++;
    }

    *numListElements = i+1;

    return 0;
}

int UeiPacGetVarSize(UEIPAC_VAR_TYPE type)
{
    int varSize = -1;
    switch(type)
    {
    case UEIPAC_VAR_UINT8:
        varSize = 1;
        break;
    case UEIPAC_VAR_UINT16:
        varSize = 2;
        break;
    case UEIPAC_VAR_UINT32:
        varSize = 4;
        break;
    case UEIPAC_VAR_DOUBLE:
        varSize = 8;
        break;
    case UEIPAC_VAR_FLOAT:
        varSize = 4;
        break;
    }

    return varSize;
}

int UeiPacAllocateVars(int numVars, int varType, char names[][128])
{
    int varId;
    int shMemSize;
    int namesAreaSize = 0;
    int i;
    UEIPAC_VARS* vars;

    // Calculate size required to store variable names
    for(i=0; i<numVars; i++)
    {
        int nameLen = strlen(names[i]);
        char defaultName[16];

        sprintf(defaultName, "Var%d", i);

        if(0 == nameLen)
        {
            nameLen = strlen(defaultName);
        }

        namesAreaSize = namesAreaSize + nameLen + 1;
    }

    shMemSize = sizeof(UEIPAC_VARS) + numVars*UeiPacGetVarSize(varType) + namesAreaSize + 1;

    // Allocate shared memory segment to hold logger status and current scan
    // and make it available to other processes
    varId = shmget(SHM_VARS_KEY, shMemSize, 0777 | IPC_CREAT);
    if(varId == -1)
    {
        fprintf(stderr,  "shmget failed with error %d: %s\n", errno, strerror(errno));
        return -errno;
    }

    vars = (UEIPAC_VARS*)shmat(varId, (void *)0, 0);
    if(vars == (void *)-1)
    {
        fprintf(stderr,  "shmat failed with error %d: %s\n", errno, strerror(errno));
        return -errno;
    }

    memset(vars, 0, shMemSize);

    vars->numberOfVariables = numVars;
    vars->valuesOffset = namesAreaSize;
    vars->valueType = varType;

    // Copy variable names
    namesAreaSize = 0;
    for(i=0; i<numVars; i++)
    {
        int nameLen = strlen(names[i]);
        char defaultName[16];

        sprintf(defaultName, "Var%d", i);

        if(0 == nameLen)
        {
            nameLen = strlen(defaultName);
            strcpy((char*)vars->buffer + namesAreaSize, defaultName);
        }
        else
        {
            strcpy((char*)vars->buffer + namesAreaSize, names[i]);
        }

        namesAreaSize = namesAreaSize + nameLen + 1;
    }

    shmdt(vars);

    return varId;
}

int UeiPacGetValueArray(int varId, void** valArray)
{
    UEIPAC_VARS* vars;

    vars = (UEIPAC_VARS*)shmat(varId, (void *)0, 0);
    if(vars == (void *)-1)
    {
        fprintf(stderr,  "shmat failed with error %d: %s\n", errno, strerror(errno));
        return -errno;
    }

    *valArray = (void*)(vars->buffer + vars->valuesOffset);

    return 0;
}

int UeiPacFreeVars(int varId)
{
    shmctl(varId, IPC_RMID, 0);
    return 0;
}

void* UeiPacOpenVars(int* numVars, int* varType, char** names, void** values)
{
    int varId = -1;
    UEIPAC_VARS* vars;

    // Check whether shared memory segment is up
    varId = shmget(SHM_VARS_KEY, sizeof(int), 0777);
    if(varId == -1)
    {
        //fprintf(stderr,  "shmget failed with error %d: %s\n", errno, strerror(errno));
        return NULL;
    }

    vars = (UEIPAC_VARS*)shmat(varId, (void *)0, 0);
    if(vars == (void *)-1)
    {
        //fprintf(stderr,  "shmat failed with error %d: %s\n", errno, strerror(errno));
        return NULL;
    }

    *numVars = vars->numberOfVariables;
    *varType = vars->valueType;
    *names = (char*)vars->buffer;
    *values = (void*)(vars->buffer + vars->valuesOffset);

    return vars;
}

void UeiPacCloseVars(void* vars)
{
    shmdt(vars);
}

