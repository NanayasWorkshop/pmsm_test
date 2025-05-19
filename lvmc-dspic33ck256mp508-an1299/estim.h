#ifndef __ESTIM_H
#define __ESTIM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "motor_control_noinline.h"
/* Estimator Parameter data type

  Description:
    This structure will host parameters related to angle/speed estimator
    parameters.
 */
typedef struct
{
    /* Integration constant */
    int16_t qDeltaT;
    /* angle of estimation */
    int16_t qRho;
    /* internal variable for angle */
    int32_t qRhoStateVar;
    /* primary speed estimation */
    int16_t qOmegaMr;
    /* last value for Ialpha */
    int16_t qLastIalpha;
    /* last value for Ibeta */
    int16_t qLastIbeta;
    /* difference Ialpha */
    int16_t qDIalpha;
    /* difference Ibeta */
    int16_t qDIbeta;
    /* BEMF alpha */
    int16_t qEsa;
    /* BEMF beta */
    int16_t qEsb;
    /* BEMF d */
    int16_t qEsd;
    /* BEMF q */
    int16_t qEsq;
    /* counter in Last DI tables */
    int16_t qDiCounter;
    /* dI*Ls/dt alpha */
    int16_t qVIndalpha;
    /* dI*Ls/dt beta */
    int16_t qVIndbeta;
    /* BEMF d filtered */
    int16_t qEsdf;
    /* state variable for BEMF d Filtered */
    int32_t qEsdStateVar;
    /* BEMF q filtered */
    int16_t qEsqf;
    /* state variable for BEMF q Filtered */
    int32_t qEsqStateVar;
    /* filter constant for d-q BEMF */
    int16_t qKfilterEsdq;
    /* Estimated speed */
    int16_t qVelEstim;
    /* Filter constant for Estimated speed */
    int16_t qVelEstimFilterK;
    /* State Variable for Estimated speed */
    int32_t qVelEstimStateVar;
    /* Value from last control step Ialpha */
    int16_t qLastValpha;
    /* Value from last control step Ibeta */
    int16_t qLastVbeta;
    /* dIalphabeta/dt */
    int16_t qDIlimitLS;
    /* dIalphabeta/dt */
    int16_t qDIlimitHS;
    /*  last  value for Ialpha */
    int16_t qLastIalphaHS[8];
    /* last  value for Ibeta */
    int16_t qLastIbetaHS[8];
    /* estimator angle initial offset */
    int16_t qRhoOffset;

} ESTIM_PARM_T;
/* Motor Estimator Parameter data type

  Description:
    This structure will host motor parameters parameters required by angle
    estimator.
 */
typedef struct
{
    /* Rs value - stator resistance */
    int16_t qRs;
    /* Ls/dt value - stator inductance / dt - variable with speed */
    int16_t qLsDt;
    /* Ls/dt value - stator inductance / dt for base speed (nominal) */
    int16_t qLsDtBase;
    /* InvKfi constant value ( InvKfi = Omega/BEMF ) */
    int16_t qInvKFi;
    /* InvKfi constant - base speed (nominal) value */
    int16_t qInvKFiBase;            
} MOTOR_ESTIM_PARM_T;

extern ESTIM_PARM_T estimator;
extern MOTOR_ESTIM_PARM_T motorParm;

void Estim(void);
void InitEstimParm(void);


#ifdef __cplusplus
}
#endif

#endif /* __ESTIM_H */
