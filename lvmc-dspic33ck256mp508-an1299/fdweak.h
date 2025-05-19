#ifndef __FDWEAK_H
#define __FDWEAK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
    
/* Field weakening Parameter data type

  Description:
    This structure will host parameters related to field weakening function.
 */
typedef struct
{
    /* d-current reference */
    int16_t qIdRef;
    /* Flux weakening on speed -*/
    int16_t qFwOnSpeed;
    /* Lookup tables index */
    int16_t qIndex;
    /* Curve for magnetizing current variation with speed */
    int16_t qFwCurve[18];
    /* Curve for InvKfi constant InvKfi = Omega/BEMF variation with speed */
    int16_t qInvKFiCurve[18];
    /* Curve for Ls variation with speed */
    int16_t qLsCurve[18];    
} FDWEAK_PARM_T;

extern FDWEAK_PARM_T fdWeakParm;

void InitFWParams();
int16_t FieldWeakening( int16_t qMotorSpeed );

#ifdef __cplusplus
}
#endif

#endif /* __FDWEAK_H */
