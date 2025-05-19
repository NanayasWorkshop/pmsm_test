#ifndef __DIAGNOSTICS_H
#define __DIAGNOSTICS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initializes diagnostics
 */
void DiagnosticsInit(void);

/**
 * Executes diagnostic tasks during the controller ISR
 */
void DiagnosticsStepIsr(void);

/**
 * Executes diagnostic tasks during the main loop
 */
void DiagnosticsStepMain(void);

#ifdef __cplusplus
}
#endif

#endif /* __DIAGNOSTICS_H */
