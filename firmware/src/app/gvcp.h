/*
 * gvcp.h - GigE Vision control server (GVCP, UDP 3956), stage 2 of
 * docs/protocol/GEV_PROTOCOL_DESIGN.md (23.10.2026): bootstrap and manufacturer
 * registers, CCP and heartbeat, the GenApi XML over READMEM. It runs beside VMSP;
 * manufacturer registers call the VMSP control handlers in-process (vmsp_call), so no
 * camera, sync block or settings logic is duplicated. No streaming yet (GVSP is stage 3).
 */
#ifndef APP_GVCP_H_
#define APP_GVCP_H_

#define GVCP_PORT               3956U
#define TASK_GVCP_PRIO          2
// Runs the VMSP control handlers (camera setup over I2C, printf), as the control task.
#define TASK_GVCP_STK_SIZE      4096

void task_gvcp_entry(void *pvParameters);

#endif /* APP_GVCP_H_ */
