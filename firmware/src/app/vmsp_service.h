#ifndef VMSP_SERVICE_H
#define VMSP_SERVICE_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

// Ports per STP-PAK-026-NET-MVP (docs/protocol/VMSP_PROTOCOL_SPECIFICATION.md,
// docs/DECISIONS.md R-01): 39151 VMSP control,
// 39152 VMSP data (frames on request 0x0220), UDP 39150 discovery.
// There are no other ports: the text port 5001 from the STP and the former
// streamer 5002 were removed (R-01). Frames go on request (R-04); the later
// streams are the profile stream 0x0212 (R-20); frames over UDP
// go only as GVSP (gvsp.c) since 03.11.2026.
#define NET_VMSP_CONTROL_PORT   39151
#define NET_VMSP_VIDEO_PORT     39152

// VMSP task configuration
#define TASK_VMSP_CONTROL_PRIO  2
// CaptureImage runs the camera arming/trigger path (I2C + verbose printf) and
// the 1.1 MB frame send on this task, so it needs the same headroom as the
// legacy control task, not the 2048 words a pure ACK handler used to need.
#define TASK_VMSP_CONTROL_STK_SIZE 4096
#define TASK_VMSP_VIDEO_PRIO    2
// The data channel runs the capture itself (I2C, printf), as the control one used to.
#define TASK_VMSP_VIDEO_STK_SIZE 4096
#define TASK_VMSP_DISCOVERY_PRIO 3
#define TASK_VMSP_DISCOVERY_STK_SIZE 2048
extern TaskHandle_t TASK_VMSP_CONTROL_Handler;
extern TaskHandle_t TASK_VMSP_VIDEO_Handler;

void task_vmsp_control_entry(void *pvParameters);
void task_vmsp_discovery_entry(void *pvParameters);
void task_vmsp_video_entry(void *pvParameters);

// A control opcode (port 39151) run in-process by the caller's task, for GVCP (gvcp.c):
// the same handler, checks and locks as over TCP. The answer body (without the VMSP
// header) goes into out; returns its length, -1 if it did not fit. One caller task only.
int vmsp_call(uint16_t opcode, const uint8_t *body, uint32_t len, uint8_t *out, uint32_t cap);

#endif /* VMSP_SERVICE_H */
