/*
 * ******************************************************************************************
 * File		: main.c
 * Author 	: GowinSemiconductor / User
 * Chip		: AE350_SOC
 * Function	: start-up: hardware, lwIP, FreeRTOS primitives, camera, console
 *			  and VMSP tasks. Console commands: console.c, protocol: vmsp_service.c
 * ******************************************************************************************
 */

// Includes ---------------------------------------------------------------------------------
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"
#include "Driver_GPIO.h"
#include "uart.h"
#include "cache.h"   // ae350_dcache_invalidate_range(), used by the 'V' command

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "FreeRTOSConfig.h"

#ifndef OTA_LITE_PROFILE
#include "camera.h"
#include "i2c_bus.h"
#include "settings.h"
#include "laser.h"
#include "mm_table.h"
#include "work_mode.h"
#include "gvcp.h"
#include "gvsp.h"
#include "frame_udp.h"
#include "hik_client.h"
#endif
#include "vmsp_service.h"
#include "board.h"
#include "console.h"
#include "net_config.h"
#include "nvstore.h"
#include "ota.h"
#include "spi_flash.h"
#include "lcd_hud.h"
#include "arch/sys_arch.h"
#include "arch/ethernetif.h"
#include "ethernet.h"
#include "watchdog.h"


// Definitions ------------------------------------------------------------------------------
#define SW_STR_NAME							"FreeRTOS-AE350_DualTask_CamI2C_UART"
#define SW_STR_EDITION						"V1.0.0"
#define SW_STR_AUTHOR						"GowinSemiconductor"



// FreeRTOS primitives live with their owners: the I2C bus lock in i2c_bus.c,
// the camera state in camera.c (both created in camera_init()), xEthTxMutex in
// ethernet.c (created here, in main()).

#ifndef OTA_LITE_PROFILE
// Task Handles
#define TASK_CAM0_PRIO			1
#define CAM_REPROBE_MS			10000U	// a camera missing at boot is probed again this often
#define TASK_CAM0_STK_SIZE		2048
TaskHandle_t TASK_CAM0_Handler;
static void task_cam0_poll(void *pvParameters);

#define TASK_CAM1_PRIO			1
#define TASK_CAM1_STK_SIZE		2048
TaskHandle_t TASK_CAM1_Handler;
static void task_cam1_poll(void *pvParameters);
#endif

#define TASK_CONSOLE_PRIO			1
#define TASK_CONSOLE_STK_SIZE		4096
TaskHandle_t TASK_CONSOLE_Handler;
static void task_console(void *pvParameters);


extern AE350_DRIVER_GPIO 		Driver_GPIO;	// GPIO driver

#define LED0_PIN				0				// LED0 (Toggled by Cam0 Task)
#define LED1_PIN				1				// LED1 (Toggled by UART Task)
volatile int led0_flag = 0;
volatile int led1_flag = 0;

static void prvSetupHardware( void )
{
	/* Ensure no interrupts execute while the scheduler is in an inconsistent state. */
	portDISABLE_INTERRUPTS();
}

// GPIO callback
void gpio_callback(uint32_t event)
{
	(void)event;
}

// The FPGA capture path is enabled by its own post-reset quiet-window gate.
// GPIO[8:6] select the active camera through the Tang Mega TWI multiplexer.
// GPIO[4:0] are LEDs.
// Initializes GPIO pins for LEDs and camera multiplexer control
static void gpio_init(void)
{
	AE350_DRIVER_GPIO *GPIO_Dri = &Driver_GPIO;

	GPIO_Dri->Initialize(gpio_callback);
	// GPIO[8:6] drive the three TWI multiplexer select lines so the camera bus can
	// be scanned; GPIO[4:0] are the LEDs.
	GPIO_Dri->SetDir((1 << LED0_PIN) | (1 << LED1_PIN)
#ifndef OTA_LITE_PROFILE
	                 | (1 << I2C_MUX_S0_PIN) | (1 << I2C_MUX_S1_PIN) | (1 << I2C_MUX_S2_PIN)
#endif
	                 , AE350_GPIO_DIR_OUTPUT);
	GPIO_Dri->PinWrite(LED0_PIN, 0);	// LED0 off
	GPIO_Dri->PinWrite(LED1_PIN, 0);	// LED1 off
}

#ifndef OTA_LITE_PROFILE
/*
 * =========================================================================================
 * TASK: Camera 0 Task (Connector 0, GPIO6 = LOW) - ON DEMAND
 * =========================================================================================
 * - Performs initial probe at boot, then sleeps until explicitly requested (On-Demand).
 * - Zero periodic I2C bus traffic and 0% CPU consumption during live streaming.
 */
static void task_cam0_poll(void *pvParameters)
{
	(void)pvParameters;
	AE350_DRIVER_GPIO *GPIO_Dri = &Driver_GPIO;
	bool is_first_run = true;
	bool found = false;

	while (1)
	{
		if (!is_first_run)
		{
			// Found: sleep until asked. Missing: probe again every CAM_REPROBE_MS, so a
			// camera that came up late is not lost until a reboot.
			ulTaskNotifyTake(pdTRUE, found ? portMAX_DELAY : pdMS_TO_TICKS(CAM_REPROBE_MS));
		}

		bool connected = false;
		uint32_t model_id = 0, sensor_w = 0, sensor_h = 0;
		uint8_t trg_mode = 0, trg_src = 0, auto_exp = 0;
		uint32_t exp = 0, gain = 0, fps = 0, max_fps = 0, err_code = 0;
		uint8_t testpat = 0;
		uint32_t roi_w = 0, roi_h = 0, off_x = 0, off_y = 0;

		// Settings from settings store, before the scan and before camera_boot_reset():
		// the camera is set up from the already loaded SERVICE set. The scan follows
		// and sets the multiplexer to the camera itself. A repeated call does nothing.
		settings_boot_load();

		// Scan TWI multiplexer for Camera 0
		if (i2c_bus_lock(2000))
		{
			camera_scan_mux(0);
			i2c_bus_unlock();
		}

		// Try probing Camera 0 with up to 5 retries
		for (int retry = 0; retry < 5 && !connected; retry++)
		{
			if (i2c_bus_lock(500))
			{
				camera_read_config(0, &connected, &model_id, &sensor_w, &sensor_h,
				                   &trg_mode, &trg_src,
				                   &auto_exp, &exp, &gain, &fps, &max_fps, &err_code, &testpat,
				                   &roi_w, &roi_h, &off_x, &off_y);

				if (connected)
				{
					printf("[CAMERA] Cam 0 (Connector 0): DETECTED (Model=0x%04X, Sensor=%ux%u, Exp=%uus, Gain=%u.%udB, FPS=%u.%uHz)\r\n",
					       (unsigned int)model_id, (unsigned int)sensor_w, (unsigned int)sensor_h,
					       (unsigned int)exp, (unsigned int)(gain / 10), (unsigned int)(gain % 10), (unsigned int)(fps / 100), (unsigned int)(fps % 100));

					// Not just a stop: the camera remembers what was done to it before the FPGA
					// reboot (ROI, 120 fps, stream), ERRORS 2.6.
					{
						camera_params_t boot;
						work_mode_boot_cam(0, &boot);
						camera_boot_reset(0, &boot);
					}
				}
				i2c_bus_unlock();
			}
			if (!connected)
			{
				vTaskDelay(pdMS_TO_TICKS(500));
			}
		}

		if (!connected && is_first_run)
		{
			printf("[CAMERA] Cam 0 (Connector 0): NOT DETECTED after retries, probing every %u s\r\n",
			       (unsigned)(CAM_REPROBE_MS / 1000U));
		}
		is_first_run = false;
		found = connected;

		// The last read parameters go into the shared camera state (camera.c).
		{
			camera_params_t st;
			st.connected = connected;
			st.model_id  = model_id;
			st.sensor_w  = sensor_w;
			st.sensor_h  = sensor_h;
			st.trg_mode  = trg_mode;
			st.trg_src   = trg_src;
			st.auto_exp  = auto_exp;
			st.exp       = exp;
			st.gain      = gain;
			st.fps       = fps;
			st.max_fps   = max_fps;
			st.err_code  = err_code;
			st.testpat   = testpat;
			st.roi_w     = roi_w;
			st.roi_h     = roi_h;
			st.offset_x  = off_x;
			st.offset_y  = off_y;
			camera_state_store(0, &st);
		}

		// Toggle LED0 to indicate Cam0 activity
		led0_flag = !led0_flag;
		GPIO_Dri->PinWrite(LED0_PIN, led0_flag);
	}
}

/*
 * =========================================================================================
 * TASK: Camera 1 Task (Connector 1) - ON DEMAND
 * =========================================================================================
 * - Performs initial probe at boot, then sleeps until explicitly requested (On-Demand).
 * - Zero periodic I2C bus traffic and 0% CPU consumption during live streaming.
 */
static void task_cam1_poll(void *pvParameters)
{
	(void)pvParameters;
	bool is_first_run = true;

	// Stagger initial probe by 500ms so Cam0 completes its boot probe first
	vTaskDelay(pdMS_TO_TICKS(500));

	bool found = false;
	while (1)
	{
		if (!is_first_run)
		{
			// As camera 0: sleep when found, probe again every CAM_REPROBE_MS when missing.
			ulTaskNotifyTake(pdTRUE, found ? portMAX_DELAY : pdMS_TO_TICKS(CAM_REPROBE_MS));
		}

		bool connected = false;
		uint32_t model_id = 0, sensor_w = 0, sensor_h = 0;
		uint8_t trg_mode = 0, trg_src = 0, auto_exp = 0;
		uint32_t exp = 0, gain = 0, fps = 0, max_fps = 0, err_code = 0;
		uint8_t testpat = 0;
		uint32_t roi_w = 0, roi_h = 0, off_x = 0, off_y = 0;

		// Usually already loaded by the camera 0 task; if it did not get the ring,
		// this one loads it (see settings_boot_load()).
		settings_boot_load();

		// Scan TWI multiplexer for Camera 1 (skipping Cam 0's mux position)
		if (i2c_bus_lock(2000))
		{
			camera_scan_mux(1);
			i2c_bus_unlock();
		}

		// Try probing Camera 1 with up to 5 retries
		for (int retry = 0; retry < 5 && !connected; retry++)
		{
			if (i2c_bus_lock(500))
			{
				camera_read_config(1, &connected, &model_id, &sensor_w, &sensor_h,
				                   &trg_mode, &trg_src,
				                   &auto_exp, &exp, &gain, &fps, &max_fps, &err_code, &testpat,
				                   &roi_w, &roi_h, &off_x, &off_y);

				if (connected)
				{
					printf("[CAMERA] Cam 1 (Connector 1): DETECTED (Model=0x%04X, Sensor=%ux%u, Exp=%uus, Gain=%u.%udB, FPS=%u.%uHz)\r\n",
					       (unsigned int)model_id, (unsigned int)sensor_w, (unsigned int)sensor_h,
					       (unsigned int)exp, (unsigned int)(gain / 10), (unsigned int)(gain % 10), (unsigned int)(fps / 100), (unsigned int)(fps % 100));

					// Capture stopped and settings known (ERRORS 2.6)
					{
						camera_params_t boot;
						work_mode_boot_cam(1, &boot);
						camera_boot_reset(1, &boot);
					}
				}
				i2c_bus_unlock();
			}
			if (!connected)
			{
				vTaskDelay(pdMS_TO_TICKS(500));
			}
		}

		if (!connected && is_first_run)
		{
			printf("[CAMERA] Cam 1 (Connector 1): NOT DETECTED after retries, probing every %u s\r\n",
			       (unsigned)(CAM_REPROBE_MS / 1000U));
		}
		is_first_run = false;
		found = connected;

		// The last read parameters go into the shared camera state (camera.c).
		{
			camera_params_t st;
			st.connected = connected;
			st.model_id  = model_id;
			st.sensor_w  = sensor_w;
			st.sensor_h  = sensor_h;
			st.trg_mode  = trg_mode;
			st.trg_src   = trg_src;
			st.auto_exp  = auto_exp;
			st.exp       = exp;
			st.gain      = gain;
			st.fps       = fps;
			st.max_fps   = max_fps;
			st.err_code  = err_code;
			st.testpat   = testpat;
			st.roi_w     = roi_w;
			st.roi_h     = roi_h;
			st.offset_x  = off_x;
			st.offset_y  = off_y;
			camera_state_store(1, &st);
		}
	}
}
#endif /* !OTA_LITE_PROFILE */



/*
 * =========================================================================================
 * TASK: LED Heartbeat (Slot-dependent pattern)
 * - Slot A (Golden): Calm 1 Hz alternating (LED0 / LED1)
 * - Slot B (Candidate): Rapid distinctive synchronous double-strobe
 * =========================================================================================
 */
static void task_led_heartbeat(void *pvParameters)
{
	(void)pvParameters;
	AE350_DRIVER_GPIO *GPIO_Dri = &Driver_GPIO;

	for (;;) {
		uint8_t slot = ota_current_slot();
		if (slot == 1U) {
#ifdef OTA_LITE_PROFILE
			/* Lite Bootloader Probe: Fast 8 Hz alternating LED0 <-> LED1 */
			GPIO_Dri->PinWrite(LED0_PIN, 1);
			GPIO_Dri->PinWrite(LED1_PIN, 0);
			vTaskDelay(pdMS_TO_TICKS(62));

			GPIO_Dri->PinWrite(LED0_PIN, 0);
			GPIO_Dri->PinWrite(LED1_PIN, 1);
			vTaskDelay(pdMS_TO_TICKS(63));
#else
			/* Slot B (Candidate): Rapid double-strobe
			 * Pattern: Flash (80ms) -> Off (80ms) -> Flash (80ms) -> Off (760ms)
			 */
			GPIO_Dri->PinWrite(LED0_PIN, 1);
			GPIO_Dri->PinWrite(LED1_PIN, 1);
			vTaskDelay(pdMS_TO_TICKS(80));

			GPIO_Dri->PinWrite(LED0_PIN, 0);
			GPIO_Dri->PinWrite(LED1_PIN, 0);
			vTaskDelay(pdMS_TO_TICKS(80));

			GPIO_Dri->PinWrite(LED0_PIN, 1);
			GPIO_Dri->PinWrite(LED1_PIN, 1);
			vTaskDelay(pdMS_TO_TICKS(80));

			GPIO_Dri->PinWrite(LED0_PIN, 0);
			GPIO_Dri->PinWrite(LED1_PIN, 0);
			vTaskDelay(pdMS_TO_TICKS(760));
#endif
		} else {
			/* Slot A (Golden): Calm 1 Hz alternating
			 * 0..500 ms: LED0 ON, LED1 OFF
			 * 500..1000 ms: LED0 OFF, LED1 ON
			 */
			GPIO_Dri->PinWrite(LED0_PIN, 1);
			GPIO_Dri->PinWrite(LED1_PIN, 0);
			vTaskDelay(pdMS_TO_TICKS(500));

			GPIO_Dri->PinWrite(LED0_PIN, 0);
			GPIO_Dri->PinWrite(LED1_PIN, 1);
			vTaskDelay(pdMS_TO_TICKS(500));
		}
	}
}

/*
 * =========================================================================================
 * TASK: console - PHY at start-up, LEDs, an [ETH STATS] line every second,
 *       UART command input (console_command(), console.c)
 * =========================================================================================
 */
static void task_console(void *pvParameters)
{
	(void)pvParameters;

	printf("\r\n[Task UART] Memory Validator & Heartbeat active!\r\n");

	// Initial Diagnostics & PHY Configuration on boot
	eth_phy_init(0);

	// Wait for PHY auto-negotiation to complete (1Gbps requires ~1-3 seconds)
	printf("[Task UART] Waiting 3s for PHY auto-negotiation...\r\n");
	vTaskDelay(pdMS_TO_TICKS(3000));

	// The address from the store, or the board.h default (net_config.c, 15.10.2026).
	(void)net_config_boot();

	extern struct netif gnetif;
	netif_set_link_up(&gnetif);
	netif_set_up(&gnetif);
	DEV_ETHERNET->ETH_RX_IC = BIT0;
	DEV_ETHERNET->ETH_RX_IE = 1;
	printf("[Task UART] Network interface gnetif brought UP!\r\n");
	ota_init();
	ota_confirm_boot();
	lcd_hud_init();
#ifndef OTA_LITE_PROFILE
	hik_client_init();
#endif

	printf("\r\n**************************************************************\r\n");
	printf(" * TANG MEGA 138K PRO - %s\r\n", ota_get_version_string());
	printf(" * Running Slot:  %s\r\n", ota_current_slot() ? "SLOT B (Candidate 0xF60000)" : "SLOT A (Golden 0x600000)");
	printf(" * LED Heartbeat: %s\r\n", ota_current_slot() ? "DOUBLE-STROBE (Flash-Flash 80ms, 1 Hz)" : "CALM-1HZ (Alternating 500ms)");
	printf("**************************************************************\r\n\r\n");

	// FPGA capture is NOT enabled here: GPIO[5] is raised by task_cam1_poll once the
	// sensor is confirmed streaming, so the FPGA never writes DDR3 before the CPU has
	// finished booting (a full-rate capture starves the core on the shared AHB bus).

	while (1)
	{

		extern volatile uint32_t g_eth_rx_cnt;
		extern volatile uint32_t g_eth_rx_pbuf_cnt;
		extern volatile uint32_t g_eth_tx_cnt;

		// PLIC and RX-watchdog counters are appended, not inserted, so tools that
		// parse "RX_ISR=... TX_CNT=..." keep working (ERRORS former 31.29).
		extern volatile uint32_t g_gp0_irq_cnt;
		extern volatile uint32_t g_plic_claim_src4;
		extern volatile uint32_t g_plic_complete_src4;
		extern volatile uint32_t g_plic_claim_zero;
		extern volatile uint32_t g_eth_wd_complete;
		extern volatile uint32_t g_eth_wd_drain;
		extern volatile uint32_t g_eth_wd_rxic;
		extern volatile uint32_t g_eth_wd_late, g_eth_wd_max_cons;
		extern volatile uint32_t g_eth_fast_lost, g_eth_fast_late, g_eth_fast_arms, g_eth_fast_ticks, g_eth_fast_cyc;

		// The link comes from the PHY (PHYSR) once a second. Until 06.10.2026 this was
		// a constant "Link=UP 1000M FD", and all "link UP" records during receive
		// failures (ERRORS 3.1, 3.7) did not measure it (ERRORS 3.10). drops counts how
		// many times the BMSR latch showed a link drop since boot; appended to the end
		// of the line.
		static uint32_t s_link_drops = 0;
		eth_phy_link_t ls;
		char link_txt[24];
		// Polling the PHY once a second is DISABLED: A/B on 06.10.2026 (ERRORS 3.10):
		// with it 6 boots of 6 were sick (WD cmp up to 109 in 20 s, ping 4/20), without
		// it 2 of 6 and 2 of 8. Why MIIM interferes with delivery of the receive
		// interrupt is not established (MIIM_IS on sick boots = 0). Link state is on
		// request, console 'Y'; the MAC IS/IE flags are at the end of this line.
#ifndef ETH_STATS_PHY_POLL
#define ETH_STATS_PHY_POLL 0
#endif
		if (!ETH_STATS_PHY_POLL)
			snprintf(link_txt, sizeof(link_txt), "n/a");
		else if (!eth_phy_link_status(0, &ls))
			snprintf(link_txt, sizeof(link_txt), "PHY?");
		else
		{
			if (ls.dropped) ++s_link_drops;
			if (ls.link)
				snprintf(link_txt, sizeof(link_txt), "UP %uM %s",
				         (unsigned int)ls.speed_mbps, ls.full_duplex ? "FD" : "HD");
			else
				snprintf(link_txt, sizeof(link_txt), "DOWN");
		}

		printf("[ETH STATS] Tick=%u | RX_ISR=%u, RX_PBUF=%u, TX_CNT=%u | Link=%s"
		       " | IRQ=%u CLM4=%u CMP4=%u CLM0=%u | WD cmp=%u drain=%u rxic=%u late=%u maxc=%u | FAST lost=%u late=%u arms=%u ms=%u ticks=%u cyc/tick=%u | link drops %u"
		       " | MAC IS miim %X rx %X tx %X IE miim %X rx %X tx %X\r\n",
		       (unsigned int)xTaskGetTickCount(),
		       (unsigned int)g_eth_rx_cnt,
		       (unsigned int)g_eth_rx_pbuf_cnt,
		       (unsigned int)g_eth_tx_cnt,
		       link_txt,
		       (unsigned int)g_gp0_irq_cnt,
		       (unsigned int)g_plic_claim_src4,
		       (unsigned int)g_plic_complete_src4,
		       (unsigned int)g_plic_claim_zero,
		       (unsigned int)g_eth_wd_complete,
		       (unsigned int)g_eth_wd_drain,
		       (unsigned int)g_eth_wd_rxic,
		       (unsigned int)g_eth_wd_late,
		       (unsigned int)g_eth_wd_max_cons,
		       (unsigned int)g_eth_fast_lost,
		       (unsigned int)g_eth_fast_late,
		       (unsigned int)g_eth_fast_arms,
		       (unsigned int)eth_fast_armed_ms(),
		       (unsigned int)g_eth_fast_ticks,
		       (unsigned int)(g_eth_fast_ticks ? g_eth_fast_cyc / g_eth_fast_ticks : 0U),
		       (unsigned int)s_link_drops,
		       (unsigned int)DEV_ETHERNET->MIIM_IS, (unsigned int)DEV_ETHERNET->ETH_RX_IS,
		       (unsigned int)DEV_ETHERNET->ETH_TX_IS, (unsigned int)DEV_ETHERNET->MIIM_IE,
		       (unsigned int)DEV_ETHERNET->ETH_RX_IE, (unsigned int)DEV_ETHERNET->ETH_TX_IE);

		// Transmit anomalies that must not happen: frames over the 1536 B window refused, and
		// waits for ETH_TX_EN that hit their limit (ethernetif.c). Printed only when they grow.
		{
			extern volatile uint32_t g_eth_tx_busy_to, g_eth_tx_oversize;
			static uint32_t p_busy_to, p_over;
			if (g_eth_tx_oversize != p_over || g_eth_tx_busy_to != p_busy_to)
			{
				printf("[ETH TX] frames over 1536 B refused %u, ETH_TX_EN wait limit hit %u\r\n",
				       (unsigned int)(g_eth_tx_oversize - p_over), (unsigned int)(g_eth_tx_busy_to - p_busy_to));
				p_over = g_eth_tx_oversize;
				p_busy_to = g_eth_tx_busy_to;
			}
		}

		// One PHY/EEE/lwIP report 25 s after the network is up: after the 20 s
		// window that judges a boot (WD cmp), so its MIIM reads cannot affect the
		// verdict (polling the PHY made boots sick, ERRORS 3.10; ERRORS 3.15).
		{
			static uint32_t s_loops = 0;
			if (++s_loops == 25) console_phy_boot_report();
		}

		// Check for incoming UART commands
		while (DEV_UART2->LSR & 0x01)
		{
			int cmd = DEV_UART2->RBR;
			console_command(cmd);
		}

		lcd_hud_tick(g_eth_rx_cnt, g_eth_tx_cnt);
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}



#ifndef OTA_LITE_PROFILE
// The "calibration" storage partition: the 72 B of 0x0130, as is (laser.c). The
// adapters are here so that neither laser nor settings knows about the other.
static uint16_t calib_payload(uint8_t *out)
{
	if (!laser_calib_valid()) return 0;
	return (uint16_t)laser_calib_get(out, LASER_CALIB_BYTES);
}

static int calib_take(const uint8_t *p, uint16_t len, bool strict, uint32_t *bad)
{
	(void)strict;
	if (bad) *bad = 0;
	return laser_calib_set(p, len);
}

typedef char calib_section_check[
	(SETTINGS_HDR_BYTES + LASER_CALIB_BYTES <= SETTINGS_SLOT_BYTES) ? 1 : -1];

static const settings_section_t k_calib_section = {
	calib_payload, calib_take, NULL, NULL, NULL, NULL, LASER_CALIB_BYTES
};
#endif /* !OTA_LITE_PROFILE */

// Drains the printf ring into the UART (uart.c): every tick while text is waiting,
// every 10 ms when idle. Dropped characters are reported once the ring is empty.
static void task_log(void *pvParameters)
{
	(void)pvParameters;
	uart_log_start();
	for (;;) {
		if (uart_log_pump()) {
			vTaskDelay(1);
			continue;
		}
		unsigned int d = uart_log_take_dropped();
		if (d) printf("[LOG] ring full: %u characters dropped\r\n", d);
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}

// Main entry function
int main(void)
{
	prvSetupHardware();

	/* Initializes hardware peripherals */
	uart_init(115200);		// Initializes UART (115200 baud)
	gpio_init();			// Initializes GPIO for status LEDs
	printf("\r\n[BOOT] Andes AE350 SoC booting...\r\n");
	watchdog_boot_report();

	/* Configure Ethernet Hardware Interrupt on PLIC */
	HAL_INTERRUPT_SET_LEVEL(IRQ_GP0_SOURCE, 2);
	HAL_INTERRUPT_ENABLE(IRQ_GP0_SOURCE);
	HAL_MEIP_ENABLE();

	printf("[BOOT] Initializing lwIP TCP/IP Stack (default IP %u.%u.%u.%u, the store one is applied before the netif is up)...\r\n",
	       NET_IP_ADDR0, NET_IP_ADDR1, NET_IP_ADDR2, NET_IP_ADDR3);
	LwIP_Init();
	printf("[BOOT] lwIP Initialization completed successfully!\r\n");

#ifdef OTA_LITE_PROFILE
	spi_flash_init();
	nv_init();
#else
	// Probe hardware and initialize cameras over I2C
	camera_init();

	// The UDP frame delivery descriptor (udp_tx in the FPGA): zero it before the
	// end of the FPGA's quiet window; after power-up DDR3 holds garbage, after a
	// firmware restart the previous job.
	frame_udp_init();

	// "pixel -> mm" table (R-38): MMTB with both cameras off before anything
	// configures a camera - after a firmware restart DDR3 still holds the last
	// block, and the FPGA would load a band this firmware knows nothing about.
	// Also hooks the camera ROI listener.
	mm_init();
	nv_init();          // settings store in flash (nvstore.h): after spi_flash_init in mm_init
#endif

	// NOTE: do not touch the sensor over I2C here. camera_select_channel()
	// drives a fixed TWI mux position, but the camera is only located by the
	// mux scan that task_cam0_poll runs after the scheduler starts (this module
	// answers on position 3). An I2C transfer issued before that scan leaves the
	// bus on the wrong mux leg and the scan then fails to find the camera at
	// all. Acquisition is stopped by task_cam0_poll once detection succeeds.

	printf("[BOOT] Creating FreeRTOS primitives...\r\n");
	// Create FreeRTOS synchronization primitives (Mutexes & Queue)
	xEthTxMutex = xSemaphoreCreateMutex();
	configASSERT(xEthTxMutex != NULL);


#ifndef OTA_LITE_PROFILE
	// Settings storage: partition owners and the write rule (settings.h).
	settings_register(SETTINGS_SEC_CFG, &work_mode_settings_section);
	settings_register(SETTINGS_SEC_CALIB, &k_calib_section);
	settings_set_guard(work_mode_settings_writable);
#endif

	printf("[BOOT] Creating FreeRTOS tasks...\r\n");
#ifndef OTA_LITE_PROFILE
	// Task 1: Camera 0 Polling Task (Offset 100ms)
	configASSERT(xTaskCreate((TaskFunction_t )task_cam0_poll,
				(const char *   )"Task_Cam0_Poll",
				(const uint16_t )TASK_CAM0_STK_SIZE,
				(void * const   )NULL,
				(UBaseType_t    )TASK_CAM0_PRIO,
				(TaskHandle_t *  )&TASK_CAM0_Handler) == pdPASS);

	// Task 2: Camera 1 Polling Task (Offset 600ms - 500ms time separation!)
	configASSERT(xTaskCreate((TaskFunction_t )task_cam1_poll,
				(const char *   )"Task_Cam1_Poll",
				(const uint16_t )TASK_CAM1_STK_SIZE,
				(void * const   )NULL,
				(UBaseType_t    )TASK_CAM1_PRIO,
				(TaskHandle_t *  )&TASK_CAM1_Handler) == pdPASS);
#endif

	// Task 3: UART console ([ETH STATS], commands)
	configASSERT(xTaskCreate((TaskFunction_t )task_console,
				(const char *   )"Task_Console",
				(const uint16_t )TASK_CONSOLE_STK_SIZE,
				(void * const   )NULL,
				(UBaseType_t    )TASK_CONSOLE_PRIO,
				(TaskHandle_t *  )&TASK_CONSOLE_Handler) == pdPASS);

	// Task: LED Heartbeat (Slot-dependent pattern: Calm 1Hz for Slot A, Double-Strobe for Slot B)
	configASSERT(xTaskCreate((TaskFunction_t )task_led_heartbeat,
				(const char *   )"Task_LED_HB",
				(const uint16_t )configMINIMAL_STACK_SIZE,
				(void * const   )NULL,
				(UBaseType_t    )1,
				(TaskHandle_t *  )NULL) == pdPASS);

	// printf output (uart.c log ring, ERRORS 3.18): above the VMSP tasks so the log keeps
	// flowing under a stream; it only copies up to 16 bytes into the UART FIFO per wake.
	if (xTaskCreate(task_log, "Task_Log", configMINIMAL_STACK_SIZE * 2, NULL, 3, NULL) != pdPASS)
		printf("[BOOT] log task NOT created: printf stays blocking\r\n");

	// Task 4: VMSP UDP Discovery (port 39150)
	configASSERT(xTaskCreate((TaskFunction_t )task_vmsp_discovery_entry,
				(const char *   )"Task_VMSP_Discovery",
				(const uint16_t )TASK_VMSP_DISCOVERY_STK_SIZE,
				(void * const   )NULL,
				(UBaseType_t    )TASK_VMSP_DISCOVERY_PRIO,
				(TaskHandle_t *  )NULL) == pdPASS);

	// The text TCP server 5001 and the former streamer 5002 were removed
	// (15.09.2026): all control and delivery is VMSP on 39151/39152, the duplicate
	// debug functionality was removed (docs/DECISIONS.md R-01).
	// lwIP answers ARP and ICMP ping itself, no separate task is needed for them.

	// Task 6: VMSP Control Server (TCP 39151)
	configASSERT(xTaskCreate((TaskFunction_t )task_vmsp_control_entry,
				(const char *   )"Task_VMSP_Control",
				(const uint16_t )TASK_VMSP_CONTROL_STK_SIZE,
				(void * const   )NULL,
				(UBaseType_t    )TASK_VMSP_CONTROL_PRIO,
				(TaskHandle_t *  )&TASK_VMSP_CONTROL_Handler) == pdPASS);

	// Task 7: VMSP Data (TCP 39152): frame delivery / memory dump on request.
	configASSERT(xTaskCreate((TaskFunction_t )task_vmsp_video_entry,
				(const char *   )"Task_VMSP_Data",
				(const uint16_t )TASK_VMSP_VIDEO_STK_SIZE,
				(void * const   )NULL,
				(UBaseType_t    )TASK_VMSP_VIDEO_PRIO,
				(TaskHandle_t *  )&TASK_VMSP_VIDEO_Handler) == pdPASS);

#ifndef OTA_LITE_PROFILE
	// GigE Vision streaming (GVSP): the frame scheduler feeding udp_tx (stage 3, gvsp.c).
	gvsp_init();
	configASSERT(xTaskCreate((TaskFunction_t )task_gvsp_entry,
				(const char *   )"Task_GVSP",
				(const uint16_t )TASK_GVSP_STK_SIZE,
				(void * const   )NULL,
				(UBaseType_t    )TASK_GVSP_PRIO,
				(TaskHandle_t *  )NULL) == pdPASS);

	// GigE Vision control (GVCP, UDP 3956) beside VMSP: docs/protocol/GEV_PROTOCOL_DESIGN.md, stage 2.
	configASSERT(xTaskCreate((TaskFunction_t )task_gvcp_entry,
				(const char *   )"Task_GVCP",
				(const uint16_t )TASK_GVCP_STK_SIZE,
				(void * const   )NULL,
				(UBaseType_t    )TASK_GVCP_PRIO,
				(TaskHandle_t *  )NULL) == pdPASS);
#endif

	// Supervisor task (OPEN_ISSUES 1.2, ERRORS 5.13): resets the SoC when a watched task hangs.
	watchdog_start();

	printf("[BOOT] Starting FreeRTOS scheduler...\r\n");

	// Start FreeRTOS Scheduler
	vTaskStartScheduler();

	while (1);

	return 0;
}

