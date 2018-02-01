

#ifndef _DSM_PUB_H
#define _DSM_PUB_H
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/workqueue.h>
#include <linux/wait.h>

#define CLIENT_NAME_LEN						(32) /* max client name length */
#define DSM_MAX_DEVICE_NAME_LEN				(32) /* max device name length */
#define DSM_MAX_MODULE_NAME_LEN				(4)  /* max module name length */
#define DSM_MAX_IC_NAME_LEN					(4)  /* max ic name length */

/*dsm error no define*/
#define DSM_ERR_NO_ERROR					(0)
#define DSM_ERR_I2C_TIMEOUT					(1)

#define DSM_PMU_IRQ_ERROR_NO				(10000)
#define DSM_PMU_VSYS_OV_D200UR_ERROR_NO          (DSM_PMU_IRQ_ERROR_NO + 2)
#define DSM_PMU_VSYS_UV_D10MR_ERROR_NO           (DSM_PMU_IRQ_ERROR_NO + 3)
#define DSM_PMU_VSYS_PWROFF_ABS_2D_ERROR_NO      (DSM_PMU_IRQ_ERROR_NO + 4)
#define DSM_PMU_VSYS_PWROFF_DEB_D80MR_ERROR_NO   (DSM_PMU_IRQ_ERROR_NO + 5)
#define DSM_PMU_VSYS_THSD_OTMP140_D1MR_ERROR_NO  (DSM_PMU_IRQ_ERROR_NO + 6)
#define DSM_PMU_VSYS_THSD_OTMP125_D1MR_ERROR_NO  (DSM_PMU_IRQ_ERROR_NO + 7)

#define DSM_PMU_OCP_ERROR_NO_BASE			(10100)
#define DSM_PMU_OCPLDO2_ERROR_NO            (DSM_PMU_OCP_ERROR_NO_BASE + 0)
#define DSM_PMU_OCPLDO1_ERROR_NO            (DSM_PMU_OCP_ERROR_NO_BASE + 1)
#define DSM_PMU_OCPLDO0_ERROR_NO            (DSM_PMU_OCP_ERROR_NO_BASE + 2)
#define DSM_PMU_OCPBUCK4_ERROR_NO           (DSM_PMU_OCP_ERROR_NO_BASE + 3)
#define DSM_PMU_OCPBUCK3_ERROR_NO           (DSM_PMU_OCP_ERROR_NO_BASE + 4)
#define DSM_PMU_OCPBUCK2_ERROR_NO           (DSM_PMU_OCP_ERROR_NO_BASE + 5)
#define DSM_PMU_OCPBUCK1_ERROR_NO           (DSM_PMU_OCP_ERROR_NO_BASE + 6)
#define DSM_PMU_OCPBUCK0_ERROR_NO           (DSM_PMU_OCP_ERROR_NO_BASE + 7)
#define DSM_PMU_OCPLDO11_ERROR_NO           (DSM_PMU_OCP_ERROR_NO_BASE + 8)
#define DSM_PMU_OCPLDO10_ERROR_NO           (DSM_PMU_OCP_ERROR_NO_BASE + 9)
#define DSM_PMU_OCPLDO9_ERROR_NO            (DSM_PMU_OCP_ERROR_NO_BASE + 10)
#define DSM_PMU_OCPLDO8_ERROR_NO            (DSM_PMU_OCP_ERROR_NO_BASE + 11)
#define DSM_PMU_OCPLDO7_ERROR_NO            (DSM_PMU_OCP_ERROR_NO_BASE + 12)
#define DSM_PMU_OCPLDO5_ERROR_NO            (DSM_PMU_OCP_ERROR_NO_BASE + 13)
#define DSM_PMU_OCPLDO4_ERROR_NO            (DSM_PMU_OCP_ERROR_NO_BASE + 14)
#define DSM_PMU_OCPLDO3_ERROR_NO            (DSM_PMU_OCP_ERROR_NO_BASE + 15)
#define DSM_PMU_OCPLDO20_ERROR_NO           (DSM_PMU_OCP_ERROR_NO_BASE + 16)
#define DSM_PMU_OCPLDO19_ERROR_NO           (DSM_PMU_OCP_ERROR_NO_BASE + 17)
#define DSM_PMU_OCPLDO17_ERROR_NO           (DSM_PMU_OCP_ERROR_NO_BASE + 18)
#define DSM_PMU_OCPLDO16_ERROR_NO           (DSM_PMU_OCP_ERROR_NO_BASE + 19)
#define DSM_PMU_OCPLDO15_ERROR_NO           (DSM_PMU_OCP_ERROR_NO_BASE + 20)
#define DSM_PMU_OCPLDO14_ERROR_NO           (DSM_PMU_OCP_ERROR_NO_BASE + 21)
#define DSM_PMU_OCPLDO13_ERROR_NO           (DSM_PMU_OCP_ERROR_NO_BASE + 22)
#define DSM_PMU_OCPLDO12_ERROR_NO           (DSM_PMU_OCP_ERROR_NO_BASE + 23)
#define DSM_PMU_OCPLDO28_ERROR_NO           (DSM_PMU_OCP_ERROR_NO_BASE + 24)
#define DSM_PMU_OCPLDO27_ERROR_NO           (DSM_PMU_OCP_ERROR_NO_BASE + 25)
#define DSM_PMU_OCPLDO26_ERROR_NO           (DSM_PMU_OCP_ERROR_NO_BASE + 26)
#define DSM_PMU_OCPLDO25_ERROR_NO           (DSM_PMU_OCP_ERROR_NO_BASE + 27)
#define DSM_PMU_OCPLDO24_ERROR_NO           (DSM_PMU_OCP_ERROR_NO_BASE + 28)
#define DSM_PMU_OCPLDO23_ERROR_NO           (DSM_PMU_OCP_ERROR_NO_BASE + 29)
#define DSM_PMU_OCPLDO22_ERROR_NO           (DSM_PMU_OCP_ERROR_NO_BASE + 30)
#define DSM_PMU_OCPLDO21_ERROR_NO           (DSM_PMU_OCP_ERROR_NO_BASE + 31)
#define DSM_PMU_CLASSD_OCP_ERROR_NO         (DSM_PMU_OCP_ERROR_NO_BASE + 32)
#define DSM_PMU_OCPLDO32_ERROR_NO           (DSM_PMU_OCP_ERROR_NO_BASE + 36)
#define DSM_PMU_OCPLDO31_ERROR_NO           (DSM_PMU_OCP_ERROR_NO_BASE + 37)
#define DSM_PMU_OCPLDO30_ERROR_NO           (DSM_PMU_OCP_ERROR_NO_BASE + 38)
#define DSM_PMU_OCPLDO29_ERROR_NO           (DSM_PMU_OCP_ERROR_NO_BASE + 39)
#define DSM_PMU_BUCK4_SCP_ERROR_NO          (DSM_PMU_OCP_ERROR_NO_BASE + 40)
#define DSM_PMU_BUCK3_SCP_ERROR_NO          (DSM_PMU_OCP_ERROR_NO_BASE + 41)
#define DSM_PMU_BUCK2_SCP_ERROR_NO          (DSM_PMU_OCP_ERROR_NO_BASE + 42)
#define DSM_PMU_BUCK1_SCP_ERROR_NO          (DSM_PMU_OCP_ERROR_NO_BASE + 43)
#define DSM_PMU_BUCK0_SCP_ERROR_NO          (DSM_PMU_OCP_ERROR_NO_BASE + 44)

#define DSM_CHARGER_ERROR_NO  				(10200)
#define DSM_PMU_COUL_ERROR_NO				(10300)
#define DSM_BATTERY_ERROR_NO				(10300)
#define DSM_BATTERY_DETECT_ERROR_NO			(10400)
#define ERROR_NO_USB_SHORT_PROTECT          (10450)
#define ERROR_NO_USB_SHORT_PROTECT_NTC      (10451)
#define DSM_TP_I2C_RW_ERROR_NO 				(20000)
#define DSM_TP_FWUPDATE_ERROR_NO 			(20001)
#define DSM_TP_RAWDATA_ERROR_NO             (20002)
#define DSM_TP_FW_CRC_ERROR_NO              (20003)
#define DSM_TP_DEV_STATUS_ERROR_NO          (20004)
#define DSM_TP_CHANGE_REPORT_ERROR_NO       (20006)
#define DSM_TP_GLOVE_ON_COUNT_ERROR_NO      (20007)
#define DSM_TP_ATMEL_IN_MOISTURE_ERROR_NO   (20008)
#define DSM_TP_ATMEL_IN_VNOISE_ERROR_NO     (20009)
#define DSM_TP_CALIBRATION_CRC_ERROR_NO     (20010)
#define DSM_LCD_LDI_UNDERFLOW_NO            (20100)
#define DSM_TP_NO_IRQ_ERROR_NO				(20002)
#define DSM_LCD_LDI_UNDERFLOW_ERROR_NO     	(20100)
#define DSM_LCD_TE_TIME_OUT_ERROR_NO        (20101)
#define DSM_LCD_STATUS_ERROR_NO				(20102)
#define DSM_LCD_POWER_STATUS_ERROR_NO		(20103)
#define DSM_LCD_PWM_ERROR_NO				(20104)
#define DSM_LCD_BRIGHTNESS_ERROR_NO			(20105)
#define DSM_LCD_ESD_RECOVERY_NO				(20106)
#define DSM_LCD_ESD_OCP_RECOVERY_NO			(20107)
#define DSM_LCD_OVP_ERROR_NO				(20108)

#define DSM_SOC_HIFI_RESET                  (20201)
#define DSM_CODEC_HIFI_RESET                (20202)
#define DSM_CODEC_HIFI_IF_OPEN_ERR          (20204)
#define DSM_SOC_HIFI_HIGH_CPU               (20211)
#define DSM_SOC_HIFI_UPDATE_BUFF_DELAY      (20212)
#define DSM_HI6402_PLL_PD                   (20221)
#define DSM_HI6402_PLL_UNLOCK               (20222)
#define DSM_HI6402_PLL_CANNOT_LOCK          (20223)
#define DSM_HI6402_SLIMBUS_READ_ERR         (20224)
#define DSM_HI6402_SLIMBUS_LOST_MS          (20225)
#define DSM_HI6402_MBHC_HS_ERR_TYPE         (20226)
#define ANC_HS_OPEN_BOOST_ERR               (22201)
#define ANC_HS_CANCEL_WORK_BLOCK            (22202)
#define ANC_HS_CLOSE_BOOST_ERR              (22203)
#define ANC_HS_ADCH_READ_ERR                (22204)
#define ANC_HS_ADC_FULL_ERR                 (22205)
#define ANC_HS_MIC_WITH_GPIO_ERR            (22206)
#define ANC_HS_QUEUE_WORK_ERR               (22207)
#define ANC_HS_BTN_NOT_IN_RANGE             (22208)

#define DSM_SDIO_RW_ERROR_NO                        (22300)
#define DSM_SDIO_DCM_INIT_ERROR_NO                  (22301)
#define DSM_SDIO_HARDWARE_LOCK_NO                   (22302)
#define DSM_SDIO_RW_TIMEOUT_NO                      (22303)
#define DSM_SDIO_ATTACH_ERR_NO                      (22304)
#define DSM_SDIO_CDM52_INVELADE_ARGUMENT_ERR_NO     (22305)
#define DSM_SDIO_CDM52_R5_ERR_NO                    (22306)
#define DSM_SDIO_CDM52_R5_FUNCTION_NUMBER_ERR_NO    (22307)
#define DSM_SDIO_CDM52_R5_OUT_OF_RANGE_ERR_NO       (22308)
#define DSM_SDIO_CDM52__WAIT_FOR_CMD_ERR_NO         (22309)
#define DSM_SDIO_CMD53_INVELADE_ARGUMENT_ERR_NO     (22310)
#define DSM_SDIO_CMD53_ALLOC_TABLE_ERR_NO           (22311)
#define DSM_SDIO_CMD53_CMD_ERR_NO                   (22312)
#define DSM_SDIO_CMD53_DATA_ERR_NO                  (22313)
#define DSM_SDIO_CMD53_R5_ERR_NO                    (22314)
#define DSM_SDIO_CMD53_R5_FUNCTION_NUMBER_ERR_NO    (22315)
#define DSM_SDIO_CMD53_R5_OUT_OF_RANGE_ERR_NO       (22316)
#define DSM_SIDO_UNKOWN_ERR_NO                      (22317)

#define DSM_SHB_ERR_IPC_TIMEOUT				(20400)
#define DSM_SHB_ERR_IOM3_ACC				(20401)
#define DSM_SHB_ERR_IOM3_MAG				(20402)
#define DSM_SHB_ERR_IOM3_GYRO				(20403)
#define DSM_SHB_ERR_IOM3_LIGHT				(20404)
#define DSM_SHB_ERR_IOM3_OTHER				(20405)
#define DSM_SHB_ERR_IOM3_UNKNOW				(20406)
#define DSM_SHB_ERR_MAIN_INIT				(20407)
#define DSM_SHB_ERR_SERVER_INIT				(20408)
#define DSM_SHB_ERR_SERVER_UNINIT			(20409)
#define DSM_SHB_ERR_READ					(20410)
#define DSM_SHB_ERR_WRITE					(20411)
#define DSM_SHB_ERR_CMD						(20412)
#define DSM_SHB_ERR_CFG_DATA				(20413)
#define DSM_SHB_ERR_DYNLOAD					(20414)
#define DSM_SHB_ERR_I2C_DBG					(20415)
#define DSM_SHB_ERR_IOM3_STEP				(20416)
#define DSM_SHB_INVENSENSE_DMP_GET_FIFO_ALL (20417)
#define DSM_SHB_ERR_IOM3_ACC_EXCEED (20418)
#define DSM_SHB_ERR_INVENSENSE_DMP_FIFO_HANDLER_CONTROL_MUTEX (20419)
#define DSM_SHB_ERR_INVENSENSE_DMP_FIFO_HANDLER_DATA_MUTEX (20420)
#define DSM_SHB_ERR_INVENSENSE_DMP_ALL_ZERO_HEAD (20421)
#define DSM_SHB_ERR_IOM3_WDG                             (20422)

#define DSM_KEY_ERROR_NO					(20500)
#define DSM_POWER_KEY_ERROR_NO				(20501)
#if defined (CONFIG_HUAWEI_DSM)
#define DSM_COMPASS_I2C_RW_ERROR_NO			(21100)
#define DSM_GSENSOR_I2C_RW_ERROR_NO			(21200)
#define DSM_ALS_PS_I2C_RW_ERROR_NO			(21300)
#endif
#define DSM_HALL_ERROR_NO				(21500)
#define DSM_EXTISP_LOAD_FW_ERROR_NO			(20603)
#define DSM_FLASH_OPEN_SHOTR_ERROR_NO		(20607)
#define DSM_FLASH_HOT_DIE_ERROR_NO          (20608)
#define DSM_EXTISP_PQ_ERROR_NO              (20609)
#define DSM_EXTISP_I2C_ERROR_NO             (20610)
#define DSM_EXTISP_USPI_ERROR_NO            (20611)
#define DSM_EXTISP_VCM_ERROR_NO             (20612)
#define DSM_EXTISP_INTERRUPT_ERROR_NO       (20613)
#define DSM_EXTISP_COMMONFUNC_ERROR_NO      (20614)
#define DSM_EXTISP_A3_ERROR_NO              (20615)
#define DSM_EXTISP_PIPE_ERROR_NO            (20616)
#define DSM_EXTISP_RDN_ERROR_NO             (20617)
#define DSM_EXTISP_TXLM_ERROR_NO            (20618)
#define DSM_EXTISP_MTX_ERROR_NO             (20619)
#define DSM_EXTISP_MRX_ERROR_NO             (20620)
#define DSM_EXTISP_FEC0_ERROR_NO            (20621)
#define DSM_EXTISP_SENSOR_ERROR_NO          (20622)
#define DSM_EXTISP_FEC1_ERROR_NO            (20623)
#define DSM_EXTISP_CAP_ERROR_NO             (20624)
#define DSM_EXTISP_OTHER_ERROR_NO           (20625)
#define DSM_CSI_MIPI0_ERROR_NO              (20626)
#define DSM_CSI_MIPI1_ERROR_NO              (20627)

#define DSM_ISP22_MCU_NO_RSP_ERROR_NO       (20630)
#define DSM_ISP22_CMD_SET_ERROR_NO          (20631)
#define DSM_ISP22_SENSOR_PRIMARY_ERROR_NO   (20632)
#define DSM_ISP22_SENSOR_SECONDARY_ERROR_NO (20633)
#define DSM_ISP22_FLASH_ERROR_NO            (20634)

#define DSM_IVP_SMMU_ERROR_NO               (20680)
#define DSM_IVP_WATCH_ERROR_NO              (20681)
#define DSM_IVP_DWAXI_ERROR_NO              (20682)
#define DSM_IVP_OPEN_ERROR_NO               (20683)
#define DSM_IVP_MESSAGE_ERROR_NO            (20684)
#define DSM_IVP_PANIC_ERROR_NO              (20685)

/************DMD NUMBER FOR WIFI BEGIN************/
/*************** 20700 ~ 20799 *******************/
#define DSM_WIFI_MODULE_INIT_ERROR          (20700)
#define DSM_WIFI_RESERVED                   (20701)
#define DSM_WIFI_CMD52_ERROR                (20702)
#define DSM_WIFI_CMD53_ERROR                (20703)
/*wifi open*/
#define DSM_WIFI_SDIO_RESET_COMM_ERROR      (20704)
#define DSM_WIFI_SDIO_PROBE_ATTACH_ERROR    (20705)
#define DSM_WIFI_SDIO_FIRMWARE_DL_ERROR     (20706)
#define DSM_WIFI_DHD_DEV_INIT_IOCTL_ERROR   (20707)
/*wifi scan*/
#define DSM_WIFI_WLC_SET_PASSIVE_SCAN_ERROR (20708)
#define DSM_WIFI_WLC_SCAN_ERROR             (20709)
#define DSM_WIFI_WLC_SET_SCANSUPPRESS_ERROR (20710)
#define DSM_WIFI_WLC_GET_CHANNEL_ERROR      (20711)
#define DSM_WIFI_WLC_SCAN_RESULTS_ERROR     (20712)
/*wifi connect*/
#define DSM_WIFI_WLC_DISASSOC_ERROR         (20713)
#define DSM_WIFI_WLC_SET_SSID_ERROR         (20714)
#define DSM_WIFI_SET_CIPHER_ERROR           (20715)
#define DSM_WIFI_SET_KEY_MGMT_ERROR         (20716)
#define DSM_WIFI_SET_SHAREDKEY_ERROR        (20717)
#define DSM_WIFI_OPEN_ERROR                 (20718)
#define DSM_WIFI_KSO_ERROR                  (20719)
/*************** 20700 ~ 20799 *******************/
/************DMD NUMBER FOR WIFI END**************/

#define DSM_WIFI_ERR                                (30100)
#define DSM_WIFI_DRIVER_LOAD_ERR                    (DSM_WIFI_ERR + 0)
#define DSM_WIFI_SUPPLICANT_START_ERR               (DSM_WIFI_ERR + 1)
#define DSM_WIFI_CONNECT_SUPPLICANT_ERR             (DSM_WIFI_ERR + 2)
#define DSM_WIFI_MAC_ERR                            (DSM_WIFI_ERR + 3)
#define DSM_WIFI_SCAN_FAIL_ERR                      (DSM_WIFI_ERR + 4)
#define DSM_WIFI_FAIL_LOADFIRMWARE_ERR              (DSM_WIFI_ERR + 5)
#define DSM_WIFI_ABNORMAL_DISCONNECT_ERR            (DSM_WIFI_ERR + 6)
#define DSM_WIFI_CANNOT_CONNECT_AP_ERR              (DSM_WIFI_ERR + 7)
#define DSM_WIFI_ROOT_NOT_RIGHT_ERR                 (DSM_WIFI_ERR + 8)

#define DSM_BLUETOOTH_DM_ERROR                        (30200)
#define DSM_BLUETOOTH_DM_OPEN_ERROR                   (DSM_BLUETOOTH_DM_ERROR)
#define DSM_BLUETOOTH_DM_GET_ADDR_ERROR               (DSM_BLUETOOTH_DM_ERROR+1)
#define DSM_BLUETOOTH_A2DP_ERROR                      (30210)
#define DSM_BLUETOOTH_A2DP_CONNECT_ERROR              (DSM_BLUETOOTH_A2DP_ERROR)
#define DSM_BLUETOOTH_A2DP_AUDIO_ERROR                (DSM_BLUETOOTH_A2DP_ERROR+1)
#define DSM_BLUETOOTH_HFP_ERROR                       (30220)
#define DSM_BLUETOOTH_HFP_CONNECT_ERROR               (DSM_BLUETOOTH_HFP_ERROR)
#define DSM_BLUETOOTH_HFP_SCO_CONNECT_ERROR           (DSM_BLUETOOTH_HFP_ERROR+1)
#define DSM_BLUETOOTH_BLE_ERROR                       (30230)
#define DSM_BLUETOOTH_BLE_CONNECT_ERROR               (DSM_BLUETOOTH_BLE_ERROR)
#define DSM_BLUETOOTH_PROBE_ERROR					  (30215)
/*
#define DSM_BLUETOOTH_HANDLE_EXCEPTIONS_ERROR   	  	(30200)
#define DSM_BLUETOOTH_FIRMWARE_ERROR					(30210)
#define DSM_BLUETOOTH_FIRMWARE_DOWNLOAD_ERROR			(30211)
#define DSM_BLUETOOTH_UART_ERROR						(30220)
#define DSM_BLUETOOTH_HCI_DATA_ERROR					(30221)
#define DSM_BLUETOOTH_POWER_ON_ERROR					(30230)
#define DSM_BLUETOOTH_PROBE_ERROR						(30215)
*/

#define DSM_USB_ERROR_NO					(20800)

#define DSM_USB_ERROR_NO					(20800)

#define DSM_RECOVERY_ERROR_NO        (20900)

#define DSM_SELINUX_ERROR_NO                  (20901)

#define DSM_SPI_WRITE_ACK_ERROR_NO                   (21400)
#define DSM_SPI_READ_ACK_ERROR_NO                   (DSM_SPI_WRITE_ACK_ERROR_NO+1)
#define DSM_SPI_80_ACK_ERROR_NO                       (DSM_SPI_WRITE_ACK_ERROR_NO+2)
#define DSM_SPI_SYNC_ERROR_NO                           (DSM_SPI_WRITE_ACK_ERROR_NO + 3)
#define DSM_SPI_DOWN_ERROR_NO                          (DSM_SPI_WRITE_ACK_ERROR_NO + 4)
#define DSM_SPI_PORT_BUSY_NO                             (DSM_SPI_WRITE_ACK_ERROR_NO + 5)
#define DSM_SPI_DATA_READ_ERROR_NO                (DSM_SPI_WRITE_ACK_ERROR_NO + 6)
#define DSM_SPI_DATA_DROP_NO                            (DSM_SPI_WRITE_ACK_ERROR_NO + 7)
#define DSM_SPI_AUTO_READY_ERROR_NO              (DSM_SPI_WRITE_ACK_ERROR_NO + 8)
#define DSM_SPI_KZALLOC_FAILED_NO                    (DSM_SPI_WRITE_ACK_ERROR_NO + 9)
#define DSM_SPI_ALLOC_SKB_FAILED_NO                (DSM_SPI_WRITE_ACK_ERROR_NO + 10)
#define DSM_SPI_DOWN_RETRY_MAX_NO                 (DSM_SPI_WRITE_ACK_ERROR_NO + 11)
#define DSM_SPI_SYNC_ERR_MAX_NO                      (DSM_SPI_WRITE_ACK_ERROR_NO + 12)
#define DSM_APCP_SDIO_WRITE_ACK_ERROR_NO    (DSM_SPI_WRITE_ACK_ERROR_NO + 13)
#define DSM_SPI_ERROR_NO					(21490) /* spi_sync timeout error */
#define DSM_SPI_INVALID_ARGUMENT_ERR_NO (21491)
#define DSM_SPI_SHUTDOWN_ERR_NO               (21492)
#define DSM_SPI_UNKOWN_ERR_NO                   (21493)
#define DSM_SPI_SG_ALLOC_RX_PAGE_INVILAD_LEN (21494)
#define DSM_SPI_SG_ALLOC_TX_PAGE_INVILAD_LEN (21495)
#define DSM_SPI_RX_FIFO_NOT_EMPTY            (21496)
#define DSM_SPI_SPI_IS_BUSY                  (21497)

#ifdef CONFIG_HUAWEI_FS_DSM
#define DSM_FS_EXT4_ERROR					(22000)
#define DSM_FS_EXT4_ERROR_INODE				(22001)
#define DSM_FS_EXT4_ERROR_FILE				(22002)
#define DSM_FS_EXT4_ERROR_READ_SUPER		(22003)
#define DSM_FS_EXT4_ERROR_READ_SUPER_SECOND (22004)
#define DSM_FS_EXT4_ERROR_WRITE_SUPER		(22005)
#endif

#ifdef CONFIG_HUAWEI_F2FS_DSM
#define DSM_F2FS_ERROR_MSG                      (22500)
#define DSM_F2FS_SUPER_NEED_FSCK                (22501)
#define DSM_F2FS_BUGCHK_NEED_FSCK               (22502)
#endif

#define DSM_UART_OPEN_HWINIT_ERROR              (22100)
#define DSM_UART_OPEN_IRQ_REQUSET_FAILED        (DSM_UART_OPEN_HWINIT_ERROR+1)
#define DSM_UART_TX_DMA_MAP_FAILED              (DSM_UART_OPEN_HWINIT_ERROR+2)
#define DSM_UART_TX_DMA_BUSY_ERROR              (DSM_UART_OPEN_HWINIT_ERROR+3)
#define DSM_UART_RX_DMA_READ_BUFFER_FULL_ERROR  (DSM_UART_OPEN_HWINIT_ERROR+4)
#define DSM_UART_CONTROLLER_FIFO_OVERRUN_ERROR  (DSM_UART_OPEN_HWINIT_ERROR+5)
#define DSM_UART_CONTROLLER_LINE_BREAK_ERROR    (DSM_UART_OPEN_HWINIT_ERROR+6)
#define DSM_UART_CONTROLLER_PARITY_ERROR        (DSM_UART_OPEN_HWINIT_ERROR+7)
#define DSM_UART_CONTROLLER_FRAME_ERROR         (DSM_UART_OPEN_HWINIT_ERROR+8)

#define DSM_LEDS_ERROR_NO							(22400)

#define DSM_HWTC_ERROR_NO                       (22500)

#define DSM_MM_STAT                             (22800)


/************DMD NUMBER FOR NFC BEGIN**************/
#define DSM_NFC_I2C_WRITE_ERROR_NO					(30300)
#define DSM_NFC_I2C_READ_ERROR_NO					(30301)
#define DSM_NFC_CLK_ENABLE_ERROR_NO					(30302)
#define DSM_NFC_I2C_WRITE_EOPNOTSUPP_ERROR_NO		(30303)
#define DSM_NFC_I2C_READ_EOPNOTSUPP_ERROR_NO		(30304)
#define DSM_NFC_I2C_WRITE_EREMOTEIO_ERROR_NO		(30305)
#define DSM_NFC_I2C_READ_EREMOTEIO_ERROR_NO			(30306)
#define DSM_NFC_RD_I2C_WRITE_ERROR_NO				(30307)
#define DSM_NFC_RD_I2C_READ_ERROR_NO				(30308)
#define DSM_NFC_RD_I2C_WRITE_EOPNOTSUPP_ERROR_NO	(30309)
#define DSM_NFC_RD_I2C_READ_EOPNOTSUPP_ERROR_NO		(30310)
#define DSM_NFC_RD_I2C_WRITE_EREMOTEIO_ERROR_NO		(30311)
#define DSM_NFC_RD_I2C_READ_EREMOTEIO_ERROR_NO		(30312)
#define DSM_NFC_SIM_CHECK_ERROR_NO					(30313)
/************DMD NUMBER FOR NFC END**************/

#define DSM_FINGERPRINT_WAIT_FOR_FINGER_ERROR_NO	(21000)
#define DSM_FINGERPRINT_CAPTURE_IMAGE_ERROR_NO	(21001)
#define DSM_FINGERPRINT_IDENTIFY_ERROR_NO                 (21002)
#define DSM_FINGERPRINT_TEST_DEADPIXELS_ERROR_NO	(21003)
#define DSM_FINGERPRINT_ENROLL_ERROR_NO	             (21004)
#define DSM_FINGERPRINT_REMOVE_TEMPLATE_ERROR_NO	(21005)
#define DSM_FINGERPRINT_ENUMERATE_ERROR_NO		(21006)
#define DSM_FINGERPRINT_MODULE_OPEN_ERROR_NO		(21007)
#define DSM_FINGERPRINT_PROBE_FAIL_ERROR_NO	       (21008)
#define DSM_FINGERPRINT_DIFF_DEADPIXELS_ERROR_NO	(21009)
#define DSM_FINGERPRINT_MANY_DEADPIXELS_ERROR_NO	(21010)
#define DSM_FINGERPRINT_DB_FILE_LOST_ERROR_NO	    (21011)

#define DSM_DM_VERITY_ERROR_NO	(22700)


struct dsm_client_ops{
	int (*poll_state) (void);
	int (*dump_func) (int type, void *buff, int size);
};

struct dsm_dev{
	const char *name;
	const char *device_name;
	const char *ic_name;
	const char *module_name;
	struct dsm_client_ops *fops;
	size_t buff_size;
};

struct dsm_client{
	char *client_name;
	char *device_name;
	char *ic_name;
	char *module_name;
	int client_id;
	int error_no;
	unsigned long buff_flag;
	struct dsm_client_ops *cops;
	wait_queue_head_t waitq;
	size_t read_size;
	size_t used_size;
	size_t buff_size;
	u8 dump_buff[];
};

/*
* for userspace client, such as sensor service, please refer to it.
*/
struct dsm_extern_client{
	char client_name[CLIENT_NAME_LEN];
	int buf_size;
};

#ifdef CONFIG_HUAWEI_DSM
struct dsm_client *dsm_register_client (struct dsm_dev *dev);
struct dsm_client *dsm_find_client(char *dsm_name);
int dsm_client_ocuppy(struct dsm_client *client);
int dsm_client_unocuppy(struct dsm_client *client);
int dsm_client_record(struct dsm_client *client, const char *fmt, ...);
int dsm_client_copy(struct dsm_client *client, void *src, int sz);
void dsm_client_notify(struct dsm_client *client, int error_no);
#else
static inline struct dsm_client *dsm_register_client (struct dsm_dev *dev)
{
	return NULL;
}
static inline struct dsm_client *dsm_find_client(char *dsm_name)
{
	return NULL;
}
static inline int dsm_client_ocuppy(struct dsm_client *client)
{
	return 1;
}
static inline int dsm_client_unocuppy(struct dsm_client *client)
{
	return 0;
}

static inline int dsm_client_record(struct dsm_client *client, const char *fmt, ...)
{
	return 0;
}
static inline int dsm_client_copy(struct dsm_client *client, void *src, int sz)
{
	return 0;
}
static inline void dsm_client_notify(struct dsm_client *client, int error_no)
{
	return;
}
#endif

#endif
