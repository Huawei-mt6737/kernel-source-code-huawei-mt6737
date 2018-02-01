/*
** Id: //Department/DaVinci/BRANCHES/MT6620_WIFI_DRIVER_V2_3/include/wlan_lib.h#1
*/

/*! \file   "wlan_lib.h"
    \brief  The declaration of the functions of the wlanAdpater objects

    Detail description.
*/



#ifndef _WLAN_LIB_H
#define _WLAN_LIB_H

/*******************************************************************************
*                         C O M P I L E R   F L A G S
********************************************************************************
*/

/*******************************************************************************
*                    E X T E R N A L   R E F E R E N C E S
********************************************************************************
*/
#include "CFG_Wifi_File.h"
#include "rlm_domain.h"
#include "wlan_typedef.h"


extern BOOLEAN fgIsResetting;

/*******************************************************************************
*                              C O N S T A N T S
********************************************************************************
*/

#define MAX_NUM_GROUP_ADDR                      32	/* max number of group addresses */

#define TX_CS_TCP_UDP_GEN        BIT(1)
#define TX_CS_IP_GEN             BIT(0)

#define CSUM_OFFLOAD_EN_TX_TCP      BIT(0)
#define CSUM_OFFLOAD_EN_TX_UDP      BIT(1)
#define CSUM_OFFLOAD_EN_TX_IP       BIT(2)
#define CSUM_OFFLOAD_EN_RX_TCP      BIT(3)
#define CSUM_OFFLOAD_EN_RX_UDP      BIT(4)
#define CSUM_OFFLOAD_EN_RX_IPv4     BIT(5)
#define CSUM_OFFLOAD_EN_RX_IPv6     BIT(6)
#define CSUM_OFFLOAD_EN_TX_MASK     BITS(0, 2)
#define CSUM_OFFLOAD_EN_ALL         BITS(0, 6)

/* TCP, UDP, IP Checksum */
#define RX_CS_TYPE_UDP           BIT(7)
#define RX_CS_TYPE_TCP           BIT(6)
#define RX_CS_TYPE_IPv6          BIT(5)
#define RX_CS_TYPE_IPv4          BIT(4)

#define RX_CS_STATUS_UDP         BIT(3)
#define RX_CS_STATUS_TCP         BIT(2)
#define RX_CS_STATUS_IP          BIT(0)

#define CSUM_NOT_SUPPORTED      0x0

#define TXPWR_USE_PDSLOPE 0

/* NVRAM error code definitions */
#define NVRAM_ERROR_VERSION_MISMATCH        BIT(1)
#define NVRAM_ERROR_INVALID_TXPWR           BIT(2)
#define NVRAM_ERROR_INVALID_DPD             BIT(3)
#define NVRAM_ERROR_INVALID_MAC_ADDR        BIT(4)
#if CFG_SUPPORT_PWR_LIMIT_COUNTRY
#define NVRAM_POWER_LIMIT_TABLE_INVALID     BIT(5)
#endif

#define NUM_TC_RESOURCE_TO_STATISTICS       4

#define WLAN_CFG_ARGV_MAX 8
#define WLAN_CFG_ENTRY_NUM_MAX 128
#define WLAN_CFG_KEY_LEN_MAX 32	/* include \x00  EOL */
#define WLAN_CFG_VALUE_LEN_MAX 32	/* include \x00 EOL */
#define WLAN_CFG_FLAG_SKIP_CB BIT(0)
#define WLAN_CFG_FILE_BUF_SIZE 2048

#define WLAN_CFG_SET_CHIP_LEN_MAX 10
#define WLAN_CFG_SET_DEBUG_LEVEL_LEN_MAX 10
#define WLAN_CFG_SET_SW_CTRL_LEN_MAX 10

#define WLAN_OID_TIMEOUT_THRESHOLD                  2000	/* OID timeout (in ms) */
#define WLAN_OID_TIMEOUT_THRESHOLD_IN_RESETTING      300	/* OID timeout during chip-resetting  (in ms) */

#define WLAN_OID_NO_ACK_THRESHOLD                   3

#define WLAN_TX_THREAD_TASK_PRIORITY        0	/* If not setting the priority, 0 is the default */
#define WLAN_TX_THREAD_TASK_NICE            (-10)	/* If not setting the nice, -10 is the default */

/*******************************************************************************
*                             D A T A   T Y P E S
********************************************************************************
*/
typedef WLAN_STATUS(*PFN_OID_HANDLER_FUNC) (IN P_ADAPTER_T prAdapter,
					    IN PVOID pvBuf, IN UINT_32 u4BufLen, OUT PUINT_32 pu4OutInfoLen);

typedef enum _ENUM_CSUM_TYPE_T {
	CSUM_TYPE_IPV4,
	CSUM_TYPE_IPV6,
	CSUM_TYPE_TCP,
	CSUM_TYPE_UDP,
	CSUM_TYPE_NUM
} ENUM_CSUM_TYPE_T, *P_ENUM_CSUM_TYPE_T;

typedef enum _ENUM_CSUM_RESULT_T {
	CSUM_RES_NONE,
	CSUM_RES_SUCCESS,
	CSUM_RES_FAILED,
	CSUM_RES_NUM
} ENUM_CSUM_RESULT_T, *P_ENUM_CSUM_RESULT_T;

typedef enum _ENUM_PHY_MODE_T {
	ENUM_PHY_2G4_CCK,
	ENUM_PHY_2G4_OFDM_BPSK,
	ENUM_PHY_2G4_OFDM_QPSK,
	ENUM_PHY_2G4_OFDM_16QAM,
	ENUM_PHY_2G4_OFDM_48M,
	ENUM_PHY_2G4_OFDM_54M,
	ENUM_PHY_2G4_HT20_BPSK,
	ENUM_PHY_2G4_HT20_QPSK,
	ENUM_PHY_2G4_HT20_16QAM,
	ENUM_PHY_2G4_HT20_MCS5,
	ENUM_PHY_2G4_HT20_MCS6,
	ENUM_PHY_2G4_HT20_MCS7,
	ENUM_PHY_2G4_HT40_BPSK,
	ENUM_PHY_2G4_HT40_QPSK,
	ENUM_PHY_2G4_HT40_16QAM,
	ENUM_PHY_2G4_HT40_MCS5,
	ENUM_PHY_2G4_HT40_MCS6,
	ENUM_PHY_2G4_HT40_MCS7,
	ENUM_PHY_5G_OFDM_BPSK,
	ENUM_PHY_5G_OFDM_QPSK,
	ENUM_PHY_5G_OFDM_16QAM,
	ENUM_PHY_5G_OFDM_48M,
	ENUM_PHY_5G_OFDM_54M,
	ENUM_PHY_5G_HT20_BPSK,
	ENUM_PHY_5G_HT20_QPSK,
	ENUM_PHY_5G_HT20_16QAM,
	ENUM_PHY_5G_HT20_MCS5,
	ENUM_PHY_5G_HT20_MCS6,
	ENUM_PHY_5G_HT20_MCS7,
	ENUM_PHY_5G_HT40_BPSK,
	ENUM_PHY_5G_HT40_QPSK,
	ENUM_PHY_5G_HT40_16QAM,
	ENUM_PHY_5G_HT40_MCS5,
	ENUM_PHY_5G_HT40_MCS6,
	ENUM_PHY_5G_HT40_MCS7,
	ENUM_PHY_MODE_NUM
} ENUM_PHY_MODE_T, *P_ENUM_PHY_MODE_T;

typedef enum _ENUM_POWER_SAVE_POLL_MODE_T {
	ENUM_POWER_SAVE_POLL_DISABLE,
	ENUM_POWER_SAVE_POLL_LEGACY_NULL,
	ENUM_POWER_SAVE_POLL_QOS_NULL,
	ENUM_POWER_SAVE_POLL_NUM
} ENUM_POWER_SAVE_POLL_MODE_T, *P_ENUM_POWER_SAVE_POLL_MODE_T;

typedef enum _ENUM_AC_TYPE_T {
	ENUM_AC_TYPE_AC0,
	ENUM_AC_TYPE_AC1,
	ENUM_AC_TYPE_AC2,
	ENUM_AC_TYPE_AC3,
	ENUM_AC_TYPE_AC4,
	ENUM_AC_TYPE_AC5,
	ENUM_AC_TYPE_AC6,
	ENUM_AC_TYPE_BMC,
	ENUM_AC_TYPE_NUM
} ENUM_AC_TYPE_T, *P_ENUM_AC_TYPE_T;

typedef enum _ENUM_ADV_AC_TYPE_T {
	ENUM_ADV_AC_TYPE_RX_NSW,
	ENUM_ADV_AC_TYPE_RX_PTA,
	ENUM_ADV_AC_TYPE_RX_SP,
	ENUM_ADV_AC_TYPE_TX_PTA,
	ENUM_ADV_AC_TYPE_TX_RSP,
	ENUM_ADV_AC_TYPE_NUM
} ENUM_ADV_AC_TYPE_T, *P_ENUM_ADV_AC_TYPE_T;

typedef enum _ENUM_REG_CH_MAP_T {
	REG_CH_MAP_COUNTRY_CODE,
	REG_CH_MAP_TBL_IDX,
	REG_CH_MAP_CUSTOMIZED,
	REG_CH_MAP_NUM
} ENUM_REG_CH_MAP_T, *P_ENUM_REG_CH_MAP_T;

#define CHIP_CONFIG_RESP_SIZE 320
enum {
	CHIP_CONFIG_TYPE_WO_RESPONSE = 0x00,
	CHIP_CONFIG_TYPE_MEM8 = 0x01,
	CHIP_CONFIG_TYPE_MEM32 = 0x02,
	CHIP_CONFIG_TYPE_ASCII = 0x03,
	CHIP_CONFIG_TYPE_BINARY = 0x04,
	CHIP_CONFIG_TYPE_DRV_PASSTHROUGH = 0x05,
	CHIP_CONFIG_TYPE_END
};

typedef struct _SET_TXPWR_CTRL_T {
	INT_8 c2GLegacyStaPwrOffset;	/* Unit: 0.5dBm, default: 0 */
	INT_8 c2GHotspotPwrOffset;
	INT_8 c2GP2pPwrOffset;
	INT_8 c2GBowPwrOffset;
	INT_8 c5GLegacyStaPwrOffset;	/* Unit: 0.5dBm, default: 0 */
	INT_8 c5GHotspotPwrOffset;
	INT_8 c5GP2pPwrOffset;
	INT_8 c5GBowPwrOffset;
	UINT_8 ucConcurrencePolicy;	/* TX power policy when concurrence
					   in the same channel
					   0: Highest power has priority
					   1: Lowest power has priority */
	INT_8 acReserved1[3];	/* Must be zero */

	/* Power limit by channel for all data rates */
	INT_8 acTxPwrLimit2G[14];	/* Channel 1~14, Unit: 0.5dBm */
	INT_8 acTxPwrLimit5G[4];	/* UNII 1~4 */
	INT_8 acReserved2[2];	/* Must be zero */
} SET_TXPWR_CTRL_T, *P_SET_TXPWR_CTRL_T;

/* For storing driver initialization value from glue layer */
typedef struct _REG_INFO_T {
	UINT_32 u4SdBlockSize;	/* SDIO block size */
	UINT_32 u4SdBusWidth;	/* SDIO bus width. 1 or 4 */
	UINT_32 u4SdClockRate;	/* SDIO clock rate. (in unit of HZ) */
	UINT_32 u4StartAddress;	/* Starting address of Wi-Fi Firmware */
	UINT_32 u4LoadAddress;	/* Load address of Wi-Fi Firmware */
	UINT_16 aucFwImgFilename[65];	/* Firmware filename */
	UINT_16 aucFwImgFilenameE6[65];	/* Firmware filename for E6 */
	UINT_32 u4StartFreq;	/* Start Frequency for Ad-Hoc network : in unit of KHz */
	UINT_32 u4AdhocMode;	/* Default mode for Ad-Hoc network : ENUM_PARAM_AD_HOC_MODE_T */
	UINT_32 u4RddStartFreq;
	UINT_32 u4RddStopFreq;
	UINT_32 u4RddTestMode;
	UINT_32 u4RddShutFreq;
	UINT_32 u4RddDfs;
	INT_32 i4HighRssiThreshold;
	INT_32 i4MediumRssiThreshold;
	INT_32 i4LowRssiThreshold;
	INT_32 au4TxPriorityTag[ENUM_AC_TYPE_NUM];
	INT_32 au4RxPriorityTag[ENUM_AC_TYPE_NUM];
	INT_32 au4AdvPriorityTag[ENUM_ADV_AC_TYPE_NUM];
	UINT_32 u4FastPSPoll;
	UINT_32 u4PTA;		/* 0: disable, 1: enable */
	UINT_32 u4TXLimit;	/* 0: disable, 1: enable */
	UINT_32 u4SilenceWindow;	/* range: 100 - 625, unit: us */
	UINT_32 u4TXLimitThreshold;	/* range: 250 - 1250, unit: us */
	UINT_32 u4PowerMode;
	UINT_32 fgEnArpFilter;
	UINT_32 u4PsCurrentMeasureEn;
	UINT_32 u4UapsdAcBmp;
	UINT_32 u4MaxSpLen;
	UINT_32 fgDisOnlineScan;	/* 0: enable online scan, non-zero: disable online scan */
	UINT_32 fgDisBcnLostDetection;	/* 0: enable online scan, non-zero: disable online scan */
	UINT_32 u4FixedRate;	/* 0: automatic, non-zero: fixed rate */
	UINT_32 u4ArSysParam0;
	UINT_32 u4ArSysParam1;
	UINT_32 u4ArSysParam2;
	UINT_32 u4ArSysParam3;
	UINT_32 fgDisRoaming;	/* 0:enable roaming 1:disable */

	/* NVRAM - MP Data -START- */
	UINT_8 aucMacAddr[6];
	UINT_16 au2CountryCode[4];	/* Country code (in ISO 3166-1 expression, ex: "US", "TW")  */
	TX_PWR_PARAM_T rTxPwr;
	UINT_8 aucEFUSE[144];
	UINT_8 ucTxPwrValid;
	UINT_8 ucSupport5GBand;
	UINT_8 fg2G4BandEdgePwrUsed;
	INT_8 cBandEdgeMaxPwrCCK;
	INT_8 cBandEdgeMaxPwrOFDM20;
	INT_8 cBandEdgeMaxPwrOFDM40;
	ENUM_REG_CH_MAP_T eRegChannelListMap;
	UINT_8 ucRegChannelListIndex;
	DOMAIN_INFO_ENTRY rDomainInfo;
	/* NVRAM - MP Data -END- */

	/* NVRAM - Functional Data -START- */
	UINT_8 uc2G4BwFixed20M;
	UINT_8 uc5GBwFixed20M;
	UINT_8 ucEnable5GBand;
	UINT_8 uc2GRssiCompensation;
	UINT_8 uc5GRssiCompensation;
	UINT_8 fgRssiCompensationValidbit;
	UINT_8 ucRxAntennanumber;
	/* NVRAM - Functional Data -END- */

} REG_INFO_T, *P_REG_INFO_T;

/* for divided firmware loading */
typedef struct _FWDL_SECTION_INFO_T {
	UINT_32 u4Offset;
	UINT_32 u4Reserved;
	UINT_32 u4Length;
	UINT_32 u4DestAddr;
} FWDL_SECTION_INFO_T, *P_FWDL_SECTION_INFO_T;

typedef struct _FIRMWARE_DIVIDED_DOWNLOAD_T {
	UINT_32 u4Signature;
	UINT_32 u4CRC;		/* CRC calculated without first 8 bytes included */
	UINT_32 u4NumOfEntries;
	UINT_32 u4Reserved;
	FWDL_SECTION_INFO_T arSection[];
} FIRMWARE_DIVIDED_DOWNLOAD_T, *P_FIRMWARE_DIVIDED_DOWNLOAD_T;

typedef struct _PARAM_MCR_RW_STRUCT_T {
	UINT_32 u4McrOffset;
	UINT_32 u4McrData;
} PARAM_MCR_RW_STRUCT_T, *P_PARAM_MCR_RW_STRUCT_T;

typedef struct _PARAM_GET_STA_STATISTICS {
	/* Per-STA statistic */
	UINT_8 aucMacAddr[MAC_ADDR_LEN];

	UINT_32 u4Flag;

	/* From driver */
	UINT_32 u4TxTotalCount;
	UINT_32 u4TxExceedThresholdCount;

	UINT_32 u4TxMaxTime;
	UINT_32 u4TxAverageProcessTime;

	UINT_32 au4TcResourceEmptyCount[NUM_TC_RESOURCE_TO_STATISTICS];
	UINT_32 au4TcQueLen[NUM_TC_RESOURCE_TO_STATISTICS];

	/* From FW */
	UINT_8 ucPer;		/* base: 128 */
	UINT_8 ucRcpi;
	UINT_32 u4PhyMode;
	UINT_16 u2LinkSpeed;	/* unit is 0.5 Mbits */

	UINT_32 u4TxFailCount;
	UINT_32 u4TxLifeTimeoutCount;

	UINT_32 u4TxAverageAirTime;

	/* Global queue management statistic */
	UINT_32 au4TcAverageQueLen[NUM_TC_RESOURCE_TO_STATISTICS];
	UINT_32 au4TcCurrentQueLen[NUM_TC_RESOURCE_TO_STATISTICS];

	/* Reserved fields */
	UINT_8 au4Reserved[32];
} PARAM_GET_STA_STA_STATISTICS, *P_PARAM_GET_STA_STATISTICS;

typedef enum _ENUM_BAND_EDGE_CERT_T {
	BAND_EDGE_CERT_FCC = 0,
	BAND_EDGE_CERT_KCC = 1,
	BAND_EDGE_CERT_NUM = 2
} ENUM_BAND_EDGE_CERT_T, *P_ENUM_BAND_EDGE_CERT_T;

#if CFG_AUTO_CHANNEL_SEL_SUPPORT
typedef enum _ENUM_TESTMODE_AVAILABLE_CHAN_ATTR {
	NL80211_TESTMODE_AVAILABLE_CHAN_INVALID = 0,
	NL80211_TESTMODE_AVAILABLE_CHAN_2G_BASE_1,
	NL80211_TESTMODE_AVAILABLE_CHAN_5G_BASE_34,
	NL80211_TESTMODE_AVAILABLE_CHAN_5G_BASE_149,
	NL80211_TESTMODE_AVAILABLE_CHAN_5G_BASE_184,

	NL80211_TESTMODE_AVAILABLE_CHAN_NUM,
} ENUM_TESTMODE_AVAILABLE_CHAN_ATTR;

typedef struct _LTE_SAFE_CH_INFO_T {
	UINT_32 au4SafeChannelBitmask[NL80211_TESTMODE_AVAILABLE_CHAN_NUM - 1];
} LTE_SAFE_CH_INFO_T, *P_CMD_LTE_SAFE_CH_INFO_T;

    /* Record Each CH Load */
typedef struct _PARAM_CHN_LOAD_INFO {
	/* Per-CHN Load */
	UINT_32 u4Flag;

	UINT_8 ucChannel;
	UINT_16 u2ChannelLoad;
	UINT_8 au4Reserved[1];

	UINT_16 u2APNum;
	UINT_16 u2APNumTmpCountingBuf;

	/* Reserved fields */
	UINT_8 au4Reserved1[8];
} PARAM_CHN_LOAD_INFO, *P_PARAM_CHN_LOAD_INFO;

typedef struct _PARAM_GET_CHN_LOAD {
	LTE_SAFE_CH_INFO_T rLteSafeChnList;
	PARAM_CHN_LOAD_INFO rEachChnLoad[MAX_AUTO_CHAL_NUM];
	BOOLEAN fgDataReadyBit;
	UINT_8 au4Reserved1[3];
} PARAM_GET_CHN_LOAD, *P_PARAM_GET_CHN_LOAD;

typedef struct _PARAM_PREFER_CHN_INFO {

	UINT_8 ucChannel;
	UINT_16 u2APNum;
	UINT_8 au4Reserved1[1];
} PARAM_PREFER_CHN_INFO, *P_PARAM_PREFER_CHN_INFO;

typedef struct _PARAM_GET_LTE_MODE {
	/* Event Body */
	UINT_8 ucVersion;
	UINT_8 aucReserved1[3];
	UINT_32 u4Flags;	/* Bit0: valid */

	LTE_SAFE_CH_INFO_T LTE_MODE;
	UINT_8 aucReserved4[3];

	UINT_8 aucReserved[4];

} PARAM_GET_LTE_MODE, *P_PARAM_GET_LTE_MODE;

#endif

/*******************************************************************************
*                            P U B L I C   D A T A
********************************************************************************
*/

/*******************************************************************************
*                           P R I V A T E   D A T A
********************************************************************************
*/

/*******************************************************************************
*                                 M A C R O S
********************************************************************************
*/
#define BUILD_SIGN(ch0, ch1, ch2, ch3) \
	((UINT_32)(UINT_8)(ch0) | ((UINT_32)(UINT_8)(ch1) << 8) |   \
	((UINT_32)(UINT_8)(ch2) << 16) | ((UINT_32)(UINT_8)(ch3) << 24))

#define MTK_WIFI_SIGNATURE BUILD_SIGN('M', 'T', 'K', 'W')

/*******************************************************************************
*                   F U N C T I O N   D E C L A R A T I O N S
********************************************************************************
*/

P_ADAPTER_T wlanAdapterCreate(IN P_GLUE_INFO_T prGlueInfo);

VOID wlanAdapterDestroy(IN P_ADAPTER_T prAdapter);

VOID wlanCardEjected(IN P_ADAPTER_T prAdapter);

VOID wlanIST(IN P_ADAPTER_T prAdapter);

BOOLEAN wlanISR(IN P_ADAPTER_T prAdapter, IN BOOLEAN fgGlobalIntrCtrl);

WLAN_STATUS wlanProcessCommandQueue(IN P_ADAPTER_T prAdapter, IN P_QUE_T prCmdQue);

WLAN_STATUS wlanSendCommand(IN P_ADAPTER_T prAdapter, IN P_CMD_INFO_T prCmdInfo);

VOID wlanReleaseCommand(IN P_ADAPTER_T prAdapter, IN P_CMD_INFO_T prCmdInfo);

VOID wlanReleasePendingOid(IN P_ADAPTER_T prAdapter, IN ULONG ulData);

VOID wlanReleasePendingCMDbyNetwork(IN P_ADAPTER_T prAdapter, IN ENUM_NETWORK_TYPE_INDEX_T eNetworkType);

VOID wlanReturnPacket(IN P_ADAPTER_T prAdapter, IN PVOID pvPacket);

VOID wlanReturnIndicatedPacketsTimeOut(IN P_ADAPTER_T prAdapter, IN ULONG ulData);

WLAN_STATUS
wlanQueryInformation(IN P_ADAPTER_T prAdapter,
		     IN PFN_OID_HANDLER_FUNC pfOidQryHandler,
		     IN PVOID pvInfoBuf, IN UINT_32 u4InfoBufLen, OUT PUINT_32 pu4QryInfoLen);

WLAN_STATUS
wlanSetInformation(IN P_ADAPTER_T prAdapter,
		   IN PFN_OID_HANDLER_FUNC pfOidSetHandler,
		   IN PVOID pvInfoBuf, IN UINT_32 u4InfoBufLen, OUT PUINT_32 pu4SetInfoLen);

WLAN_STATUS
wlanAdapterStart(IN P_ADAPTER_T prAdapter,
		 IN P_REG_INFO_T prRegInfo, IN PVOID pvFwImageMapFile, IN UINT_32 u4FwImageFileLength);

WLAN_STATUS wlanAdapterStop(IN P_ADAPTER_T prAdapter);

#if CFG_SUPPORT_WAPI
BOOLEAN wlanQueryWapiMode(IN P_ADAPTER_T prAdapter);
#endif

VOID wlanReturnRxPacket(IN PVOID pvAdapter, IN PVOID pvPacket);

VOID wlanRxSetBroadcast(IN P_ADAPTER_T prAdapter, IN BOOLEAN fgEnableBroadcast);

BOOLEAN wlanIsHandlerNeedHwAccess(IN PFN_OID_HANDLER_FUNC pfnOidHandler, IN BOOLEAN fgSetInfo);

VOID wlanSetPromiscuousMode(IN P_ADAPTER_T prAdapter, IN BOOLEAN fgEnablePromiscuousMode);

#if CFG_ENABLE_FW_DOWNLOAD
#if CFG_ENABLE_FW_DOWNLOAD_AGGREGATION
WLAN_STATUS
wlanImageSectionDownloadAggregated(IN P_ADAPTER_T prAdapter,
				   IN UINT_32 u4DestAddr, IN UINT_32 u4ImgSecSize, IN PUINT_8 pucImgSecBuf);
#endif

WLAN_STATUS
wlanImageSectionDownload(IN P_ADAPTER_T prAdapter,
			 IN UINT_32 u4DestAddr, IN UINT_32 u4ImgSecSize, IN PUINT_8 pucImgSecBuf);

#if !CFG_ENABLE_FW_DOWNLOAD_ACK
WLAN_STATUS wlanImageQueryStatus(IN P_ADAPTER_T prAdapter);
#else
WLAN_STATUS wlanImageSectionDownloadStatus(IN P_ADAPTER_T prAdapter, IN UINT_8 ucCmdSeqNum);
#endif

WLAN_STATUS wlanConfigWifiFunc(IN P_ADAPTER_T prAdapter, IN BOOLEAN fgEnable, IN UINT_32 u4StartAddress);

UINT_32 wlanCRC32(PUINT_8 buf, UINT_32 len);

#endif

WLAN_STATUS wlanSendNicPowerCtrlCmd(IN P_ADAPTER_T prAdapter, IN UINT_8 ucPowerMode);

BOOLEAN wlanIsHandlerAllowedInRFTest(IN PFN_OID_HANDLER_FUNC pfnOidHandler, IN BOOLEAN fgSetInfo);

WLAN_STATUS wlanProcessQueuedSwRfb(IN P_ADAPTER_T prAdapter, IN P_SW_RFB_T prSwRfbListHead);

WLAN_STATUS wlanProcessQueuedMsduInfo(IN P_ADAPTER_T prAdapter, IN P_MSDU_INFO_T prMsduInfoListHead);

BOOLEAN wlanoidTimeoutCheck(IN P_ADAPTER_T prAdapter, IN PFN_OID_HANDLER_FUNC pfnOidHandler);

VOID wlanoidClearTimeoutCheck(IN P_ADAPTER_T prAdapter);

WLAN_STATUS wlanUpdateNetworkAddress(IN P_ADAPTER_T prAdapter);

BOOLEAN wlanQueryTestMode(IN P_ADAPTER_T prAdapter);

/* Security Frame Handling */
BOOLEAN wlanProcessSecurityFrame(IN P_ADAPTER_T prAdapter, IN P_NATIVE_PACKET prPacket);

VOID wlanSecurityFrameTxDone(IN P_ADAPTER_T prAdapter, IN P_CMD_INFO_T prCmdInfo, IN PUINT_8 pucEventBuf);

VOID wlanSecurityFrameTxTimeout(IN P_ADAPTER_T prAdapter, IN P_CMD_INFO_T prCmdInfo);

/*----------------------------------------------------------------------------*/
/* OID/IOCTL Handling                                                         */
/*----------------------------------------------------------------------------*/
VOID wlanClearScanningResult(IN P_ADAPTER_T prAdapter);

VOID wlanClearBssInScanningResult(IN P_ADAPTER_T prAdapter, IN PUINT_8 arBSSID);

#if CFG_TEST_WIFI_DIRECT_GO
VOID wlanEnableP2pFunction(IN P_ADAPTER_T prAdapter);

VOID wlanEnableATGO(IN P_ADAPTER_T prAdapter);
#endif

/*----------------------------------------------------------------------------*/
/* Address Retrieve by Polling                                                */
/*----------------------------------------------------------------------------*/
WLAN_STATUS wlanQueryPermanentAddress(IN P_ADAPTER_T prAdapter);

/*----------------------------------------------------------------------------*/
/* NIC Capability Retrieve by Polling                                         */
/*----------------------------------------------------------------------------*/
WLAN_STATUS wlanQueryNicCapability(IN P_ADAPTER_T prAdapter);

/*----------------------------------------------------------------------------*/
/* NIC Capability Retrieve by Polling                                         */
/*----------------------------------------------------------------------------*/
WLAN_STATUS wlanQueryDebugCode(IN P_ADAPTER_T prAdapter);

/*----------------------------------------------------------------------------*/
/* Compiler Flags Retrieve by Polling                                         */
/*----------------------------------------------------------------------------*/
WLAN_STATUS wlanQueryCompileFlags(IN P_ADAPTER_T prAdapter);

/*----------------------------------------------------------------------------*/
/* PD MCR Retrieve by Polling                                         */
/*----------------------------------------------------------------------------*/
WLAN_STATUS wlanQueryPdMcr(IN P_ADAPTER_T prAdapter, IN P_PARAM_MCR_RW_STRUCT_T prMcrRdInfo);
/*----------------------------------------------------------------------------*/
/* Loading Manufacture Data                                                   */
/*----------------------------------------------------------------------------*/
WLAN_STATUS wlanLoadManufactureData(IN P_ADAPTER_T prAdapter, IN P_REG_INFO_T prRegInfo);

/*----------------------------------------------------------------------------*/
/* Media Stream Mode                                                          */
/*----------------------------------------------------------------------------*/
BOOLEAN wlanResetMediaStreamMode(IN P_ADAPTER_T prAdapter);

/*----------------------------------------------------------------------------*/
/* Timer Timeout Check (for Glue Layer)                                       */
/*----------------------------------------------------------------------------*/
WLAN_STATUS wlanTimerTimeoutCheck(IN P_ADAPTER_T prAdapter);

/*----------------------------------------------------------------------------*/
/* Mailbox Message Check (for Glue Layer)                                     */
/*----------------------------------------------------------------------------*/
WLAN_STATUS wlanProcessMboxMessage(IN P_ADAPTER_T prAdapter);

/*----------------------------------------------------------------------------*/
/* TX Pending Packets Handling (for Glue Layer)                               */
/*----------------------------------------------------------------------------*/
WLAN_STATUS wlanEnqueueTxPacket(IN P_ADAPTER_T prAdapter, IN P_NATIVE_PACKET prNativePacket);

WLAN_STATUS wlanFlushTxPendingPackets(IN P_ADAPTER_T prAdapter);

WLAN_STATUS wlanTxPendingPackets(IN P_ADAPTER_T prAdapter, IN OUT PBOOLEAN pfgHwAccess);

/*----------------------------------------------------------------------------*/
/* Low Power Acquire/Release (for Glue Layer)                                 */
/*----------------------------------------------------------------------------*/
WLAN_STATUS wlanAcquirePowerControl(IN P_ADAPTER_T prAdapter);

WLAN_STATUS wlanReleasePowerControl(IN P_ADAPTER_T prAdapter);

/*----------------------------------------------------------------------------*/
/* Pending Packets Number Reporting (for Glue Layer)                          */
/*----------------------------------------------------------------------------*/
UINT_32 wlanGetTxPendingFrameCount(IN P_ADAPTER_T prAdapter);

/*----------------------------------------------------------------------------*/
/* ACPI state inquiry (for Glue Layer)                                        */
/*----------------------------------------------------------------------------*/
ENUM_ACPI_STATE_T wlanGetAcpiState(IN P_ADAPTER_T prAdapter);

VOID wlanSetAcpiState(IN P_ADAPTER_T prAdapter, IN ENUM_ACPI_STATE_T ePowerState);

VOID wlanDefTxPowerCfg(IN P_ADAPTER_T prAdapter);

/*----------------------------------------------------------------------------*/
/* get ECO version from Revision ID register (for Win32)                      */
/*----------------------------------------------------------------------------*/
UINT_8 wlanGetEcoVersion(IN P_ADAPTER_T prAdapter);

/*----------------------------------------------------------------------------*/
/* set preferred band configuration corresponding to network type             */
/*----------------------------------------------------------------------------*/
VOID
wlanSetPreferBandByNetwork(IN P_ADAPTER_T prAdapter, IN ENUM_BAND_T eBand, IN ENUM_NETWORK_TYPE_INDEX_T eNetTypeIndex);

/*----------------------------------------------------------------------------*/
/* get currently operating channel information                                */
/*----------------------------------------------------------------------------*/
UINT_8 wlanGetChannelNumberByNetwork(IN P_ADAPTER_T prAdapter, IN ENUM_NETWORK_TYPE_INDEX_T eNetTypeIndex);

/*----------------------------------------------------------------------------*/
/* get BSS Descriptor information                                             */
/*----------------------------------------------------------------------------*/
P_BSS_DESC_T wlanGetTargetBssDescByNetwork(IN P_ADAPTER_T prAdapter, IN ENUM_NETWORK_TYPE_INDEX_T eNetTypeIndex);

/*----------------------------------------------------------------------------*/
/* check for system configuration to generate message on scan list            */
/*----------------------------------------------------------------------------*/
WLAN_STATUS wlanCheckSystemConfiguration(IN P_ADAPTER_T prAdapter);

/*----------------------------------------------------------------------------*/
/* query sta statistics information from driver and firmware                  */
/*----------------------------------------------------------------------------*/
WLAN_STATUS
wlanoidQueryStaStatistics(IN P_ADAPTER_T prAdapter,
			  IN PVOID pvQueryBuffer, IN UINT_32 u4QueryBufferLen, OUT PUINT_32 pu4QueryInfoLen);

WLAN_STATUS wlanCfgParseArgument(CHAR *cmdLine, INT_32 *argc, CHAR *argv[]);

P_WLAN_CFG_ENTRY_T wlanCfgGetEntry(IN P_ADAPTER_T prAdapter, const PCHAR pucKey);

WLAN_STATUS
wlanCfgGet(IN P_ADAPTER_T prAdapter, const PCHAR pucKey, PCHAR pucValue, PCHAR pucValueDef, UINT_32 u4Flags);

UINT_32 wlanCfgGetUint32(IN P_ADAPTER_T prAdapter, const PCHAR pucKey, UINT_32 u4ValueDef);

INT_32 wlanCfgGetInt32(IN P_ADAPTER_T prAdapter, const PCHAR pucKey, INT_32 i4ValueDef);

WLAN_STATUS wlanCfgSetUint32(IN P_ADAPTER_T prAdapter, const PCHAR pucKey, UINT_32 u4Value);

WLAN_STATUS wlanCfgSet(IN P_ADAPTER_T prAdapter, const PCHAR pucKey, PCHAR pucValue, UINT_32 u4Flags);

WLAN_STATUS
wlanCfgSetCb(IN P_ADAPTER_T prAdapter, const PCHAR pucKey, WLAN_CFG_SET_CB pfSetCb, void *pPrivate, UINT_32 u4Flags);

#if CFG_SUPPORT_CFG_FILE
WLAN_STATUS wlanCfgInit(IN P_ADAPTER_T prAdapter, PUINT_8 pucConfigBuf, UINT_32 u4ConfigBufLen, UINT_32 u4Flags);

VOID wlanCfgApply(IN P_ADAPTER_T prAdapter);
#endif /* CFG_SUPPORT_CFG_FILE */
#endif /* _WLAN_LIB_H */
