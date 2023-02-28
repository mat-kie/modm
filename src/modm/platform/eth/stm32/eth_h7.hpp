/*
 * Copyright (c) 2020, Mattis Kieffer
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef MODM_ETH_HPP
#define MODM_ETH_HPP

#include <modm/platform/gpio/connector.hpp>
#include <modm/architecture/interface.hpp>
#include <modm/math/utils/bit_constants.hpp>
#include "../device.hpp"
#include <modm/platform/clock/rcc.hpp>
#include <modm/platform/eth/phy_registers.hpp>
#include <modm/processing/timer.hpp>
#include <array>
#include <etl/span.h>
#include <cstring>
#include <type_traits>

namespace modm
{
	namespace platform
	{

		/// @ingroup modm_platform_eth
		struct eth
		{
			typedef void (*notificationCB_t)(void);
			typedef uint8_t MacAddr_t[6];

			/** DMA mode register
			 */
			enum class DmaMode : uint32_t
			{
				SoftwareReset = modm::Bit0,
				ArbitrationScheme = modm::Bit1,
				TransmitPriority = modm::Bit11,
				PriorityRatio = modm::Bit12,
				InterruptMode = modm::Bit16,
			};
			MODM_FLAGS32(DmaMode);
			static constexpr uint32_t DmaModeReservedBits{~(Bit0 | Bit1 | Bit11 | Bit12 | Bit16)};

			enum class PriorityRatio : uint32_t
			{
				OneToOne = 0,
				TwoToOne = 1,
				ThreeToOne = 2,
				FourToOne = 3,
				FiveToOne = 4,
				SixToOne = 5,
				SevenToOne = 6,
				EightToOne = 7,
			};
			typedef modm::Configuration<DmaMode_t, PriorityRatio, 0b111, 12> PriorityRatio_t;

			/** System bus mode register
			 */
			enum class DmaSystemBusMode : uint32_t
			{
				FixedBurstLength = modm::Bit0,
				AddressAlignedBeats = modm::Bit12,
				MixedBurst = modm::Bit14,
				RebuildINCRxBurst = modm::Bit15,
			};
			MODM_FLAGS32(DmaSystemBusMode);
			static constexpr uint32_t DmaSystemBusModeReservedBits{~uint32_t(Bit0 | Bit12 | Bit14 | Bit15)};

			/** Interrupt status register
			 */
			enum class DmaInterruptStatus : uint32_t
			{
				DmaChan0IntStatus = modm::Bit0,
				MtlIntStatus = modm::Bit16,
				MacIntStatus = modm::Bit17,
			};
			MODM_FLAGS32(DmaInterruptStatus);
			static constexpr uint32_t DmaInterruptStatusReservedBits{~(Bit0 | Bit16 | Bit17)};

			/** Debug status register
			 */
			enum class DmaDebugStatus : uint32_t
			{
				AXWHSTS = modm::Bit0,
				RPS0 = modm::Bit8,
				TPS0 = modm::Bit12,
			};
			MODM_FLAGS32(DmaDebugStatus);

			/** Channel control register
			 */
			enum class DmaChannelControl : uint32_t
			{
				MaxSegmentSizeLSB = modm::Bit0,
				PBLX8 = modm::Bit16,
				DescriptorSkipLengthLSB = modm::Bit18,
			};
			MODM_FLAGS32(DmaChannelControl);
			static constexpr uint32_t DmaChannelControlReservedBits{Bit14 | Bit15 | Bit17 | Bit21 | Bit22 | Bit23 | Bit24 | Bit25 | Bit26 | Bit27 | Bit28 | Bit29 | Bit30 | Bit31};
			typedef modm::Value<DmaChannelControl_t, 14, 0> MaxSegmentSize_t;
			typedef modm::Value<DmaChannelControl_t, 3, 18> DescriptorSkipLength_t;

			/** Channel transmit control
					  register
			*/
			enum class DmaChannelTransmitControl : uint32_t
			{
				StartTx = modm::Bit0,
				OperateSecondFrame = modm::Bit4,
				TcpSegmentationEnable = modm::Bit12,
				TxBurstLengthLSB = modm::Bit16,
			};
			MODM_FLAGS32(DmaChannelTransmitControl);
			static constexpr uint32_t DmaChannelTransmitControlReservedBits{~(Bit0 | Bit4 | Bit12 | Bit17 | Bit18 | Bit19 | Bit20 | Bit21)};
			typedef modm::Value<DmaChannelTransmitControl_t, 6, 16> TxBurstLength_t;
			/** Channel receive control
					  register
			*/
			enum class DmaChannelReceiveControl : uint32_t
			{
				StartRx = modm::Bit0,
				RxBufferSizeLSB = modm::Bit1,
				RxBurstLengthLSB = modm::Bit16,
				RxPacketFlush = modm::Bit31,
			};
			MODM_FLAGS32(DmaChannelReceiveControl);
			static constexpr uint32_t DmaChannelReceiveControlReservedBits{Bit15 | Bit22 | Bit23 | Bit24 | Bit25 | Bit26 | Bit27 | Bit28 | Bit29 | Bit30};
			typedef modm::Value<DmaChannelReceiveControl_t, 6, 16> RxBurstLength_t;
			typedef modm::Value<DmaChannelReceiveControl_t, 14, 1> RxBufferSize_t;

			/** Channel interrupt enable
					  register
			*/
			enum class DmaChannelInterruptEnable : uint32_t
			{
				TxInt = modm::Bit0,
				TxStopped = modm::Bit1,
				TxBufferUnavailable = modm::Bit2,
				RxInt = modm::Bit6,
				RxBufferUnavailable = modm::Bit7,
				RxStopped = modm::Bit8,
				RxWdgTimeout = modm::Bit9,
				EarlyTx = modm::Bit10,
				EarlyRx = modm::Bit11,
				FatalBusError = modm::Bit12,
				CtxDescriptorError = modm::Bit13,
				AbnormalInterruptSummaryEnable = modm::Bit14,
				NormalInterruptSummaryEnable = modm::Bit15,
			};
			MODM_FLAGS32(DmaChannelInterruptEnable);
			static constexpr uint32_t DmaChannelInterruptEnableReservedBits{~uint32_t(Bit0 | Bit1 | Bit2 | Bit6 | Bit7 | Bit8 | Bit9 | Bit10 | Bit11 | Bit12 | Bit13 | Bit14 | Bit15)};

			/** Channel status register
			 */
			enum class DmaChannelStatus : uint32_t
			{
				TxInterrupt = modm::Bit0,
				TxProcessStopped = modm::Bit1,
				TxBufferUnavailable = modm::Bit2,
				RxInterrupt = modm::Bit6,
				RxBufferUnavailable = modm::Bit7,
				RxProcessStopped = modm::Bit8,
				RxWdgTimeout = modm::Bit9,
				EarlyTx = modm::Bit10,
				EarlyRx = modm::Bit11,
				FatalBusError = modm::Bit12,
				CtxDescriptorError = modm::Bit13,
				AbnormalInterruptSummary = modm::Bit14,
				NormalInterruptSummary = modm::Bit15,
				TxDmaErrorLSB = modm::Bit16,
				RxDmaErrorLSB = modm::Bit19,
			};
			MODM_FLAGS32(DmaChannelStatus);
			static constexpr uint32_t DmaChannelStatusReservedBits{Bit3 | Bit4 | Bit5 | Bit22 | Bit23 | Bit24 | Bit25 | Bit26 | Bit27 | Bit28 | Bit29 | Bit30 | Bit31};

			/** Operating mode Register
			 */
			enum class MtlOperatingMode : uint32_t
			{
				DropTxStatus = modm::Bit1,
				CountersOreset = modm::Bit8,
				CountersReset = modm::Bit9,
			};
			MODM_FLAGS32(MtlOperatingMode);
			static constexpr uint32_t MtlOperatingModeReservedBits{~uint32_t(Bit1 | Bit8 | Bit9)};

			/** Interrupt status Register
			 */
			enum class MtlInterruptStatus : uint32_t
			{
				QueueIntStatus = modm::Bit0,
			};
			MODM_FLAGS32(MtlInterruptStatus);
			static constexpr uint32_t MtlInterruptStatusReservedBits{~uint32_t(Bit0)};

			/** Tx queue operating mode
					  Register
			*/
			enum class MtlTxQueueOperatingMode : uint32_t
			{
				FlushTxQueue = modm::Bit0,
				TxStoreAndForward = modm::Bit1,
				TxQueueEnable = modm::Bit3,
				TxThresholdCtrlLSB = modm::Bit4,
				TxQueueSizeLSB = modm::Bit16,
			};
			MODM_FLAGS32(MtlTxQueueOperatingMode);
			static constexpr uint32_t MtlTxQueueOperatingModeReservedBits{~(Bit0 | Bit1 | Bit2 | Bit3 | Bit4 | Bit5 | Bit6 | Bit16 | Bit17 | Bit18)};

			enum class MtlTxThresholdCtrl : uint32_t
			{
				Level32 = 0,
				Level64 = 1,
				Level96 = 2,
				Level128 = 3,
				Level192 = 4,
				Level256 = 5,
				Level384 = 6,
				Level512 = 7
			};
			enum class MtlTxQueueSize : uint32_t
			{
				Size256 = 0,
				Size512 = 1,
				Size768 = 2,
				Size1024 = 3,
				Size1280 = 4,
				Size1536 = 5,
				Size1792 = 6,
				Size2048 = 7
			};
			typedef modm::Configuration<MtlTxQueueOperatingMode_t, MtlTxThresholdCtrl, 0b111, 4> MtlTxThresholdCtrl_t;
			typedef modm::Configuration<MtlTxQueueOperatingMode_t, MtlTxQueueSize, 0b111, 16> MtlTxQueueSize_t;

			/** Rx queue operating mode
					  register
			*/
			enum class MtlRxQueueOperatingMode : uint32_t
			{
				RxThresholdCtrlLSB = modm::Bit0,
				ForwardUndersizedPackets = modm::Bit3,
				ForwardErrorPackets = modm::Bit4,
				RxStoreAndForward = modm::Bit5,
				DisableTCPErrorDrop = modm::Bit6,
				RxQueueSizeLSB = modm::Bit20,
			};
			MODM_FLAGS32(MtlRxQueueOperatingMode);
			static constexpr uint32_t MtlRxQueueOperatingModeReservedBits{~(Bit0 | Bit1 | Bit3 | Bit4 | Bit5 | Bit6 | Bit20 | Bit21 | Bit22)};

			enum class MtlRxThresholdCtrl : uint32_t
			{
				Level64 = 0,
				Level32 = 1,
				Level96 = 2,
				Level128 = 3
			};
			typedef modm::Configuration<MtlRxQueueOperatingMode_t, MtlRxThresholdCtrl, 0b11, 0> MtlRxThresholdCtrl_t;

			/** Operating mode configuration
					  register
			*/
			enum class MacConfiguration : uint32_t
			{
				RxEnable = modm::Bit0,
				TxEnable = modm::Bit1,
				PreambleLengthLSB = modm::Bit2,
				DeferralCheck = modm::Bit4,
				BackoffLimitLSB = modm::Bit5,
				DisableRetry = modm::Bit8,
				DisableCrsTx = modm::Bit9,
				DisableRxOwn = modm::Bit10,
				EnableCRSFullDuplex = modm::Bit11,
				LoopbackMode = modm::Bit12,
				FullDuplexMode = modm::Bit13,
				FastEthernetSpeed = modm::Bit14,
				JumboPacketEnable = modm::Bit16,
				JabberDisable = modm::Bit17,
				WdgDisable = modm::Bit19,
				AutoCrcStripping = modm::Bit20,
				CrcStripTypePacket = modm::Bit21,
				Support2KPackets = modm::Bit22,
				GiantPacketSizeLimitControl = modm::Bit23,
				InterPacketGapLSB = modm::Bit24,
				IPChecksumOffload = modm::Bit27,
				SourceAddressInsertionLSB = modm::Bit28,
				ARPOffload = modm::Bit31,
			};
			MODM_FLAGS32(MacConfiguration);
			static constexpr uint32_t MacConfigurationReservedBits{Bit7 | Bit15 | Bit18};

			enum class BackoffLimit : uint32_t
			{
				Limit10 = 0,
				Limit8 = 1,
				Limit4 = 2,
				Limit1 = 3
			};
			typedef modm::Configuration<MacConfiguration_t, BackoffLimit, 0b11, 5> BackoffLimit_t;

			enum class InterPacketGap : uint32_t
			{
				Gap96 = 0,
				Gap88,
				Gap80,
				Gap72,
				Gap64,
				Gap56,
				Gap48,
				Gap40
			};
			typedef modm::Configuration<MacConfiguration_t, InterPacketGap, 0b111, 24> InterPacketGap_t;

			enum class MacCrSourceAddressInsertionControl : uint32_t
			{
				InsertMac0 = 0b010,
				ReplaceMac0 = 0b011,
				InsertMac1 = 0b110,
				ReplaceMac1 = 0b111
			};
			typedef modm::Configuration<MacConfiguration_t, MacCrSourceAddressInsertionControl, 0b111, 28> MacCrSourceAddressInsertionControl_t;

			/** Packet filtering control
					  register
			*/
			enum class MacPacketFilteringControl : uint32_t
			{
				Promiscuous = modm::Bit0,
				HashUnicast = modm::Bit1,
				HashMulticast = modm::Bit2,
				DestinationAddressInverseFilter = modm::Bit3,
				PassAllMulticast = modm::Bit4,
				DisableBroadcastFrames = modm::Bit5,
				PassControlFramesLSB = modm::Bit6,
				SourceAddressInverseFilter = modm::Bit8,
				SourceAddressFilter = modm::Bit9,
				HashOrPerfectFilter = modm::Bit10,
				VlanTagFilter = modm::Bit16,
				IPFilter = modm::Bit20,
				DropNonTcpUdp = modm::Bit21,
				RxAll = modm::Bit31,
			};
			MODM_FLAGS32(MacPacketFilteringControl);
			static constexpr uint32_t MacPacketFilteringControlReservedBits{Bit11 | Bit12 | Bit13 | Bit14 | Bit15 | Bit17 | Bit18 | Bit19 | Bit22 | Bit23 | Bit24 | Bit25 | Bit26 | Bit27 | Bit28 | Bit29 | Bit30};

			enum class PassControlPacket : uint32_t
			{
				BlockAll = 0,
				BlockPause,
				BlockNone,
				BlockFailedAF
			};
			typedef modm::Configuration<MacPacketFilteringControl_t, PassControlPacket, 0b11, 6> PassControlPacket_t;

			/** Tx Queue flow control register
			 */
			enum class TxQueueFlowControl : uint32_t
			{
				FCBusy = modm::Bit0,
				TxFlowcontrolEnable = modm::Bit1,
				PauseLowThresholdLSB = modm::Bit4,
				DisableZeroQuantaPause = modm::Bit7,
				PauseTimeLSB = modm::Bit16,
			};
			MODM_FLAGS32(TxQueueFlowControl);

			enum class PauseTImeLowThreshold : uint32_t
			{
				PTMinus4 = 0,
				PTMinus28 = 1,
				PTMinus36 = 2,
				PTMinus144 = 3,
				PTMinus256 = 4,
				PTMinus512 = 5
			};
			typedef modm::Configuration<TxQueueFlowControl_t, PauseTImeLowThreshold, 0b111, 4> PauseTImeLowThreshold_t;
			typedef modm::Value<TxQueueFlowControl_t, 16, 16> PauseTime_t;

			/** Rx flow control register
			 */
			enum class RxFlowControl : uint32_t
			{
				RxFlowControlEnable = modm::Bit0,
				UnicastPauseDetect = modm::Bit1,
			};
			MODM_FLAGS32(RxFlowControl);

			/** Interrupt status register
			 */
			enum class MacInterruptStatus : uint32_t
			{
				Phy = modm::Bit3,
				Pmt = modm::Bit4,
				Lpi = modm::Bit5,
				Mmc = modm::Bit8,
				MmcRx = modm::Bit9,
				MmcTx = modm::Bit10,
				Timestamp = modm::Bit12,
				TxStatus = modm::Bit13,
				RxStatus = modm::Bit14,
			};
			MODM_FLAGS32(MacInterruptStatus);

			/** Interrupt enable register
			 */
			enum class MacInterruptEnable : uint32_t
			{
				Phy = modm::Bit3,
				Pmt = modm::Bit4,
				Lpi = modm::Bit5,
				Timestamp = modm::Bit12,
				TxStatus = modm::Bit13,
				RxStatus = modm::Bit14,
			};
			MODM_FLAGS32(MacInterruptEnable);

			/** Rx Tx status register
			 */
			enum class RxTxStatus : uint32_t
			{
				TJT = modm::Bit0,
				NCARR = modm::Bit1,
				LCARR = modm::Bit2,
				EXDEF = modm::Bit3,
				LCOL = modm::Bit4,
				EXCOL = modm::Bit5,
				RWT = modm::Bit8,
			};
			MODM_FLAGS32(RxTxStatus);

			/** MDIO address register
			 */
			enum class MdioAddress : uint32_t
			{
				MiiBusy = modm::Bit0,
				Clause45 = modm::Bit1,
				MiiOpCodeLSB = modm::Bit2,
				SkipAddressPacket = modm::Bit4,
				CsrClockRangeLSB = modm::Bit8,
				NumTrailingClocksLSB = modm::Bit12,
				RegDeviceAddressLSB = modm::Bit16,
				PhysicalAddressLSB = modm::Bit21,
				BackToBack = modm::Bit26,
				PreambleSuppression = modm::Bit27,
			};
			MODM_FLAGS32(MdioAddress);

			enum class MdcClockDivider : uint32_t
			{
				Div42 = 0,
				Div62 = 1,
				Div16 = 2,
				Div26 = 3,
				Div102 = 4,
				Div124 = 5,
			};
			enum class MdioOpCode : uint32_t
			{
				Write = 0b01,
				Read = 0b11
			};
			typedef modm::Configuration<MdioAddress_t, MdcClockDivider, 0b1111, 8> MdcClockDivider_t;
			typedef modm::Configuration<MdioAddress_t, MdioOpCode, 0b11, 2> MdioOpCode_t;
			typedef modm::Value<MdioAddress_t, 3, 12> MdioNumTrailingClocks_t;
			typedef modm::Value<MdioAddress_t, 5, 16> MdioRegDeviceAddress_t;
			typedef modm::Value<MdioAddress_t, 5, 21> MdioPhysicalAddress_t;

			/** MDIO data register
			 */
			enum class MdioData : uint32_t
			{
				MiiDataLSB = modm::Bit0,
				RegisterAddressLSB = modm::Bit16,
			};
			MODM_FLAGS32(MdioData);
			typedef modm::Value<MdioData_t, 16, 0> MiiData_t;
			typedef modm::Value<MdioData_t, 16, 16> MiiRegisterAddress_t;

			enum class MediaInterface : uint32_t
			{
				MII = 0x00,
				RMII = SYSCFG_PMCR_EPIS_SEL_2 ///< TODO: ?
			};
			////////////////////////
			enum class NormalTDes2 : uint32_t
			{
				InterruptOnCompletion = modm::Bit31,
				TransmitTimestampEnable = modm::Bit30,
				Buffer2LengthLSB = modm::Bit16,
				Buffer1LengthLSB = modm::Bit0,
			};
			MODM_FLAGS32(NormalTDes2);
			typedef Value<NormalTDes2_t, 13, 0> TxBuffer1Size_t;
			typedef Value<NormalTDes2_t, 13, 16> TxBuffer2Size_t;

			enum class NormalTDes3 : uint32_t
			{
				DmaOwned = Bit31,
				ContextType = Bit30,
				FirstDescriptor = Bit29,
				LastDescriptor = Bit28,
				TcpSegmentationEnable = Bit18,

			};
			MODM_FLAGS32(NormalTDes3);

			enum class CrcPadControl : uint32_t
			{
				AppendPadAndCrc,
				AppendCrc,
				DisableCrcInsertion,
				ReplaceCrc
			};
			typedef modm::Configuration<NormalTDes3_t, CrcPadControl, 0b11, 26> CrcPadControl_t;

			enum class IPChecksumControl : uint32_t
			{
				InsertionDisabled = 0b00,
				IpHeaderOnly = 0b01,
				IpHeaderAndPayload = 0b10,
				HardwareCalculated = 0b11
			};
			typedef modm::Configuration<NormalTDes3_t, IPChecksumControl, 0b11, 16> IPChecksumControl_t;

			enum class SourceAddressInsertionControl : uint32_t
			{
				DoNotInclude,
				InsertSourceAddress,
				ReplaceSourceAddress
			};
			typedef modm::Configuration<NormalTDes3_t, SourceAddressInsertionControl, 0b11, 16> SourceAddressInsertionControl_t;
			typedef Value<NormalTDes3_t, 17, 0> TcpPayloadLength_t;
			typedef Value<NormalTDes3_t, 4, 19> TcpHeaderLength_t;

			enum class WritebackTDes3 : uint32_t
			{
				DmaOwned = Bit31,
				ContextType = Bit30,
				FirstDescriptor = Bit29,
				LastDescriptor = Bit28,
				TxTimestampStatus = Bit17,
				ErrorSummary = Bit15

			};
			MODM_FLAGS32(WritebackTDes3);

			enum class TDes1 : uint32_t
			{
				TBS1_0 = Bit0,
				TBS1_1 = Bit1,
				TBS1_2 = Bit2,
				TBS1_3 = Bit3,
				TBS1_4 = Bit4,
				TBS1_5 = Bit5,
				TBS1_6 = Bit6,
				TBS1_7 = Bit7,
				TBS1_8 = Bit8,
				TBS1_9 = Bit9,
				TBS1_10 = Bit10,
				TBS1_11 = Bit11,
				TBS1_12 = Bit12,
				TBS2_0 = Bit16,
				TBS2_1 = Bit17,
				TBS2_2 = Bit18,
				TBS2_3 = Bit19,
				TBS2_4 = Bit20,
				TBS2_5 = Bit21,
				TBS2_6 = Bit22,
				TBS2_7 = Bit23,
				TBS2_8 = Bit24,
				TBS2_9 = Bit25,
				TBS2_10 = Bit26,
				TBS2_11 = Bit27,
				TBS2_12 = Bit28
			};
			MODM_FLAGS32(TDes1);

			enum class NormalRDes3 : uint32_t
			{
				DmaOwned = modm::Bit31,
				EnableInterruptOnCompletition = Bit30,
				Buffer2AddressValid = Bit25,
				Buffer1AddressValid = Bit24
			};
			MODM_FLAGS32(NormalRDes3);

			enum class WritebackRDes0 : uint32_t
			{
				InnerVlanLSB = Bit16,
				OuterVlanLSB = Bit0
			};
			MODM_FLAGS32(WritebackRDes0);
			typedef modm::Value<WritebackRDes0_t, 16, 0> RDesOuterVlanTag_t;
			typedef modm::Value<WritebackRDes0_t, 16, 16> RDesInnerVlanTag_t;


			enum class WritebackRDes1 : uint32_t
			{
				TimestampDropped = Bit15,
				TimestampAvailable = Bit14,
				PtpVersion = Bit13,
				PtpRawEth = Bit12,
				IPPayloadError = Bit7
			};
			MODM_FLAGS32(WritebackRDes1);

			enum class WritebackRDes2 : uint32_t
			{
				/// TODO: Bit defs
			};
			MODM_FLAGS32(WritebackRDes2);

			enum class WritebackRDes3 : uint32_t
			{
				DmaOwned = modm::Bit31,
				ContextType = Bit30,
				FirstDescriptor = Bit29,
				LastDescriptor = Bit28,
				RDes2Valid = Bit27,
				RDes1Valid = Bit26,
				RDes0Valid = Bit25,
				CrcError = Bit24,
				GiantPacket = Bit23,
				RxWdgTimeout = Bit22,
				OverflowError = Bit21,
				RxError = Bit20,
			};
			MODM_FLAGS32(WritebackRDes3);
			typedef modm::Value<WritebackRDes3_t, 15, 0> RDes3RxPacketLength_t;

			enum class ContextRDes3 : uint32_t
			{
				DmaOwned = modm::Bit31,
				ContextType = Bit30
			};
			MODM_FLAGS32(ContextRDes3);

			enum class DescriptorType
			{
				Normal,
				Writeback,
				Context
			};

			struct NormalTxDmaDescriptor
			{
				uint32_t buffer1Address;
				uint32_t buffer2Address;
				NormalTDes2_t controlBufferSize;
				NormalTDes3_t control;
			};

			struct WritebackTxDmaDescriptor
			{
				uint32_t timestampLow;
				uint32_t timestampHigh;
				uint32_t _reserved;
				WritebackTDes3_t status;
			};

			struct NormalRxDmaDescriptor
			{
				uint32_t buffer1Address;
				uint32_t _reserved;
				uint32_t buffer2Address;
				NormalRDes3_t control;
			};

			struct WritebackRxDmaDescriptor
			{
				WritebackRDes0_t vlanInfo;
				WritebackRDes1_t extendedStatus;
				WritebackRDes2_t filterStatus;
				WritebackRDes3_t statusAndLength;

				size_t getSize()const{
					return RDes3RxPacketLength_t::get(statusAndLength);
				}
			};
			struct ContextRxDmaDescriptor
			{
				uint32_t timestampLow;
				uint32_t timestampHigh;
				uint32_t _reserved;
				uint32_t status;
			};

			struct TxDmaDescriptor
			{
				uint32_t word0;
				uint32_t word1;
				uint32_t word2;
				uint32_t word3;
				DescriptorType getType()
				{
					if (word3 & Bit31)
					{
						// dma owned -> normal desc.
						if ((word3 & Bit30) != 0)
						{
							return DescriptorType::Context;
						}
						else
						{
							return DescriptorType::Normal;
						}
					}
					else
					{
						if ((word3 & Bit30) != 0)
						{
							return DescriptorType::Context;
						}
						else
						{
							return DescriptorType::Writeback;
						}
					}
				};
				TxDmaDescriptor &operator=(const NormalTxDmaDescriptor &rhs)
				{
					word0 = rhs.buffer1Address;
					word1 = rhs.buffer2Address;
					word2 = rhs.controlBufferSize.value;
					word3 = rhs.control.value;
					return *this;
				};

				const WritebackTxDmaDescriptor &asWriteback() const
				{
					// todo: assert correct type in debug
					return reinterpret_cast<const WritebackTxDmaDescriptor &>(*this);
				};
			};

			struct RxDmaDescriptor
			{
				uint32_t word0;
				uint32_t word1;
				uint32_t word2;
				uint32_t word3;
				DescriptorType getType()
				{
					if (word3 & Bit31)
					{
						// dma owned -> normal desc.
						return DescriptorType::Normal;

					}
					else
					{
						if ((word3 & Bit30) != 0)
						{
							return DescriptorType::Context;
						}
						else
						{
							return DescriptorType::Writeback;
						}
					}
				};

				RxDmaDescriptor &operator=(const NormalRxDmaDescriptor & rhs)
				{
					word0 = rhs.buffer1Address;
					word1 = rhs._reserved;
					word2 = rhs.buffer2Address;
					word3 = rhs.control.value;
					return *this;
				};
				const WritebackRxDmaDescriptor &asWriteback() const
				{
					return reinterpret_cast<const WritebackRxDmaDescriptor &>(*this);
				};
				const ContextRxDmaDescriptor &asContext() const
				{
					return reinterpret_cast<const ContextRxDmaDescriptor &>(*this);
				};
			};

			enum class MacAddressIndex
			{
				MacAddress0,
				MacAddress1 = 8,
				MacAddress2 = 16,
				MacAddress3 = 24
			};
		};

		/// @ingroup modm_platform_eth
		/// TODO:
		///   - setting/resetting mac adresses in dest. address filter
		///   - setting/resetting mac adresses in dest. address filter
		class Eth : public eth
		{

			struct IsrCallbacks_t
			{

				notificationCB_t Transmit{nullptr};
				notificationCB_t TransmitStopped{nullptr};
				notificationCB_t TransmitBufferUnavailable{nullptr};
				notificationCB_t TransmitJabberTimeout{nullptr};
				notificationCB_t ReceiveOverflow{nullptr};
				notificationCB_t TransmitUnderflow{nullptr};
				notificationCB_t Receive{nullptr};
				notificationCB_t ReceiveBufferUnavailable{nullptr};
				notificationCB_t ReceiveStopped{nullptr};
				notificationCB_t ReceiveWatchdog{nullptr};
				notificationCB_t EarlyReceive{nullptr};
				notificationCB_t FatalBusError{nullptr};
				notificationCB_t EarlyTransmit{nullptr};
				notificationCB_t TimestampTrigger{nullptr};
				notificationCB_t PMTEvent{nullptr};
			};

			static IsrCallbacks_t EthIsrCallbacks;

		public:
			template <class... Signals>
			static void
			connect()
			{
				(GpioStatic<typename Signals::Data>::configure(Gpio::OutputType::PushPull, Gpio::OutputSpeed::VeryHigh), ...);
				GpioConnector<Peripheral::Eth, Signals...>::connect();
			}

			template <MediaInterface Interface>
			static inline bool
			initialize(uint8_t priority = 5)
			{
				using namespace modm::literals;
				Rcc::enable<Peripheral::Eth>();

				NVIC_SetPriority(ETH_IRQn, priority);
				NVIC_EnableIRQ(ETH_IRQn);

				/* Select MII or RMII Mode*/
				SYSCFG->PMCR &= ~(SYSCFG_PMCR_EPIS_SEL_Msk);
				SYSCFG->PMCR |= uint32_t(Interface); // rmii

				if (not softReset())
					return false;

				/* Configure SMI clock range */
				MdioAddress_t mdioar(ETH->MACMDIOAR);
				mdioar.reset(MdcClockDivider_t::mask());
				if (SystemCoreClock >= 20_MHz and SystemCoreClock < 35_MHz)
					mdioar |= MdcClockDivider_t(MdcClockDivider::Div16);
				else if (SystemCoreClock >= 35_MHz and SystemCoreClock < 60_MHz)
					mdioar |= MdcClockDivider_t(MdcClockDivider::Div26);
				else if (SystemCoreClock >= 60_MHz and SystemCoreClock < 100_MHz)
					mdioar |= MdcClockDivider_t(MdcClockDivider::Div42);
				else if (SystemCoreClock >= 100_MHz and SystemCoreClock < 150_MHz)
					mdioar |= MdcClockDivider_t(MdcClockDivider::Div62);
				else if (SystemCoreClock >= 150_MHz and SystemCoreClock < 250_MHz)
					mdioar |= MdcClockDivider_t(MdcClockDivider::Div102);
				else if (SystemCoreClock >= 250_MHz)
					mdioar |= MdcClockDivider_t(MdcClockDivider::Div124);

				writeSafe(mdioar);

				return true;
			}

			static void
			configureMac(MacConfiguration_t cfg, MacPacketFilteringControl_t filter_cfg, TxQueueFlowControl_t fc_cfg)
			{
				MacConfiguration_t maccr(ETH->MACCR & MacConfigurationReservedBits);
				maccr |= cfg;
				writeSafe(maccr);
				MacPacketFilteringControl_t macpfcr(ETH->MACPFR & MacPacketFilteringControlReservedBits);
				macpfcr |= filter_cfg;
				writeSafe(macpfcr);
				TxQueueFlowControl_t macqtxfcr(ETH->MACTFCR);
				macqtxfcr |= fc_cfg;
				writeSafe(macqtxfcr);
			}

			static bool softReset()
			{
				/* Ethernet Software reset */
				/* Set the SWR bit: resets all MAC subsystem internal registers and logic */
				/* After reset all the registers holds their respective reset values */
				ETH->DMAMR |= uint32_t(DmaMode::SoftwareReset);

				/* Wait for software reset */
				/* Note: The SWR is not performed if the ETH_RX_CLK or the ETH_TX_CLK are
				 * not available, please check your external PHY or the IO configuration */
				auto timeout = modm::PreciseClock::now() + 4ms;
				while (modm::PreciseClock::now() <= timeout)
				{
					if (DmaMode_t(ETH->DMAMR).none(DmaMode::SoftwareReset))
					{
						return true;
					}
				}
				/// NOTE: If the program gets here check MII/RMII value.
				return false;
			}

			// TxDescBase, RxDescBase, RxBufferSize, RxTailPtr, TxTailPtr
			// etl span TxDesc RxDesc rxBufferSize
			static void
			configureDma(etl::span<TxDmaDescriptor> txDescriptors, etl::span<RxDmaDescriptor> rxDescriptors, size_t rxBufferSize)
			{
				// modm_assert(rxBufferSize%4==0);

				// 3. Program the following fields to initialize the System bus mode register(ETH_DMASBMR):
				//		a)AAL
				//		b)Fixed burst or undefined burst
				DmaSystemBusMode_t dmasbmr(ETH->DMASBMR & DmaSystemBusModeReservedBits);
				dmasbmr |= DmaSystemBusMode::AddressAlignedBeats | DmaSystemBusMode::FixedBurstLength;
				//		c)Burst mode values in case of AHB bus interface.
				// see 7.
				writeSafe(dmasbmr);

				// Precondition 4. Create a transmit and a receive descriptor list. In addition, ensure that the receive descriptors are owned by the DMA (set bit 31 of TDES3/RDES3 descriptor).

				// 5. Program ETH_DMACTXRLR and ETH_DMACRXRLR registers (see Channel Txdescriptor ring length register (ETH_DMACTXRLR) and Channel Rx descriptor ringlength register (ETH_DMACRXRLR)).
				ETH->DMACRDRLR = rxDescriptors.size() - 1; // < RefMan says for 10 descriptors set to 9...
				ETH->DMACTDRLR = txDescriptors.size() - 1;

				// 6. Initialize receive and transmit descriptor list address with the base address of transmit and receive descriptor (Channel Tx descriptor list address register(ETH_DMACTXDLAR), Channel Rx descriptor list address register (ETH_DMACRXDLAR)). In addition, program the transmit and receive tail pointer registers that inform the DMA about the available descriptors (see Channel Tx descriptor tail pointer register (ETH_DMACTXDTPR) and Channel Rx descriptor tail pointer register (ETH_DMACRXDTPR)).
				ETH->DMACTDLAR = (uint32_t)txDescriptors.data(); // tx Descriptor list
				ETH->DMACRDLAR = (uint32_t)rxDescriptors.data(); // rx Descriptor list
				ETH->DMACTDTPR = (uint32_t)txDescriptors.end();
				ETH->DMACRDTPR = (uint32_t)rxDescriptors.end();

				// 7. Program ETH_DMACCR, ETH_DMACTXCR and ETH_DMACRXCR registers (seeChannel control register (ETH_DMACCR), Channel transmit control register(ETH_DMACTXCR) and Channel receive control register (ETH_DMACRXCR)) to
				DmaChannelTransmitControl_t dmactxcr(ETH->DMACTCR & DmaChannelTransmitControlReservedBits);
				dmactxcr |= TxBurstLength_t(32); // only 1, 2, 4, 8, 16, or 32 are valid;
				writeSafe(dmactxcr);

				DmaChannelReceiveControl_t dmacrxcr(ETH->DMACRCR & DmaChannelReceiveControlReservedBits);
				dmacrxcr |= RxBurstLength_t(32);		  // only 1, 2, 4, 8, 16, or 32 are valid;
				dmacrxcr |= RxBufferSize_t(rxBufferSize); //< has to be a multiple of 4
				writeSafe(dmacrxcr);

				// Condigure MTL
				MtlTxQueueOperatingMode_t mtltxqomr(ETH->MTLTQOMR & MtlTxQueueOperatingModeReservedBits);
				mtltxqomr |= MtlTxQueueOperatingMode::FlushTxQueue | MtlTxQueueOperatingMode::TxStoreAndForward;
				MtlRxQueueOperatingMode_t mtlrxqomr(ETH->MTLRQOMR & MtlRxQueueOperatingModeReservedBits);
				mtlrxqomr |= MtlRxQueueOperatingMode::RxThresholdCtrlLSB | MtlRxQueueOperatingMode::ForwardUndersizedPackets | MtlRxQueueOperatingMode::ForwardErrorPackets; // set to 32
				writeSafe(mtltxqomr);
				writeSafe(mtlrxqomr);
			}

			/// @brief set the corresponding mac filter, or clear it by passing nullptr
			static void
			setMacFilter(MacAddressIndex index, uint8_t const *macAddress = nullptr)
			{
				if (macAddress != nullptr)
				{
					uint32_t tmp_register = (macAddress[5] << 8) | macAddress[4];
					if (index != MacAddressIndex::MacAddress0)
					{
						// destination address filter mode, compare all 6 bytes
						// tmp_register &= ~(Bit24 | Bit25  | Bit26  | Bit27  | Bit28  | Bit29  | Bit30);
						// enable mac filter
						tmp_register |= Bit31;
					}
					*reinterpret_cast<__IO uint32_t *>(&(ETH->MACA0HR) + uint32_t(index)) = tmp_register;

					tmp_register = (macAddress[3] << 24) | (macAddress[2] << 16) | (macAddress[1] << 8) | macAddress[0];
					*reinterpret_cast<__IO uint32_t *>(&(ETH->MACA0LR) + uint32_t(index)) = tmp_register;
				}
			}

			static void
			start()
			{
				// DMA transmission enable
				// DMA reception enable
				DmaChannelReceiveControl_t dmacrxcr(ETH->DMACRCR);
				dmacrxcr |= DmaChannelReceiveControl::StartRx;
				DmaChannelTransmitControl_t dmactxcr(ETH->DMACTCR);
				dmactxcr |= DmaChannelTransmitControl::StartTx;
				writeSafe(dmacrxcr);
				writeSafe(dmactxcr);
				// transmission enable
				// reception enable
				MacConfiguration_t maccr(ETH->MACCR);
				maccr |= MacConfiguration::TxEnable | MacConfiguration::RxEnable;
				writeSafe(maccr);
			};
			static void
			stop()
			{
				// DMA transmission disable
				// DMA reception disable
				DmaChannelTransmitControl_t dmactxcr(ETH->DMACTCR);
				dmactxcr.reset(DmaChannelTransmitControl::StartTx);
				writeSafe(dmactxcr);
				modm::delay(2ms); /// TODO: wait for tx to complete
				MacConfiguration_t maccr(ETH->MACCR);
				maccr.reset(MacConfiguration::TxEnable | MacConfiguration::RxEnable);
				writeSafe(maccr);
				modm::delay(2ms); /// TODO: wait for rx dma to complete
				DmaChannelReceiveControl_t dmacrxcr(ETH->DMACRCR);
				dmacrxcr.reset(DmaChannelReceiveControl::StartRx);
				writeSafe(dmacrxcr);
			};

			// interrupt regs: DMAISR-> only see if Mac DMA or MTL is source
			// DMACSR: Dma Intrruots 11
			// MACISR: mac interrupts 9
			// MTLISR Mtl Interrupts
			static DmaInterruptStatus_t
			getInterruptSource()
			{
				return DmaInterruptStatus_t(ETH->DMAISR);
			}

			static DmaChannelStatus_t getDmaChanIntStatus()
			{
				return DmaChannelStatus_t(ETH->DMACSR);
			};

			static void acknowlegeDmaChanIntStatus(DmaChannelStatus_t irq)
			{
				// rc_w1 register, clear by writing 1
				ETH->DMACSR = irq.value;
			}

			static MacInterruptStatus_t getMacIntStatus(){
				return MacInterruptStatus_t(ETH->MACISR);
			}

			static void acknowlegeMacIntStatus(MacInterruptStatus_t irq)
			{
				ETH->MACISR = irq.value;
			}

			static void
			enableInterrupt(DmaChannelInterruptEnable_t irq, notificationCB_t callback)
			{
				DmaChannelInterruptEnable_t dmacier(ETH->DMACIER);
				if(irq.any(DmaChannelInterruptEnable::RxInt))
				{
					EthIsrCallbacks.Receive = callback;
					dmacier.set(DmaChannelInterruptEnable::NormalInterruptSummaryEnable);
				}
				if(irq.any(DmaChannelInterruptEnable::TxInt))
				{
					EthIsrCallbacks.Transmit = callback;
					dmacier.set(DmaChannelInterruptEnable::NormalInterruptSummaryEnable);
				}
				dmacier|= irq;
				ETH->DMACIER = dmacier.value;
			}
			// enable the mac interrupt irq
			static void
			enableInterrupt(MacInterruptEnable irq, notificationCB_t callback)
			{
				MacInterruptEnable_t macier(ETH->MACIER);
				macier.set(irq);
				ETH->MACIER = macier.value;
			}

			static void NotifyFromISR(MacInterruptStatus_t s);
			static void NotifyFromISR(DmaChannelStatus_t s);

			template <class PHYUser, class RegisterType>
			static bool
			writePhyRegister(RegisterType value)
			{
				return transferPhyRegister(PHYUser::Address, PHYUser::WriteTimeout, PHY::getRegisterNumber<RegisterType>(), value.value, true);
			}
			template <class PHYUser, class RegisterType>
			static bool
			readPhyRegister(RegisterType& value)
			{
				return transferPhyRegister(PHYUser::Address, PHYUser::ReadTimeout, PHY::getRegisterNumber<RegisterType>(), value.value, false);
			};

		private:
			static bool transferPhyRegister(uint8_t phyAddress, std::chrono::milliseconds timeout, uint8_t registerAddress, uint16_t& regValue, bool isWrite=false)
			{

				// get only CR bits from MACMIIAR
				static constexpr uint32_t readKeepMask = MdcClockDivider_t::mask().value;
				MdioAddress_t mdioar(ETH->MACMDIOAR & readKeepMask);
				mdioar.set(MdioPhysicalAddress_t(phyAddress));
				mdioar.set(MdioRegDeviceAddress_t(registerAddress));
				mdioar.set(MdioOpCode_t(isWrite ? MdioOpCode::Write : MdioOpCode::Read));
				mdioar.set(MdioAddress::MiiBusy);
				ETH->MACMDIODR = regValue;
				writeSafe(mdioar);
				modm::Timeout timeoutTimer(timeout);
				while (not timeoutTimer.execute())
				{
					if (MdioAddress_t(ETH->MACMDIOAR).none(MdioAddress::MiiBusy))
					{
						// busy flag cleared, read data

						regValue = ETH->MACMDIODR;
						return true;
					}
				};
				return false;
			}

			template <typename Reg_t>
			static void writeSafe(Reg_t reg)
			{
				// wait at least 4 MII clock cycles. worst case 4 X 2.5Mhz (10Mbit) -> 1.6us
				// see f7 errata sheet for reason
				modm::delay(2us);
				if constexpr (std::is_same<Reg_t, MacConfiguration_t>::value)
				{
					ETH->MACCR = reg.value;
				}
				else if constexpr (std::is_same<Reg_t, MacPacketFilteringControl_t>::value)
				{
					ETH->MACPFR = reg.value;
				}
				else if constexpr (std::is_same<Reg_t, TxQueueFlowControl_t>::value)
				{
					ETH->MACTFCR = reg.value;
				}
				else if constexpr (std::is_same<Reg_t, MacInterruptStatus_t>::value)
				{
					ETH->MACISR = reg.value;
				}
				else if constexpr (std::is_same<Reg_t, MacInterruptEnable_t>::value)
				{
					ETH->MACIER = reg.value;
				}
				else if constexpr (std::is_same<Reg_t, DmaMode_t>::value)
				{
					ETH->DMAMR = reg.value;
				}
				else if constexpr (std::is_same<Reg_t, DmaSystemBusMode_t>::value)
				{
					ETH->DMASBMR = reg.value;
				}
				else if constexpr (std::is_same<Reg_t, DmaInterruptStatus_t>::value)
				{
					ETH->DMAISR = reg.value;
				}
				else if constexpr (std::is_same<Reg_t, DmaChannelControl_t>::value)
				{
					ETH->DMACCR = reg.value;
				}
				else if constexpr (std::is_same<Reg_t, DmaChannelTransmitControl_t>::value)
				{
					ETH->DMACTCR = reg.value;
				}
				else if constexpr (std::is_same<Reg_t, DmaChannelReceiveControl_t>::value)
				{
					ETH->DMACRCR = reg.value;
				}
				else if constexpr (std::is_same<Reg_t, MtlTxQueueOperatingMode_t>::value)
				{
					ETH->MTLTQOMR = reg.value;
				}
				else if constexpr (std::is_same<Reg_t, MtlRxQueueOperatingMode_t>::value)
				{
					ETH->MTLRQOMR = reg.value;
				}
				else if constexpr (std::is_same<Reg_t, MdioAddress_t>::value)
				{
					ETH->MACMDIOAR = reg.value;
				}
				else if constexpr (std::is_same<Reg_t, MdioData_t>::value)
				{
					ETH->MACMDIODR = reg.value;
				}
				else
				{
					// will allways assert false since we would never get here otherwise
					static_assert(std::is_same<Reg_t, MacConfiguration_t>::value);
				}
			}
		};

	}

}

#endif // MODM_ETH_HPP
