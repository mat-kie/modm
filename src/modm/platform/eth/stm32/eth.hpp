/*
 * Copyright (c) 2020, Mike Wolfram
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

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
			/// @brief  MACCR
			enum class
				MacConfiguration : uint32_t
			{
				CRCStripping = Bit25,
				WatchDogDisable = modm::Bit23,
				JabberDisable = modm::Bit22,
				InterframeGap2 = modm::Bit19,
				InterframeGap1 = modm::Bit18,
				InterframeGap0 = modm::Bit17,
				CarrierSense = modm::Bit16,
				EthernetSpeed = modm::Bit14,
				ReceiveOwn = modm::Bit13,
				LoopBack = modm::Bit12,
				DuplexMode = modm::Bit11,
				Ipv4ChecksumOffLoad = modm::Bit10,
				RetryDisable = modm::Bit9,
				AutomaticPad = modm::Bit7,
				BackOffLimit1 = modm::Bit6,
				BackOffLimit0 = modm::Bit5,
				DeferalCheck = modm::Bit4,
				TransmitterEnable = modm::Bit3,
				ReceiveEnable = modm::Bit2
			};
			MODM_FLAGS32(MacConfiguration);

			//static constexpr uint32_t MacCrClearMask{0xFF20810F}; not quite the same TE RE and a few others are also set
			static constexpr uint32_t MacCrReservedBits{Bit0 | Bit1 | Bit8 | Bit15 | Bit20 | Bit21 | Bit24 | Bit26 | Bit27 | Bit28 | Bit29 | Bit30 | Bit31};


			enum class
				Watchdog : uint32_t
			{
				Enable = 0b0,
				Disable = 0b1,
			};
			using Watchdog_t = modm::Configuration<MacConfiguration_t, Watchdog, 0b1, 23>;

			enum class
				Jabber : uint32_t
			{
				Enable = 0b0,
				Disable = 0b1,
			};
			using Jabber_t = modm::Configuration<MacConfiguration_t, Jabber, 0b1, 22>;

			enum class
				InterframeGap : uint32_t
			{
				Gap96BitTimes = 0b000,
				Gap88BitTimes = 0b001,
				Gap80BitTimes = 0b010,
				Gap72BitTimes = 0b011,
				Gap64BitTimes = 0b100,
				Gap56BitTimes = 0b101,
				Gap48BitTimes = 0b110,
				Gap40BitTimes = 0b111,
			};
			using InterframeGap_t = modm::Configuration<MacConfiguration_t, InterframeGap, 0b111, 17>;

			enum class
				CarrierSense : uint32_t
			{
				Enable = 0b0,
				Disable = 0b1,
			};
			enum class
				DuplexMode
			{
				Half = 0,
				Full = 1,
			};

			enum class
				Speed
			{
				Speed10M = 0,
				Speed100M = 1,
			};

			enum class
				LinkStatus
			{
				Down = 0,
				Up,
			};
			using CarrierSense_t = modm::Configuration<MacConfiguration_t, CarrierSense, 0b1, 16>;

			using Speed_t = modm::Configuration<MacConfiguration_t, Speed, 0b1, 14>;
			using DuplexMode_t = modm::Configuration<MacConfiguration_t, DuplexMode, 0b1, 11>;

			/// @brief  MACFFR
			enum class
				MacFrameFilter : uint32_t
			{
				ReceiveAll = modm::Bit31,
				HashOrPerfect = modm::Bit10,
				SourceAddress = modm::Bit9,
				SourceAddressInverse = modm::Bit8,
				PassControl1 = modm::Bit7,
				PassControl0 = modm::Bit6,
				BroadcastDisable = modm::Bit5,
				PassAllMulticast = modm::Bit4,
				DestinationAddressInverse = modm::Bit3,
				HashMulticast = modm::Bit2,
				HasUnicast = modm::Bit1,
				PromiscuousMode = modm::Bit0,
			};
			MODM_FLAGS32(MacFrameFilter);
			static constexpr uint32_t MacFFRReservedBits{Bit11 | Bit12 | Bit13 | Bit14 | Bit15 | Bit16 | Bit17 | Bit18 | Bit19 | Bit20 | Bit21 | Bit22 | Bit23 | Bit24 | Bit25 | Bit26 | Bit27 | Bit28 | Bit29 | Bit30};
			enum class
				PassControlFrame : uint32_t
			{
				BlockAll = 0b00,
				AllExceptPause = 0b01,
				AllAddressFilterFail = 0b10,
				AllAddressFilterPass = 0b11,
			};
			using PassControlFrame_t = modm::Configuration<MacFrameFilter_t, PassControlFrame, 0b11, 6>;

			enum class MacMIIAddress : uint32_t
			{
				MIIBusy = Bit0,
				MIIWrite = Bit1,
				CR0 = Bit2,
				CR1 = Bit3,
				CR2 = Bit4,
				MR0 = Bit6,
				MR1 = Bit7,
				MR2 = Bit8,
				MR3 = Bit9,
				MR4 = Bit10,
				PA0 = Bit11,
				PA1 = Bit12,
				PA2 = Bit13,
				PA3 = Bit14,
				PA4 = Bit15,
			};
			MODM_FLAGS32(MacMIIAddress);
			typedef Value<MacMIIAddress_t, 5, 11> PHYAddress_t;
			typedef Value<MacMIIAddress_t, 5, 6> MIIRegister_t;

			enum class MDCClockRange:uint32_t{
				Div42=0,
				Div62=1,
				Div16=2,
				Div26=3,
				Div102=4
			};
			typedef Configuration<MacMIIAddress_t, MDCClockRange, 0b111, 2> MDCClockRange_t;

			/// @brief MACFCR
			enum class
				MacFlowControl : uint32_t
			{
				PauseTime15 = modm::Bit31,
				PauseTime14 = modm::Bit30,
				PauseTime13 = modm::Bit29,
				PauseTime12 = modm::Bit28,
				PauseTime11 = modm::Bit27,
				PauseTime10 = modm::Bit26,
				PauseTime9 = modm::Bit25,
				PauseTime8 = modm::Bit24,
				PauseTime7 = modm::Bit23,
				PauseTime6 = modm::Bit22,
				PauseTime5 = modm::Bit21,
				PauseTime4 = modm::Bit20,
				PauseTime3 = modm::Bit19,
				PauseTime2 = modm::Bit18,
				PauseTime1 = modm::Bit17,
				PauseTime0 = modm::Bit16,
				ZeroQuantaPauseDisable = modm::Bit7,
				PauseLowThreshold1 = modm::Bit5,
				PauseLowThreshold0 = modm::Bit4,
				UnicastPauseDetect = modm::Bit3,
				ReceiveFlowControlEnable = modm::Bit2,
				TransmitFlowControlEnable = modm::Bit1,
				FlowControlBusy = modm::Bit0,
			};
			MODM_FLAGS32(MacFlowControl);

			//static constexpr uint32_t MacFcrClearMask{0x0000FF41}; Bit 0 also set
			static constexpr uint32_t MacFCRReservedBits{Bit6 | Bit8 | Bit9 | Bit10 | Bit11 | Bit12 | Bit13 | Bit14 | Bit15};

			enum class
				PauseLowThreshold : uint32_t
			{
				Minus4SlotTimes = 0b00,
				Minus28SlotTimes = 0b01,
				Minus144SlotTimes = 0b10,
				Minus256SlotTimes = 0b11,
			};
			using PauseLowThreshold_t = modm::Configuration<MacFlowControl_t, PauseLowThreshold, 0b11, 4>;

			enum class MacVLANTag : uint32_t
			{
				Tag12BitComparison = modm::Bit16
			};
			MODM_FLAGS32(MacVLANTag);

			enum class MacInterruptStatus : uint32_t
			{
				PMTS = Bit3,			///< PMT status
				MMCS = Bit4,			///< MMC status
				MMCRreceive = Bit5,		///< MMC status
				MMCTransmit = Bit6,
				TimestampTrigger = Bit9
			};
			MODM_FLAGS32(MacInterruptStatus);

			enum class MacInterruptMask : uint32_t
			{
				PMT = Bit3,
				TimestampTrigger = Bit9
			};
			MODM_FLAGS32(MacInterruptMask);

			enum class TimeStampControlRegister : uint32_t
			{
				PTPMacFrameFilterEnable = Bit18,
				ClockNodeType1 = Bit17,
				ClockNodeType0 = Bit16,
				TimeStampMasterMode = Bit15,
				TimeStampEventMsg = Bit14,
				TimeStampIPv4 = Bit13,
				TimeStampIPv6 = Bit12,
				TimeStamp802_3 = Bit11,
				SnoopPTPV2 = Bit10,
				SubSecondRollover = Bit9,
				TimeStampAllFrames = Bit8,
				TimeStampAddendRegisterUpdate = Bit5,
				TimeStampInterruptTrigger = Bit4,
				TimeStampSystemtTimeUpdate = Bit3,
				TimeStampSystemTimeInitialize = Bit2,
				TimeStampFineUpdate = Bit1,
				TimeStampEnable = Bit0
			};
			MODM_FLAGS32(TimeStampControlRegister);

			enum class ClockType : uint32_t
			{
				Ordinary = 0b00,
				Boundary = 0b01,
				E2ETransparent = 0b10,
				P2PTransparent = 0b11
			};
			typedef modm::Configuration<TimeStampControlRegister_t, ClockType, 0b11, 16> ClockType_t;


			enum class
				DmaOperationMode : uint32_t
			{
				DropCrcErrorFrameDisable = modm::Bit26,
				ReceiveStoreAndForward = modm::Bit25,
				DisableFlushReceivedFrames = modm::Bit24,
				TransmitStoreAndForward = modm::Bit21,
				FlushTransmitFifo = modm::Bit20,
				TransmitThreshold2 = modm::Bit16,
				TransmitThreshold1 = modm::Bit15,
				TransmitThreshold0 = modm::Bit14,
				StartTransmission = modm::Bit13,
				ForwardErrorFrames = modm::Bit7,
				ForwardUndersizedGoodFrames = modm::Bit6,
				ReceiveThreshold1 = modm::Bit4,
				ReceiveThreshold0 = modm::Bit3,
				OperateOnSecondFrame = modm::Bit2,
				StartReceive = modm::Bit1,
			};
			MODM_FLAGS32(DmaOperationMode);

			static constexpr uint32_t DmaOmrReservedBits{Bit0 | Bit5 | Bit8 | Bit9 | Bit10 | Bit11 | Bit12 | Bit17 | Bit18 | Bit19 | Bit22 | Bit23 | Bit27 | Bit28 | Bit29 | Bit30 | Bit31 };

			enum class
				TransmitThreshold : uint32_t
			{
				Bytes64 = 0b000,
				Bytes128 = 0b001,
				Bytes192 = 0b010,
				Bytes256 = 0b011,
				Bytes40 = 0b100,
				Bytes32 = 0b101,
				Bytes24 = 0b110,
				Bytes16 = 0b111,
			};
			using TransmitThreshold_t = modm::Configuration<DmaOperationMode_t, TransmitThreshold, 0b111, 14>;

			enum class
				ReceiveThreshold : uint32_t
			{
				Bytes64 = 0b00,
				Bytes32 = 0b01,
				Bytes96 = 0b10,
				Bytes128 = 0b11,
			};
			using ReceiveThreshold_t = modm::Configuration<DmaOperationMode_t, ReceiveThreshold, 0b11, 3>;

			enum class
				DmaBusMode : uint32_t
			{
				MixedBurst = modm::Bit26,
				AddressAlignedBeats = modm::Bit25,
				PblModeX4 = modm::Bit24,
				UseSeparatePbl = modm::Bit23,
				RxDmaPbl5 = modm::Bit22,
				RxDmaPbl4 = modm::Bit21,
				RxDmaPbl3 = modm::Bit20,
				RxDmaPbl2 = modm::Bit19,
				RxDmaPbl1 = modm::Bit18,
				RxDmaPbl0 = modm::Bit17,
				FixedBurst = modm::Bit16,
				RxTxPriorityRatio1 = modm::Bit15,
				RxTxPriorityRatio0 = modm::Bit14,
				Pbl5 = modm::Bit13,
				Pbl4 = modm::Bit12,
				Pbl3 = modm::Bit11,
				Pbl2 = modm::Bit10,
				Pbl1 = modm::Bit9,
				Pbl0 = modm::Bit8,
				EnhancedDescFormat = modm::Bit7,
				DescriptorSkipLength4 = modm::Bit6,
				DescriptorSkipLength3 = modm::Bit5,
				DescriptorSkipLength2 = modm::Bit4,
				DescriptorSkipLength1 = modm::Bit3,
				DescriptorSkipLength0 = modm::Bit2,
				DmaArbitration = modm::Bit1,
				SoftwareReset = modm::Bit0,
			};
			MODM_FLAGS32(DmaBusMode);

			static constexpr uint32_t DmaBmrReservedBits{Bit27 | Bit28 | Bit29 | Bit30 | Bit31};

			enum class
				BurstLength : uint32_t
			{
				Length1Beat = 0b000001,
				Length2Beats = 0b000010,
				Length4Beats = 0b000100,
				Length8Beats = 0b001000,
				Length16Beats = 0b010000,
				Length32Beats = 0b100000,
			};
			using BurstLength_t = modm::Configuration<DmaBusMode_t, BurstLength, 0b111111, 8>;
			using RxDmaBurstLength_t = modm::Configuration<DmaBusMode_t, BurstLength, 0b111111, 17>;






			enum class
				MediaInterface : uint32_t
			{
				MII = 0x00,
				RMII = SYSCFG_PMC_MII_RMII_SEL
			};

			enum class
				MacAddressIndex : uint32_t
			{
				Index0 = 0x00000000,
				Index1 = 0x00000008,
				Index2 = 0x00000010,
				Index3 = 0x00000018,
			};

			enum class
				DmaInterruptEnable : uint32_t
			{
				NormalIrqSummary = modm::Bit16,
				AbnormalIrqSummary = modm::Bit15,
				EarlyReceive = modm::Bit14,
				FatalBusError = modm::Bit13,
				EarlyTransmit = modm::Bit10,
				ReceiveWatchdog = modm::Bit9,
				ReceiveStopped = modm::Bit8,
				ReceiveBufferUnavailable = modm::Bit7,
				Receive = modm::Bit6,
				TransmitUnderflow = modm::Bit5,
				ReceiveOverflow = modm::Bit4,
				TransmitJabberTimeout = modm::Bit3,
				TransmitBufferUnavailable = modm::Bit2,
				TransmitStopped = modm::Bit1,
				Transmit = modm::Bit0,
			};
			MODM_FLAGS32(DmaInterruptEnable);

			enum class
				DmaStatus : uint32_t
			{
				TimeStampTrigger = modm::Bit29,
				Pmt = modm::Bit28,
				Mmc = modm::Bit27,
				ErrorBitStatus2 = modm::Bit25,
				ErrorBitStatus1 = modm::Bit24,
				ErrorBitStatus0 = modm::Bit23,
				TransmitProcessState2 = modm::Bit22,
				TransmitProcessState1 = modm::Bit21,
				TransmitProcessState0 = modm::Bit20,
				ReceiveProcessState2 = modm::Bit19,
				ReceiveProcessState1 = modm::Bit18,
				ReceiveProcessState0 = modm::Bit17,
				NormalIrqSummary = modm::Bit16,
				AbnormalIrqSummary = modm::Bit15,
				EarlyReceive = modm::Bit14,
				FatalBusError = modm::Bit13,
				EarlyTransmit = modm::Bit10,
				ReceiveWatchdog = modm::Bit9,
				ReceiveStopped = modm::Bit8,
				ReceiveBufferUnavailable = modm::Bit7,
				Receive = modm::Bit6,
				TransmitUnderflow = modm::Bit5,
				ReceiveOverflow = modm::Bit4,
				TransmitJabberTimeout = modm::Bit3,
				TransmitBufferUnavailable = modm::Bit2,
				TransmitStopped = modm::Bit1,
				Transmit = modm::Bit0,
			};
			MODM_FLAGS32(DmaStatus);

			enum class
				Event : uint32_t
			{
				None = 0x00,
				Receive = 0x01,
				Transmit = 0x02,
				Error = 0x04,
			};
			MODM_FLAGS32(Event);

			////////////////////////
			enum class TDes0 : uint32_t
			{
				DmaOwned = modm::Bit31,
				InterruptOnCompletion = modm::Bit30,
				LastSegment = modm::Bit29,
				FirstSegment = modm::Bit28,
				DisableCrc = modm::Bit27,
				DisablePadding = modm::Bit26,
				TransmitTimestampEnable = modm::Bit25,
				CrcCtrl1 = modm::Bit23,
				CrcCtrl0 = modm::Bit22,
				TransmitEndOfRing = modm::Bit21,
				SecondAddressChained = modm::Bit20,
				TransmitTimestampStatus = modm::Bit17,
				IpHeaderError = modm::Bit16,
				ErrorSummary = modm::Bit15,
				JabberTimeoout = modm::Bit14,
				FrameFlushed = modm::Bit13,
				IpPayloadError = modm::Bit12,
				LossOfCarrier = modm::Bit11,
				NoCarrier = modm::Bit10,
				LateCollision = modm::Bit9,
				ExcessiveCollision = modm::Bit8,
				VLanFrame = modm::Bit7,
				CollisionCount3 = modm::Bit6,
				CollisionCount2 = modm::Bit5,
				CollisionCount1 = modm::Bit4,
				CollisionCount0 = modm::Bit3,
				ExcessiveDeferral = modm::Bit2,
				UnderflowError = modm::Bit1,
				DeferredBit = modm::Bit0,
			};

			MODM_FLAGS32(TDes0);

			enum class CrcControl : uint32_t
			{
				InsertionDisabled = 0b00,
				IpHeaderOnly = 0b01,
				IpHeaderAndPayload = 0b10,
				HardwareCalculated = 0b11
			};
			using CrcControl_t = modm::Configuration<TDes0_t, CrcControl, 0b11, 22>;

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
			typedef Value<TDes1_t, 13, 0> TxBuffer1Size_t;
			typedef Value<TDes1_t, 13, 16> TxBuffer2Size_t;

			enum class RDes0 : uint32_t
			{
				DmaOwned = modm::Bit31,
				DAFilterFail = modm::Bit30,
				ErrorSummary = modm::Bit15,
				DescriptorError = modm::Bit14,
				SAFilterFail = modm::Bit13,
				LengthError = modm::Bit12,
				OverflowError = modm::Bit11,
				VLanFrame = modm::Bit10,
				FirstSegment = modm::Bit9,
				LastSegment = modm::Bit8,
				Ipv4HeaderCrcError = modm::Bit7,
				LateCollision = modm::Bit6,
				EthernetFrameType = modm::Bit5,
				ReceiveWatchdogTimeout = modm::Bit4,
				ReceiveError = modm::Bit3,
				DribbleBitError = modm::Bit2,
				CrcError = modm::Bit1,
				PayloadCrcError = modm::Bit0,
			};
			MODM_FLAGS32(RDes0);
			typedef Value<RDes0_t, 14, 16> RxFrameLength_t;

			enum class RDes1 : uint32_t
			{
				DisableIrqOnCompletion = modm::Bit31,
				ReceiveEndOfRing = modm::Bit15,
				SecondAddressChained = modm::Bit14,
				RBS1_0 = Bit0,
				RBS1_1 = Bit1,
				RBS1_2 = Bit2,
				RBS1_3 = Bit3,
				RBS1_4 = Bit4,
				RBS1_5 = Bit5,
				RBS1_6 = Bit6,
				RBS1_7 = Bit7,
				RBS1_8 = Bit8,
				RBS1_9 = Bit9,
				RBS1_10 = Bit10,
				RBS1_11 = Bit11,
				RBS1_12 = Bit12,
				RBS2_0 = Bit16,
				RBS2_1 = Bit17,
				RBS2_2 = Bit18,
				RBS2_3 = Bit19,
				RBS2_4 = Bit20,
				RBS2_5 = Bit21,
				RBS2_6 = Bit22,
				RBS2_7 = Bit23,
				RBS2_8 = Bit24,
				RBS2_9 = Bit25,
				RBS2_10 = Bit26,
				RBS2_11 = Bit27,
				RBS2_12 = Bit28,
			};
			MODM_FLAGS32(RDes1);
			typedef Value<RDes1_t, 13, 0> RxBuffer1Size_t;
			typedef Value<RDes1_t, 13, 16> RxBuffer2Size_t;

			enum class RDes4 : uint32_t
			{
				PTPVersion = Bit13,
				PTPFrameType = Bit12,
				PTPMessageType0 = Bit8,
				PTPMessageType1 = Bit9,
				PTPMessageType2 = Bit10,
				PTPMessageType3 = Bit11,
				IPv6Packet = Bit7,
				IPv4Packet = Bit6,
				IPChecksumBypassed = Bit5,
				IPPayloadError = Bit4,
				IPHeaderError = Bit3,
				IPPayloadType0 = Bit0,
				IPPayloadType1 = Bit1,
				IPPayloadType2 = Bit2,

			};
			MODM_FLAGS32(RDes4);

			static constexpr uint32_t ReceiveDescriptorFrameLengthMask{0x3fff0000};
			static constexpr uint32_t ReceiveDescriptorFrameLengthShift{16};
			static constexpr uint32_t Buffer1SizeMask{0x00001fff};
			static constexpr uint32_t Buffer2SizeMask{0x1fff0000};

			struct RxDmaDescriptor
			{
				__IO uint32_t Status;		  /*!< Status */
				uint32_t ControlBufferSize;	  /*!< Control and Buffer1, Buffer2 lengths */
				uint32_t Buffer1Addr;		  /*!< Buffer1 address pointer */
				uint32_t Buffer2NextDescAddr; /*!< Buffer2 or next descriptor address pointer */

				/*!< Enhanced Ethernet DMA PTP Descriptors */
				uint32_t ExtendedStatus; /*!< Extended status for PTP receive descriptor */
				uint32_t Reserved1;		/*!< Reserved */
				uint32_t TimeStampLow;	/*!< Time Stamp Low value for transmit and receive */
				uint32_t TimeStampHigh; /*!< Time Stamp High value for transmit and receive */
			} modm_packed;

			struct TxDmaDescriptor
			{
				__IO uint32_t Status;		  /*!< Status */
				uint32_t BufferSize;			  /*!< Control and Buffer1, Buffer2 lengths */
				uint32_t Buffer1Addr;		  /*!< Buffer1 address pointer */
				uint32_t Buffer2NextDescAddr; /*!< Buffer2 or next descriptor address pointer */

				/*!< Enhanced Ethernet DMA PTP Descriptors */
				uint32_t Reserved0;		/*!< Extended status for PTP receive descriptor */
				uint32_t Reserved1;		/*!< Reserved */
				uint32_t TimeStampLow;	/*!< Time Stamp Low value for transmit and receive */
				uint32_t TimeStampHigh; /*!< Time Stamp High value for transmit and receive */
			} modm_packed;
			///////////////////////

		protected:
			static constexpr uint32_t i(MediaInterface interface) { return uint32_t(interface); }
		};

		/// @ingroup modm_platform_eth
		/// TODO:
		///   - setting/resetting mac adresses in dest. address filter
		///   - setting/resetting mac adresses in dest. address filter
		class Eth : public eth
		{

			struct IsrCallbacks_t {

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
				SYSCFG->PMC &= ~(SYSCFG_PMC_MII_RMII_SEL);
				SYSCFG->PMC |= uint32_t(Interface);

				if( not softReset()) return false;

				/* Configure SMI clock range */
				MacMIIAddress_t miiar(ETH->MACMIIAR & ETH_MACMIIAR_CR_Msk);
				if (SystemCoreClock >= 20_MHz and SystemCoreClock < 35_MHz)
					miiar |= MDCClockRange_t(MDCClockRange::Div16);
				else if (SystemCoreClock >= 35_MHz and SystemCoreClock < 60_MHz)
					miiar |= MDCClockRange_t(MDCClockRange::Div26);
				else if (SystemCoreClock >= 60_MHz and SystemCoreClock < 100_MHz)
					miiar |= MDCClockRange_t(MDCClockRange::Div42);
				else if (SystemCoreClock >= 100_MHz and SystemCoreClock < 150_MHz)
					miiar |= MDCClockRange_t(MDCClockRange::Div62);
				else if (SystemCoreClock >= 150_MHz)
					miiar |= MDCClockRange_t(MDCClockRange::Div102);

				writeSafe(miiar);


				return true;
			}

			static void
			configureMac(MacConfiguration_t cfg, MacFrameFilter_t filter_cfg, MacFlowControl_t fc_cfg)
			{


				MacConfiguration_t maccr(ETH->MACCR & MacCrReservedBits);
				maccr |= cfg;
				writeSafe(maccr);
				MacFlowControl_t macfcr(ETH->MACFCR & MacFCRReservedBits);
				macfcr |= fc_cfg;
				writeSafe(macfcr);

				MacFrameFilter_t macffr(ETH->MACFFR & MacFFRReservedBits);
				macffr |= filter_cfg;
				writeSafe(macffr);

				// no VLAN support for now
				writeSafe(MacVLANTag_t{0});
			}

			static bool softReset(){
				/* Ethernet Software reset */
				/* Set the SWR bit: resets all MAC subsystem internal registers and logic */
				/* After reset all the registers holds their respective reset values */
				ETH->DMABMR |= uint32_t(DmaBusMode::SoftwareReset);

				/* Wait for software reset */
				/* Note: The SWR is not performed if the ETH_RX_CLK or the ETH_TX_CLK are
				 * not available, please check your external PHY or the IO configuration */
				auto timeout = modm::PreciseClock::now() + 4ms;
				while (modm::PreciseClock::now()<=timeout)
				{
					if(DmaBusMode_t(ETH->DMABMR).none(DmaBusMode::SoftwareReset))
					{
						return true;
					}

				}
				/// NOTE: If the program gets here check MII/RMII value.
				return false;


			}

			static void
			configureDma(DmaBusMode_t busMode, DmaOperationMode_t operationMode)
			{

				DmaOperationMode_t dmaomr(ETH->DMAOMR & DmaOmrReservedBits);
				dmaomr |= operationMode;
				writeSafe(dmaomr);

				DmaBusMode_t dmabmr(ETH->DMABMR & DmaBmrReservedBits);
				//dmabmr |= DmaBusMode::AddressAlignedBeats
				//		| DmaBusMode::FixedBurst
				//		| DmaBusMode::EnhancedDescFormat
				//		| DmaBusMode::UseSeparatePbl
				//		| BurstLength_t(BurstLength::Length32Beats)
				//		| RxDmaBurstLength_t(BurstLength::Length32Beats);
				dmabmr |= busMode;
				writeSafe(dmabmr);

			}

			/// @brief set the corresponding mac filter, or clear it by passing nullptr
			static void
			setMacFilter(MacAddressIndex index, uint8_t const *macAddress = nullptr)
			{
				if(macAddress != nullptr){
					uint32_t tmp_register = (macAddress[5] << 8) | macAddress[4];
					if(index != MacAddressIndex::Index0){
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
				// transmission enable
				// reception enable
				MacConfiguration_t maccr(ETH->MACCR);
				maccr |= MacConfiguration::TransmitterEnable | MacConfiguration::ReceiveEnable;
				writeSafe(maccr);

				// flush transmission fifo
				// DMA transmission enable
				// DMA reception enable
				DmaOperationMode_t dmaomr(ETH->DMAOMR);
				dmaomr |= DmaOperationMode::FlushTransmitFifo | DmaOperationMode::StartTransmission | DmaOperationMode::StartReceive;
				writeSafe(dmaomr);
			};
			static void
			stop()
			{
				// DMA transmission disable
				// DMA reception disable
				DmaOperationMode_t dmaomr(ETH->DMAOMR);
				dmaomr.reset(DmaOperationMode::StartTransmission);
				dmaomr.reset(DmaOperationMode::StartReceive);
				writeSafe(dmaomr);

				// reception disable
				MacConfiguration_t maccr(ETH->MACCR);
				maccr.reset(MacConfiguration::ReceiveEnable);
				writeSafe(maccr);

				// flush transmission fifo
				dmaomr.value = ETH->DMAOMR;
				dmaomr.set(DmaOperationMode::FlushTransmitFifo);
				writeSafe(dmaomr);

				// transmission disable
				maccr.value = ETH->MACCR;
				maccr.reset(MacConfiguration::TransmitterEnable);
				writeSafe(maccr);
			};

			static void
			setDmaTxDescriptorTable(TxDmaDescriptor *address)
			{
				writeSafe(address);
			}
			static void
			setDmaRxDescriptorTable(RxDmaDescriptor *address)
			{
				writeSafe(address);
			}

			static DmaStatus_t
			getInterruptFlags()
			{
				return DmaStatus_t(ETH->DMASR);
			}
			static void
			acknowledgeInterrupt(DmaStatus_t irq)
			{
				// set only the bits you want to clear!
				// using an |= here would clear other fields as well
				ETH->DMASR = irq.value;
			}
			static void
			enableInterrupt(DmaInterruptEnable irq, notificationCB_t callback);
			// enable the mac interrupt irq
			static void
			enableInterrupt(MacInterruptMask irq, notificationCB_t callback);

			static void NotifyFromISR(DmaStatus s);

			template <class PHYUser, class RegisterType>
			static bool
			writePhyRegister(RegisterType value)
			{
				uint32_t reg;
				if constexpr(std::is_same<RegisterType, PHY::SR_t>::value)
				{
					reg = uint32_t(PHY::Register::SR);
				}else if constexpr(std::is_same<RegisterType, PHY::CR_t>::value)
				{
					reg = uint32_t(PHY::Register::CR);
				}
				else{
					static_assert(std::is_same<RegisterType, PHY::SR_t>::value);
					return false;
				}
				// get only CR bits from MACMIIAR
				uint32_t tmp = ETH->MACMIIAR & ETH_MACMIIAR_CR_Msk;
				tmp |= (PHYUser::Address << 11) & ETH_MACMIIAR_PA;
				tmp |= (reg << 6) & ETH_MACMIIAR_MR;
				tmp |= ETH_MACMIIAR_MW;
				tmp |= ETH_MACMIIAR_MB;

				ETH->MACMIIDR = value.value;
				ETH->MACMIIAR = tmp;


				modm::Timeout timeout(PHYUser::WriteTimeout);
				while (not timeout.execute())
				{
					if ((ETH->MACMIIAR & ETH_MACMIIAR_MB) == 0)
						return true;
				}

				return false;
			}
			template <class PHYUser, class RegisterType>
			static bool
			readPhyRegister(RegisterType &value)
			{
				uint32_t reg;
				if constexpr(std::is_same<RegisterType, PHY::SR_t>::value)
				{
					reg = uint32_t(PHY::Register::SR);
				}else if constexpr(std::is_same<RegisterType, PHY::CR_t>::value)
				{
					reg = uint32_t(PHY::Register::CR);
				}
				else{
					static_assert(std::is_same<RegisterType, PHY::SR_t>::value);
					return false;
				}
				// get only CR bits from MACMIIAR
				uint32_t tmp = ETH->MACMIIAR & ETH_MACMIIAR_CR_Msk;
				tmp |= (PHYUser::Address << 11) & ETH_MACMIIAR_PA;
				tmp |= (reg << 6) & ETH_MACMIIAR_MR;
				tmp &= ~ETH_MACMIIAR_MW;
				tmp |= ETH_MACMIIAR_MB;

				ETH->MACMIIAR = tmp;

				modm::Timeout timeout(PHYUser::ReadTimeout);
				while (not timeout.execute())
				{
					if ((ETH->MACMIIAR & ETH_MACMIIAR_MB) == 0)
					{
						// busy flag cleared, read data
						value.value = ETH->MACMIIDR;
						return true;
					}
				};
				return false;
			};
		private:
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
				else if constexpr (std::is_same<Reg_t, MacFlowControl_t>::value)
				{
					ETH->MACFCR = reg.value;
				}
				else if constexpr (std::is_same<Reg_t, MacFrameFilter_t>::value)
				{
					ETH->MACFFR = reg.value;
				}
				else if constexpr (std::is_same<Reg_t, DmaBusMode_t>::value)
				{
					ETH->DMABMR = reg.value;
				}
				else if constexpr (std::is_same<Reg_t, DmaOperationMode_t>::value)
				{
					ETH->DMAOMR = reg.value;
				}
				else if constexpr (std::is_same<Reg_t, MacVLANTag_t>::value)
				{
					ETH->MACVLANTR = reg.value;
				}
				else if constexpr (std::is_same<Reg_t, MacMIIAddress_t>::value)
				{
					ETH->MACMIIAR = reg.value;
				}
				else if constexpr (std::is_same<Reg_t, MacInterruptMask_t>::value)
				{
					ETH->MACIMR = reg.value;
				}else if constexpr (std::is_same<Reg_t, TxDmaDescriptor*>::value)
				{
					ETH->DMATDLAR = (uint32_t)reg;
				}
				else if constexpr (std::is_same<Reg_t, RxDmaDescriptor*>::value)
				{
					ETH->DMARDLAR = (uint32_t)reg;
				}
				else
				{
					// will allways assert false since we would never get here otherwise
					static_assert(std::is_same<Reg_t, MacConfiguration_t>::value);
				}

			}



			static inline DuplexMode duplexMode = eth::DuplexMode::Full;
			static inline Speed speed = eth::Speed::Speed100M;
			static inline LinkStatus linkStatus = eth::LinkStatus::Down;

			using MacAddress = std::array<uint8_t, 6>;
			using MacAddresses = std::array<MacAddress, 4>;
			static inline MacAddresses macAddresses;
		};

	}

}

#endif // MODM_ETH_HPP
