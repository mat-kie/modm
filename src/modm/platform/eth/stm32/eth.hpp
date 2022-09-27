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

#include <array>
#include <cstring>
#include <modm/architecture/interface.hpp>
#include <modm/math/utils/bit_constants.hpp>
#include <modm/platform/gpio/connector.hpp>
#include <modm/platform/eth/PHYBase.hpp>
#include "../device.hpp"

namespace modm
{
	namespace platform
	{

		/// @ingroup modm_platform_eth
		struct eth
		{
			enum class MediaInterface : uint32_t
			{
				MII = 0x00,
				RMII = SYSCFG_PMC_MII_RMII_SEL
			};

			enum class DuplexMode
			{
				Half = 0,
				Full = 1,
			};

			enum class Speed
			{
				Speed10M = 0,
				Speed100M = 1,
			};

			enum class LinkStatus
			{
				Down = 0,
				Up,
			};

			enum class MacAddressIndex : uint32_t
			{
				Index0 = 0x00000000,
				Index1 = 0x00000008,
				Index2 = 0x00000010,
				Index3 = 0x00000018,
			};

			enum class Interrupt : uint32_t
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
			MODM_FLAGS32(Interrupt);

			enum class InterruptFlags : uint32_t
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
			MODM_FLAGS32(InterruptFlags);
			enum class MacConfiguration : uint32_t
			{
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
				ReceiveEnable = modm::Bit2,
			};
			MODM_FLAGS32(MacConfiguration);

			static constexpr uint32_t MacCrClearMask{0xFF20810F};

			enum class Watchdog : uint32_t
			{
				Enable = 0b0,
				Disable = 0b1,
			};
			using Watchdog_t = modm::Configuration<MacConfiguration_t, Watchdog, 0b1, 23>;

			enum class Jabber : uint32_t
			{
				Enable = 0b0,
				Disable = 0b1,
			};
			using Jabber_t = modm::Configuration<MacConfiguration_t, Jabber, 0b1, 22>;

			enum class InterframeGap : uint32_t
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

			enum class CarrierSense : uint32_t
			{
				Enable = 0b0,
				Disable = 0b1,
			};
			using CarrierSense_t = modm::Configuration<MacConfiguration_t, CarrierSense, 0b1, 16>;

			using Speed_t = modm::Configuration<MacConfiguration_t, Speed, 0b1, 14>;
			using DuplexMode_t = modm::Configuration<MacConfiguration_t, DuplexMode, 0b1, 11>;

			enum class MacFrameFilter : uint32_t
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

			enum class PassControlFrame : uint32_t
			{
				BlockAll = 0b00,
				AllExceptPause = 0b01,
				AllAddressFilterFail = 0b10,
				AllAddressFilterPass = 0b11,
			};
			using PassControlFrame_t = modm::Configuration<MacFrameFilter_t, PassControlFrame, 0b11, 6>;

			enum class MacFlowControl : uint32_t
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

			static constexpr uint32_t MacFcrClearMask{0x0000FF41};

			enum class PauseLowThreshold : uint32_t
			{
				Minus4SlotTimes = 0b00,
				Minus28SlotTimes = 0b01,
				Minus144SlotTimes = 0b10,
				Minus256SlotTimes = 0b11,
			};
			using PauseLowThreshold_t = modm::Configuration<MacFlowControl_t, PauseLowThreshold, 0b11, 4>;

			enum class DmaOperationMode : uint32_t
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

			static constexpr uint32_t DmaOmrClearMask{0xF8DE3F23};

			enum class TransmitThreshold : uint32_t
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
			using TransmitThreshold_t =
				modm::Configuration<DmaOperationMode_t, TransmitThreshold, 0b111, 14>;

			enum class ReceiveThreshold : uint32_t
			{
				Bytes64 = 0b00,
				Bytes32 = 0b01,
				Bytes96 = 0b10,
				Bytes128 = 0b11,
			};
			using ReceiveThreshold_t = modm::Configuration<DmaOperationMode_t, ReceiveThreshold, 0b11, 3>;

			enum class DmaBusMode : uint32_t
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

			enum class BurstLength : uint32_t
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

			enum class Event : uint32_t
			{
				None = 0x00,
				Receive = 0x01,
				Transmit = 0x02,
				Error = 0x04,
			};
			MODM_FLAGS32(Event);

			enum class
				TDes0 : uint32_t
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

			enum class
				CrcControl : uint32_t
			{
				InsertionDisabled = 0b00,
				IpHeaderOnly = 0b01,
				IpHeaderAndPayload = 0b10,
				HardwareCalculated = 0b11
			};
			using CrcControl_t = modm::Configuration<TDes0_t, CrcControl, 0b11, 22>;

			enum class
				RDes0 : uint32_t
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

			enum class
				RDes1 : uint32_t
			{
				DisableIrqOnCompletion = modm::Bit31,
				ReceiveEndOfRing = modm::Bit15,
				SecondAddressChained = modm::Bit14,
			};
			MODM_FLAGS32(RDes1);

			static constexpr uint32_t ReceiveDescriptorFrameLengthMask{0x3fff0000};
			static constexpr uint32_t ReceiveDescriptorFrameLengthShift{16};
			static constexpr uint32_t Buffer1SizeMask{0x00001fff};
			static constexpr uint32_t Buffer2SizeMask{0x1fff0000};
			struct DmaDescriptor
			{
				__IO uint32_t Status;		  /*!< Status */
				uint32_t ControlBufferSize;	  /*!< Control and Buffer1, Buffer2 lengths */
				uint32_t Buffer1Addr;		  /*!< Buffer1 address pointer */
				uint32_t Buffer2NextDescAddr; /*!< Buffer2 or next descriptor address pointer */

				/*!< Enhanced Ethernet DMA PTP Descriptors */
				uint32_t ExtendedStatus; /*!< Extended status for PTP receive descriptor */
				uint32_t Reserved1;		 /*!< Reserved */
				uint32_t TimeStampLow;	 /*!< Time Stamp Low value for transmit and receive */
				uint32_t TimeStampHigh;	 /*!< Time Stamp High value for transmit and receive */
			};
			using DmaDescriptor_t = DmaDescriptor;

			class RxDMADescriptorHandle
			{
				DmaDescriptor_t *dmaDescriptor;
				static inline  RDes0_t receiveStatus{RDes0::CrcError | RDes0::Ipv4HeaderCrcError | RDes0::EthernetFrameType};
			public:
				RxDMADescriptorHandle(DmaDescriptor_t *dd) : dmaDescriptor(dd){};
				inline uint32_t swapBufferPointer(uint32_t newPointer, uint32_t newSize)
				{
					uint32_t old = dmaDescriptor->Buffer1Addr;
					dmaDescriptor->Buffer1Addr = newPointer;
					dmaDescriptor->ControlBufferSize = uint32_t(RDes1::SecondAddressChained) |(newSize & Buffer1SizeMask);
					return old;
				};
				inline bool hasValidEthernetFrame()
				{
					return ((dmaDescriptor->Status & receiveStatus.value) == uint32_t(RDes0::EthernetFrameType)); // TODO
				}

				inline uint32_t getFrameLength()
				{
					return ((dmaDescriptor->Status & ReceiveDescriptorFrameLengthMask) >> ReceiveDescriptorFrameLengthShift) - 4; // Todo
				}
				inline void handBackToDMA()
				{
					dmaDescriptor->Status = uint32_t(RDes0::DmaOwned);
					// set dma ownership
				}
				// Todo: some further status determination abstraction
			};

			class TxDMADescriptorHandle
			{
				DmaDescriptor_t *dmaDescriptor;
				static inline  TDes0_t transmitStatus{
					CrcControl_t(CrcControl::HardwareCalculated) |
					TDes0_t(TDes0::InterruptOnCompletion |
							TDes0::LastSegment |
							TDes0::FirstSegment)};

			public:
				TxDMADescriptorHandle(DmaDescriptor_t *dd) : dmaDescriptor(dd){};
				inline uint32_t swapBufferPointer(uint32_t newPointer, uint32_t newSize)
				{
					uint32_t old = dmaDescriptor->Buffer1Addr;
					dmaDescriptor->Buffer1Addr = newPointer;
					dmaDescriptor->ControlBufferSize = newSize & Buffer1SizeMask;
					return old;
				};

				inline void handBackToDMA()
				{
					dmaDescriptor->Status |= transmitStatus.value | uint32_t(TDes0::DmaOwned);
					// set dma ownership
				}
				// Todo: some further status determination abstraction
			};

			struct ANResult
			{
				bool successful = false;
				eth::DuplexMode mode = eth::DuplexMode::Full;
				eth::Speed speed = eth::Speed::Speed100M;
			};

		protected:
			static constexpr uint32_t
			i(MediaInterface interface)
			{
				return uint32_t(interface);
			}
		};

		/// @ingroup modm_platform_eth
		template <int descriptorTableSize>
		class Eth : public eth
		{
		private:
			using MacAddress = std::array<uint8_t, 6>;
			using MacAddresses = std::array<MacAddress, 4>;

			static inline DuplexMode duplexMode = eth::DuplexMode::Full;
			static inline Speed speed = eth::Speed::Speed100M;
			static inline MacAddresses macAddresses;
			modm_aligned(32) static inline DmaDescriptor_t DmaTxDescriptorTable[descriptorTableSize];
			modm_aligned(32) static inline DmaDescriptor_t DmaRxDescriptorTable[descriptorTableSize];
			static inline DmaDescriptor_t *RxDescriptor = DmaRxDescriptorTable;			  ///< pointer to active descriptor
			static inline DmaDescriptor_t *TxDescriptor = DmaTxDescriptorTable;			  ///< pointer to active descriptor
			static inline DmaDescriptor_t *DmaTxDescriptorToClear = DmaTxDescriptorTable; ///< pointer to active descriptor

		public:
			template <class... Signals>
			static void
			connect();

			template <MediaInterface Interface>
			static inline bool
			initialize(uint8_t priority = 5);

			static void
			configureMac(ANResult negotiated);

			static void
			configureDma();

			static void
			setMacAddress(MacAddressIndex index, uint8_t const *macAddress);

			static void
			configureMacAddresses();

			static void
			start();

			static void
			stop();

			static void
			setDmaTxDescriptorTable(uint32_t address);
			static void
			setDmaRxDescriptorTable(uint32_t address);

			static InterruptFlags
			getInterruptFlags();
			static void
			acknowledgeInterrupt(InterruptFlags_t irq);
			static void
			enableInterrupt(Interrupt_t irq);

			template <class T>
			static bool
			writePhyRegister(uint32_t Address, PHYBase::Register reg, T value);

			template <class T>
			static bool
			readPhyRegister(uint32_t Address, PHYBase::Register reg, T &value);
			/**
			 * @brief check if another RX Framebuffer can be read
			 *
			 * @return true
			 * @return false
			 */
			static bool hasRXFrame();
			/**
			 * @brief get the next RX Framebuffer handle
			 *  this is the RecieveFrame in the procedual IEEE802.3 model
			 * @return DMADescriptorHandle
			 */
			static RxDMADescriptorHandle getRXFrame();

			static bool hasFinishedTXD();
			static TxDMADescriptorHandle getFinishedTXD();

			static bool hasNextTXD();
			static TxDMADescriptorHandle getNextTXD();

			static bool InitTXDescriptorTable();

			static bool
			InitRXDescriptorTable(uint32_t buffer_ptrs[descriptorTableSize], int buffer_size);

		private:
			static inline void writeReg(volatile uint32_t &reg, uint32_t value);
		};

	} // namespace platform
} // namespace modm
#include "eth_impl.hpp"
#endif // MODM_ETH_HPP
