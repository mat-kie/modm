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
#include <array>
#include <cstring>
#include <modm/driver/ethernet/PHYInterface.hpp>
#include "PHYBase.hpp"
#define ADJ_FREQ_BASE_ADDEND 0x3B309D72
#define ADJ_FREQ_BASE_INCREMENT 43

#define ETH_PTP_PositiveTime ((uint32_t)0x00000000) /*!< Positive time value */
#define ETH_PTP_NegativeTime ((uint32_t)0x80000000) /*!< Negative time value */
#define IS_ETH_PTP_TIME_SIGN(SIGN) (((SIGN) == ETH_PTP_PositiveTime) || \
									((SIGN) == ETH_PTP_NegativeTime))

namespace modm::platform
{

	/// @ingroup modm_platform_eth
	struct eth
	{
		/*struct EthTimeStamp
		{
			int32_t seconds;
			int32_t nanoseconds;

			static EthTimeStamp fromRegs(uint32_t hiReg, uint64_t lowreg)
			{
				// 2^32 / subsec increment = number of increments per second
				// nsec = subsec * 1e9 / 2^31;
				// subsec * 1e9 >>32
				lowreg *= 1'000'000'000ULL;
				return EthTimeStamp{
					.seconds = int32_t(hiReg),
					.nanoseconds = int32_t(lowreg >> 31) // divide by 2^31;
				};
			};
			uint32_t toSeconds()
			{
				return seconds;
			}
			uint32_t toSubSecond()
			{
				// the subsecond register rolls over every second
				// so we have subseconds = nanoseconds /1e9 * 2^31
				// since we have integer math we first multiply by 2^31
				uint64_t x = nanoseconds << 31;
				// then we divide.
				return uint32_t(x / 1'000'000'000ULL);
			}
		};*/

		enum class
			MediaInterface : uint32_t
		{
			MII = 0x00,
			RMII = SYSCFG_PMC_MII_RMII_SEL
		};

		enum class
			LinkStatus
		{
			Down = 0,
			Up,
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
			Interrupt : uint32_t
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

		enum class
			InterruptFlags : uint32_t
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
			Event : uint32_t
		{
			None = 0x00,
			Receive = 0x01,
			Transmit = 0x02,
			Error = 0x04,
		};
		MODM_FLAGS32(Event);

	protected:
		static constexpr uint32_t i(MediaInterface interface) { return uint32_t(interface); }
	};

	/// @ingroup modm_platform_eth
	class Eth : public eth
	{
		enum class
			MacConfiguration : uint32_t
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
		using CarrierSense_t = modm::Configuration<MacConfiguration_t, CarrierSense, 0b1, 16>;

		using Speed_t = modm::Configuration<MacConfiguration_t, PHYInterface::Speed, 0b1, 14>;
		using DuplexMode_t = modm::Configuration<MacConfiguration_t, PHYInterface::DuplexMode, 0b1, 11>;

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

		enum class
			PassControlFrame : uint32_t
		{
			BlockAll = 0b00,
			AllExceptPause = 0b01,
			AllAddressFilterFail = 0b10,
			AllAddressFilterPass = 0b11,
		};
		using PassControlFrame_t = modm::Configuration<MacFrameFilter_t, PassControlFrame, 0b11, 6>;

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

		static constexpr uint32_t MacFcrClearMask{0x0000FF41};

		enum class
			PauseLowThreshold : uint32_t
		{
			Minus4SlotTimes = 0b00,
			Minus28SlotTimes = 0b01,
			Minus144SlotTimes = 0b10,
			Minus256SlotTimes = 0b11,
		};
		using PauseLowThreshold_t = modm::Configuration<MacFlowControl_t, PauseLowThreshold, 0b11, 4>;

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

		static constexpr uint32_t DmaOmrClearMask{0xF8DE3F23};

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

			/* Ethernet Software reset */
			/* Set the SWR bit: resets all MAC subsystem internal registers and logic */
			/* After reset all the registers holds their respective reset values */
			ETH->DMABMR |= DmaBusMode_t(DmaBusMode::SoftwareReset | DmaBusMode::EnhancedDescFormat).value;

			/* Wait for software reset */
			/* Note: The SWR is not performed if the ETH_RX_CLK or the ETH_TX_CLK are
			 * not available, please check your external PHY or the IO configuration */
			int timeout = 1'000; // max 1ms
			while ((DmaBusMode(ETH->DMABMR) & DmaBusMode_t(DmaBusMode::SoftwareReset)) and (timeout-- > 0))
			{
				// Wait until the PHY has reset.
				modm::delay_us(1);

				// NOTE: If the program hangs here check MII/RMII value.
			}
			if (not modm_assert_continue_fail_debug(timeout > 0, "eth.init",
													"ETH::initialize() timed out on software reset!", 1))
				return false;

			/* Configure SMI clock range */
			uint32_t csr_clock_divider = ETH->MACMIIAR & ETH_MACMIIAR_CR_Msk;
			if (SystemCoreClock >= 20_MHz and SystemCoreClock < 35_MHz)
				csr_clock_divider |= ETH_MACMIIAR_CR_Div16;
			else if (SystemCoreClock >= 35_MHz and SystemCoreClock < 60_MHz)
				csr_clock_divider |= ETH_MACMIIAR_CR_Div26;
			else if (SystemCoreClock >= 60_MHz and SystemCoreClock < 100_MHz)
				csr_clock_divider |= ETH_MACMIIAR_CR_Div42;
			else if (SystemCoreClock >= 100_MHz and SystemCoreClock < 150_MHz)
				csr_clock_divider |= ETH_MACMIIAR_CR_Div62;
			else if (SystemCoreClock >= 150_MHz)
				csr_clock_divider |= ETH_MACMIIAR_CR_Div102;

			ETH->MACMIIAR = csr_clock_divider;

			configureMac(PHYInterface::ANResult());
			configureDma();

			return true;
		}

		static void
		configureMac(modm::PHYInterface::ANResult negotiated)
		{

			uint32_t tmp;

			MacConfiguration_t maccr{// MacConfiguration::Ipv4ChecksumOffLoad |
									 MacConfiguration::RetryDisable};
			maccr |= Speed_t(negotiated.speed);
			maccr |= DuplexMode_t(negotiated.mode);

			tmp = ETH->MACCR & MacCrClearMask;
			tmp |= maccr.value;
			writeMACCR(tmp);

			MacFrameFilter_t macffr{
				PassControlFrame_t(PassControlFrame::BlockAll),
			};
			macffr.set(MacFrameFilter::HashOrPerfect);
			writeMACFFR(macffr.value);

			// Hash table
			ETH->MACHTHR = 0x00000000;
			ETH->MACHTLR = 0x00000000;

			MacFlowControl_t macfcr{MacFlowControl::ZeroQuantaPauseDisable};
			tmp = ETH->MACFCR & MacFcrClearMask;
			tmp |= macfcr.value;
			// writeReg(ETH->MACFCR, tmp);
			writeMACFCR(tmp);

			// no VLAN support for now
			writeMACVLANTR(0x00000000);
		};

		static void
		configureDma()
		{
			uint32_t tmp;

			DmaOperationMode_t dmaomr{
				DmaOperationMode::ReceiveStoreAndForward |
				DmaOperationMode::TransmitStoreAndForward |
				DmaOperationMode::OperateOnSecondFrame};
			tmp = ETH->DMAOMR & DmaOmrClearMask;
			tmp |= dmaomr.value;
			writeDMAOMR(tmp);

			DmaBusMode_t dmabmr{
				DmaBusMode::AddressAlignedBeats |
				DmaBusMode::FixedBurst |
				DmaBusMode::EnhancedDescFormat |
				DmaBusMode::UseSeparatePbl};
			dmabmr |= BurstLength_t(BurstLength::Length32Beats);
			dmabmr |= RxDmaBurstLength_t(BurstLength::Length32Beats);
			writeDMABMR(dmabmr.value);

			enableInterrupt(Interrupt::NormalIrqSummary | Interrupt::Receive);

			configureMacAddresses();
		}

		static void
		setMacAddress(MacAddressIndex index, uint8_t const *macAddress)
		{
			std::memcpy(macAddresses[uint32_t(index) / 8].data(), macAddress, 6);
		}

		static void
		configureMacAddresses()
		{
			static constexpr uint32_t ETH_MAC_ADDR_HBASE{ETH_MAC_BASE + 0x40}; /* Ethernet MAC address high offset */
			static constexpr uint32_t ETH_MAC_ADDR_LBASE{ETH_MAC_BASE + 0x44}; /* Ethernet MAC address low offset */

			static constexpr MacAddress zeroMac{0};

			for (std::size_t i = 0; i < macAddresses.size(); ++i)
			{
				auto const &macAddress = macAddresses[i];
				if (std::memcmp(zeroMac.data(), macAddress.data(), macAddress.size()) == 0)
					continue;

				uint32_t tmp_register = (macAddress[5] << 8) | macAddress[4];
				*reinterpret_cast<__IO uint32_t *>(ETH_MAC_ADDR_HBASE + i * 0x08) = tmp_register;
				tmp_register = (macAddress[3] << 24) | (macAddress[2] << 16) | (macAddress[1] << 8) | macAddress[0];
				*reinterpret_cast<__IO uint32_t *>(ETH_MAC_ADDR_LBASE + i * 0x08) = tmp_register;
			}
		};

		static void
		start()
		{
			// transmission enable
			uint32_t tmp = ETH->MACCR | ETH_MACCR_TE;
			writeMACCR(tmp);

			// reception enable
			tmp = ETH->MACCR | ETH_MACCR_RE;
			writeMACCR(tmp);

			// flush transmission fifo
			tmp = ETH->DMAOMR | ETH_DMAOMR_FTF;
			writeDMAOMR(tmp);

			// DMA transmission enable
			tmp = ETH->DMAOMR | ETH_DMAOMR_ST;
			writeDMAOMR(tmp);

			// DMA reception enable
			tmp = ETH->DMAOMR | ETH_DMAOMR_SR;
			writeDMAOMR(tmp);
		};
		static void
		stop()
		{
			// DMA transmission disable
			uint32_t tmp = ETH->DMAOMR & ~ETH_DMAOMR_ST;
			writeDMAOMR(tmp);

			// DMA reception disable
			tmp = ETH->DMAOMR & ~ETH_DMAOMR_SR;
			writeDMAOMR(tmp);

			// reception disable
			tmp = ETH->MACCR & ~ETH_MACCR_RE;
			writeMACCR(tmp);

			// flush transmission fifo
			tmp = ETH->DMAOMR | ETH_DMAOMR_FTF;
			writeDMAOMR(tmp);

			// transmission disable
			tmp = ETH->MACCR & ~ETH_MACCR_TE;
			writeMACCR(tmp);
		};

		static void
		setDmaTxDescriptorTable(uint32_t address)
		{
			ETH->DMATDLAR = address;
		}
		static void
		setDmaRxDescriptorTable(uint32_t address)
		{
			ETH->DMARDLAR = address;
		}

		static InterruptFlags
		getInterruptFlags()
		{
			return InterruptFlags(ETH->DMASR);
		}
		static void
		acknowledgeInterrupt(InterruptFlags_t irq)
		{
			// set only the bits you want to clear!
			// using an |= here would clear other fields as well
			ETH->DMASR = irq.value;
		}
		static void
		enableInterrupt(Interrupt_t irq)
		{
			ETH->DMAIER |= irq.value;
		}
		template <class T>
		static bool readPhyRegister(uint32_t Address, PHYBase::Register reg, T &value);
		template <class T>
		static bool writePhyRegister(uint32_t Address, PHYBase::Register reg, T value);

		void enablePTPMacFilter()
		{
			auto reg = TimeStampControlRegister_t(ETH->PTPTSCR);
			reg.set(TimeStampControlRegister::PTPMacFrameFilterEnable);
			writeRegister(ETH->PTPTSCR, reg.value);
		};

		void disablePTPMacFilter()
		{
			auto reg = TimeStampControlRegister_t(ETH->PTPTSCR);
			reg.reset(TimeStampControlRegister::PTPMacFrameFilterEnable);
			writeRegister(ETH->PTPTSCR, reg.value);
		};

		void setClockNodeType(ClockType_t type)
		{
			auto reg = TimeStampControlRegister_t(ETH->PTPTSCR);
			reg = (type | (~type.mask())) & reg;
			writeRegister(ETH->PTPTSCR, reg.value);
		};

		ClockType_t getClockNodeType()
		{
			return ClockType_t(ETH->PTPTSCR);
		};

		// void coarseTimeUpdate(EthTimeStamp &timedelta, bool isPositive);

		void adjustFrequency(int32_t frequencyChangePPB);
		uint32_t getFrequency();

		bool initializePTP()
		{
			/**
			 * @brief
			 * The time stamping feature can be enabled by setting bit 0 in the Time stamp control register
			 * (ETH__PTPTSCR). However, it is essential to initialize the time stamp counter after this bit
			 * is set to start time stamp operation. The proper sequence is the following.  If time stamp operation is disabled by clearing bit 0 in the ETH_PTPTSCR register, the
			 * above steps must be repeated to restart the time stamp operation.*/

			// 1. Mask the Time stamp trigger interrupt by setting bit 9 in the MACIMR register.
			// TODO
			// 2. Program Time stamp register bit 0 to enable time stamping.
			TimeStampControlRegister_t cr(ETH->PTPTSCR);
			cr.set(TimeStampControlRegister::TimeStampEnable);
			writeRegister(ETH->PTPTSCR, cr.value);
			// 3. Program the Subsecond increment register based on the PTP clock frequency.
			// 20 ns addend register overflow assumed
			writeRegister(ETH->PTPSSIR, 43);
			// 4. If you are using the Fine correction method, program the Time stamp addend register
			adjustFrequency(0);
			// and set Time stamp control register bit 5 (addend register update).

			// 5. Poll the Time stamp control register until bit 5 is cleared.
			while (TimeStampControlRegister_t(ETH->PTPTSCR).any(TimeStampControlRegister::TimeStampAddendRegisterUpdate))
				;
			// 6. To select the Fine correction method (if required), program Time stamp control register bit 1.
			cr = TimeStampControlRegister_t(ETH->PTPTSCR);
			cr.set(TimeStampControlRegister::TimeStampFineUpdate);
			writeRegister(ETH->PTPTSCR, cr.value);
			// 7. Program the Time stamp high update and Time stamp low update registers with the
			// appropriate time value.
			writeRegister(ETH->PTPTSHUR, 0);
			writeRegister(ETH->PTPTSLUR, 0);

			// 8. Set Time stamp control register bit 2 (Time stamp init).

			// 9. The Time stamp counter starts operation as soon as it is initialized with the value
			// written in the Time stamp update register.
			// 10. Enable the MAC receiver and transmitter for proper time stamping.
			return true;
		};
		void enableFrameTimestamping();
		// void enableTimedInterrupt(const EthTimeStamp &triggerTime);

	private:
		/**
		 * @brief Attention: Errata sheet ES0290 Rev 7: 2.14.5 :
		 * Successive write operations need 4 idle tx/rx clock cycles inbetween
		 * for MII in 100mbit mode those are 25Mhz for RMII 50Mhz for 10 bit 2.5Mhz and 5Mhz
		 * so at most 1.6µs!
		 *
		 * The stated workaround does not require the read when the delay is guaranteed
		 */
		static void writeRegister(volatile uint32_t &reg, uint32_t value)
		{
			// reg = value;
			//(void)reg;
			modm::delay_us(2);
			// todo: when the last write is more thenn the 4 cylces in the past write directly
			reg = value;
		};
		static void
		writeMACCR(uint32_t value)
		{
			/*ETH->MACCR = value;
			(void)ETH->MACCR;
			modm::delay_ms(1);
			ETH->MACCR = value;*/
			writeRegister(ETH->MACCR, value);
		}
		static void
		writeMACFCR(uint32_t value)
		{
			writeRegister(ETH->MACFCR, value);
			/*ETH->MACFCR = value;
			(void)ETH->MACFCR;
			modm::delay_ms(1);
			ETH->MACFCR = value;*/
		}
		static void
		writeMACFFR(uint32_t value)
		{
			writeRegister(ETH->MACFFR, value);
			/*ETH->MACFFR = value;
			(void)ETH->MACFFR;
			modm::delay_ms(1);
			ETH->MACFFR = value;*/
		}
		static void
		writeMACVLANTR(uint32_t value)
		{
			writeRegister(ETH->MACVLANTR, value);
			/*ETH->MACVLANTR = value;
			(void)ETH->MACVLANTR;
			modm::delay_ms(1);
			ETH->MACVLANTR = value;*/
		}
		static void
		writeDMABMR(uint32_t value)
		{
			writeRegister(ETH->DMABMR, value);
			/*ETH->DMABMR = value;
			(void)ETH->DMABMR;
			modm::delay_ms(1);
			ETH->DMABMR = value;*/
		}
		static void
		writeDMAOMR(uint32_t value)
		{
			writeRegister(ETH->DMAOMR, value);
			/*ETH->DMAOMR = value;
			(void)ETH->DMAOMR;
			modm::delay_ms(1);
			ETH->DMAOMR = value;*/
		}

		static inline PHYInterface::DuplexMode duplexMode = PHYInterface::DuplexMode::Full;
		static inline PHYInterface::Speed speed = PHYInterface::Speed::Speed100M;
		static inline PHYInterface::LinkStatus linkStatus = PHYInterface::LinkStatus::Down;
		static inline uint32_t lastWriteCYCNT = 0;
		using MacAddress = std::array<uint8_t, 6>;
		using MacAddresses = std::array<MacAddress, 4>;
		static inline MacAddresses macAddresses;
	};

	//*+++++++++++++++++++++ EXTERN C +++++++++++++++++++++++++
	namespace ptp
	{
		typedef struct PTPTimeRegister
		{
			uint32_t seconds;
			uint32_t subseconds;
		} PTPTimeRegister_t;

		static uint32_t subsecond_to_nanosecond(uint32_t subsecond_value)
		{
			uint64_t val = subsecond_value * 1000000000ll;
			val >>= 31;
			return val;
		}

		// Conversion from PTP to hardware format.
		static uint32_t nanosecond_to_subsecond(uint32_t subsecond_value)
		{
			uint64_t val = subsecond_value * 0x80000000ll;
			val /= 1000000000;
			return val;
		}

		static void set_reg_save(volatile uint32_t &regaddr, uint32_t bits)
		{
			uint32_t temp = regaddr;
			temp |= bits;
			modm::delay_us(2);
			regaddr = temp;
			return;
		};

		static void reset_reg_save(volatile uint32_t &regaddr, uint32_t bits)
		{
			uint32_t temp = regaddr;
			temp &= ~(bits);
			modm::delay_us(2);
			regaddr = temp;
			return;
		};

		static void start()
		{
			using namespace modm::platform;
			using modm::platform::eth;
			using TimeStampControlRegister_t = modm::platform::eth::TimeStampControlRegister_t;
			using TimeStampControlRegister = modm::platform::eth::TimeStampControlRegister;
			// config
			//

			// vRegisterDelay();

			// 1. Mask the Time stamp trigger interrupt by setting bit 9 in the MACIMR register.
			set_reg_save(ETH->MACIMR, 1 << 9);
			// vRegisterDelay();
			// config

			TimeStampControlRegister_t tscr(ETH->PTPTSCR);
			tscr |= TimeStampControlRegister::TimeStamp802_3	   // enable 803.2 timestamping
					| TimeStampControlRegister::SnoopPTPV2		   // enable ptp v2 snooping
					| TimeStampControlRegister::TimeStampEventMsg; // enable snapshots for event messages
			set_reg_save(ETH->PTPTSCR, tscr.value);
			set_reg_save(ETH->PTPTSCR, uint32_t(TimeStampControlRegister::TimeStampEnable));
			// vRegisterDelay();
			//  3. Program the Subsecond increment register based on the PTP clock frequency.
			modm::delay_us(2);
			ETH->PTPSSIR = ADJ_FREQ_BASE_INCREMENT;

			// 4. If you are using the Fine correction method, program the Time stamp addend register
			//    and set Time stamp control register bit 5 (addend register update).
			// yes we are
			modm::delay_us(2);
			ETH->PTPTSAR = ADJ_FREQ_BASE_ADDEND;
			set_reg_save(ETH->PTPTSCR, uint32_t(TimeStampControlRegister::TimeStampAddendRegisterUpdate));
			// 5. Poll the Time stamp control register until bit 5 is cleared.
			while (
				TimeStampControlRegister_t(ETH->PTPTSCR) & TimeStampControlRegister::TimeStampAddendRegisterUpdate)
				;
			// 6. To select the Fine correction method (if required), program Time stamp control register bit 1.
			set_reg_save(ETH->PTPTSCR, uint32_t(TimeStampControlRegister::TimeStampFineUpdate));
			// 7. Program the Time stamp high update and Time stamp low update registers with the
			// appropriate time value.
			//  for now initialize to 0
			modm::delay_us(2);
			ETH->PTPTSHUR = 0;
			modm::delay_us(2);
			ETH->PTPTSLUR = 0;
			// 8. Set Time stamp control register bit 2 (Time stamp init).
			set_reg_save(ETH->PTPTSCR, uint32_t(TimeStampControlRegister::TimeStampSystemTimeInitialize));
			// 9. The Time stamp counter starts operation as soon as it is initialized with the value
			// written in the Time stamp update register.
			// 10. Enable the MAC receiver and transmitter for proper time stamping.
			//  Mask the time stamp trigger interrupt by setting bit 9 in the MACIMR register.

			// block until PTP clock starts running:
			while ((ETH->PTPTSCR & uint32_t(TimeStampControlRegister::TimeStampSystemTimeInitialize)) != 0)
				;

			// Set PPS to 2 ^10 (1024 Hz)
			// ETH->PTPPPSCR = 10;
			// enable pps
			// set_reg_save(TIM2->OR, TIM2_OR_ITR1_RMP_0);
		};

		// Get the PTP time.

		static PTPTimeRegister get_time()
		{
			int32_t hi_reg;
			int32_t lo_reg;
			int32_t hi_reg_after;

			// The problem is we are reading two 32-bit registers that form
			// a 64-bit value, but it's possible the high 32-bits of the value
			// rolls over before we read the low 32-bits of the value.  To avoid
			// this situation we read the high 32-bits twice and determine which
			// high 32-bits the low 32-bit are associated with.
			__disable_irq();
			hi_reg = ETH->PTPTSHR;
			lo_reg = ETH->PTPTSLR;
			hi_reg_after = ETH->PTPTSHR;
			__enable_irq();

			// Did a roll over occur while reading?
			if (hi_reg != hi_reg_after)
			{
				// We now know a roll over occurred. If the rollover occured before
				// the reading of the low 32-bits we move the substitute the second
				// 32-bit high value for the first 32-bit high value.
				if (lo_reg < (INT_MAX / 2))
					hi_reg = hi_reg_after;
			}

			// Now convert the raw registers values into timestamp values.
			return PTPTimeRegister{.seconds=hi_reg, .subseconds = lo_reg};
		};

		static void coarseUpdate(uint32_t high_reg, uint32_t lowReg, bool isPositive)
		{
			using namespace modm::platform;
			using modm::platform::eth;
			using TimeStampControlRegister_t = modm::platform::eth::TimeStampControlRegister_t;
			using TimeStampControlRegister = modm::platform::eth::TimeStampControlRegister;
			// Convert nanosecond to subseconds.

			// Write the offset (positive or negative) in the Time stamp update
			// high and low registers.
			// Set the PTP Time Update High Register
			ETH->PTPTSHUR = high_reg;
			constexpr uint32_t sign_bit = (1 << 31);
			if (isPositive)
			{
				ETH->PTPTSLUR = (lowReg & ~(sign_bit));
			}else{
				ETH->PTPTSLUR = (lowReg | sign_bit);
			}
			// wait for TS System time initialize bit to be cleared
			while ((ETH->PTPTSCR & uint32_t(TimeStampControlRegister::TimeStampSystemTimeInitialize)) != 0)
				;
			// Set Time stamp control register bit 2 (Time stamp init).
			ETH->PTPTSCR |= uint32_t(TimeStampControlRegister::TimeStampSystemtTimeUpdate);
			// block until initialization is complete
			while ((ETH->PTPTSCR & uint32_t(TimeStampControlRegister::TimeStampSystemtTimeUpdate)) != 0);
		};

		// Adjust the PTP system clock rate by the specified value in parts-per-billion.

		static void adj_freq(int32_t adj_ppb)
		{
			using namespace modm::platform;
			using modm::platform::eth;

			using TimeStampControlRegister = modm::platform::eth::TimeStampControlRegister;

			// Adjust the fixed base frequency by parts-per-billion.
			// addend = base + ((base * adj_ppb) / 1000000000);
			// addend <- the PTPTSARegister
			ETH->PTPTSAR = ADJ_FREQ_BASE_ADDEND + (int32_t)((((int64_t)ADJ_FREQ_BASE_ADDEND) * adj_ppb) / 1'000'000'000);

			// ETH_EnablePTPTimeStampAddend();
			//  wait until TSARU bit is cleared before setting it. according to refman
			while ((ETH->PTPTSCR & uint32_t(TimeStampControlRegister::TimeStampAddendRegisterUpdate)) != 0);
			// set addend enable bit
			ETH->PTPTSCR |= uint32_t(TimeStampControlRegister::TimeStampAddendRegisterUpdate);
		}
	};

}

#include "eth_impl.hpp"

#endif // MODM_ETH_HPP