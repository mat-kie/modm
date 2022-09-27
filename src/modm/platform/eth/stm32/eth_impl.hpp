/**
 * @file eth_impl.hpp
 * @author Mattis Kieffer
 * @brief Adapted from Mike Wolframs original implementation for FreeRTOS+TCP for modm
 * @version 0.1
 * @date 2022-09-27
 *
 * @copyright Copyright (c) 2022
 *
 */


// just for ide, will actually never be included due to include guard
#include "eth.hpp"
namespace modm::platform
{
	template <int descriptorTableSize>
	template <class... Signals>
	void Eth<descriptorTableSize>::connect()
	{
		(GpioStatic<typename Signals::Data>::configure(Gpio::OutputType::PushPull,
													   Gpio::OutputSpeed::VeryHigh),
		 ...);
		GpioConnector<Peripheral::Eth, Signals...>::connect();
	};

	template <int descriptorTableSize>
	template <eth::MediaInterface Interface>
	inline bool
	Eth<descriptorTableSize>::initialize(uint8_t priority)
	{
		using namespace modm::literals;

		Rcc::enable<Peripheral::Eth>();

		NVIC_SetPriority(ETH_IRQn, priority);
		NVIC_EnableIRQ(ETH_IRQn);

		// Select Interface Mode (MII/RMII)
		SYSCFG->PMC &= ~(SYSCFG_PMC_MII_RMII_SEL);
		SYSCFG->PMC |= uint32_t(Interface);

		/* Ethernet Software reset */
		/* Set the SWR bit: resets all MAC subsystem internal registers and logic */
		/* After reset all the registers holds their respective reset values */
		ETH->DMABMR |=
			DmaBusMode_t(DmaBusMode::SoftwareReset | DmaBusMode::EnhancedDescFormat).value;

		/* Wait for software reset */
		/* Note: The SWR is not performed if the ETH_RX_CLK or the ETH_TX_CLK are
		 * not available, please check your external PHY or the IO configuration */
		int timeout = 1'000; // max 1ms
		while ((DmaBusMode(ETH->DMABMR) & DmaBusMode::SoftwareReset) and
			   (timeout-- > 0))
		{
			// Wait until the PHY has reset.
			modm::delay_us(1);

			// NOTE: If the program hangs here check MII/RMII value.
		}
		if (not modm_assert_continue_fail_debug(
				timeout > 0, "eth.init", "ETH::initialize() timed out on software reset!", 1))
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

		configureMac(ANResult{
			.successful = false, .mode = eth::DuplexMode::Full, .speed = eth::Speed::Speed100M});
		configureDma();

		return true;
	};

	template <int descriptorTableSize>
	void
	Eth<descriptorTableSize>::configureMac(ANResult negotiated)
	{

		uint32_t tmp;

		MacConfiguration_t maccr{MacConfiguration::Ipv4ChecksumOffLoad |
								 MacConfiguration::RetryDisable};
		maccr |= Speed_t(negotiated.speed);
		maccr |= DuplexMode_t(negotiated.mode);

		tmp = ETH->MACCR & MacCrClearMask;
		tmp |= maccr.value;
		writeReg(ETH->MACCR, tmp);

		MacFrameFilter_t macffr{
			PassControlFrame_t(PassControlFrame::BlockAll),
		};
		writeReg(ETH->MACFFR, macffr.value);

		// Hash table
		ETH->MACHTHR = 0x00000000;
		ETH->MACHTLR = 0x00000000;

		MacFlowControl_t macfcr{MacFlowControl::ZeroQuantaPauseDisable};
		tmp = ETH->MACFCR & MacFcrClearMask;
		tmp |= macfcr.value;
		//writeReg(ETH->MACFCR, tmp);
		writeReg(ETH->MACFCR, tmp);

		// no VLAN support for now
		writeReg(ETH->MACVLANTR, 0x00000000);
	};

	template <int descriptorTableSize>
	void
	Eth<descriptorTableSize>::configureDma()
	{
		uint32_t tmp;

		DmaOperationMode_t dmaomr{DmaOperationMode::ReceiveStoreAndForward |
								  DmaOperationMode::TransmitStoreAndForward |
								  DmaOperationMode::OperateOnSecondFrame};
		tmp = ETH->DMAOMR & DmaOmrClearMask;
		tmp |= dmaomr.value;
		writeReg(ETH->DMAOMR, tmp);

		DmaBusMode_t dmabmr{DmaBusMode::AddressAlignedBeats | DmaBusMode::FixedBurst |
							DmaBusMode::EnhancedDescFormat | DmaBusMode::UseSeparatePbl};
		dmabmr |= BurstLength_t(BurstLength::Length32Beats);
		dmabmr |= RxDmaBurstLength_t(BurstLength::Length32Beats);
		writeReg(ETH->DMABMR, dmabmr.value);

		enableInterrupt(Interrupt::NormalIrqSummary | Interrupt::Receive);

		configureMacAddresses();
	};

	template <int descriptorTableSize>
	void
	Eth<descriptorTableSize>::setMacAddress(MacAddressIndex index, uint8_t const *macAddress)
	{
		std::memcpy(macAddresses[uint32_t(index) / 8].data(), macAddress, 6);
	};

	template <int descriptorTableSize>
	void
	Eth<descriptorTableSize>::configureMacAddresses()
	{
		static constexpr uint32_t ETH_MAC_ADDR_HBASE{ETH_MAC_BASE +
													 0x40}; /* Ethernet MAC address high offset */
		static constexpr uint32_t ETH_MAC_ADDR_LBASE{ETH_MAC_BASE +
													 0x44}; /* Ethernet MAC address low offset */

		static constexpr MacAddress zeroMac{0};

		for (std::size_t i = 0; i < macAddresses.size(); ++i)
		{
			auto const &macAddress = macAddresses[i];
			if (std::memcmp(zeroMac.data(), macAddress.data(), macAddress.size()) == 0)
				continue;

			uint32_t tmp_register = (macAddress[5] << 8) | macAddress[4];
			*reinterpret_cast<__IO uint32_t *>(ETH_MAC_ADDR_HBASE + i * 0x08) = tmp_register;
			tmp_register = (macAddress[3] << 24) | (macAddress[2] << 16) | (macAddress[1] << 8) |
						   macAddress[0];
			*reinterpret_cast<__IO uint32_t *>(ETH_MAC_ADDR_LBASE + i * 0x08) = tmp_register;
		}
	};

	template <int descriptorTableSize>
	void
	Eth<descriptorTableSize>::start()
	{
		// transmission enable
		uint32_t tmp = ETH->MACCR | ETH_MACCR_TE;
		writeReg(ETH->MACCR, tmp);

		// reception enable
		tmp = ETH->MACCR | ETH_MACCR_RE;
		writeReg(ETH->MACCR, tmp);

		// flush transmission fifo
		tmp = ETH->DMAOMR | ETH_DMAOMR_FTF;
		writeReg(ETH->DMAOMR, tmp);

		// DMA transmission enable
		tmp = ETH->DMAOMR | ETH_DMAOMR_ST;
		writeReg(ETH->DMAOMR, tmp);

		// DMA reception enable
		tmp = ETH->DMAOMR | ETH_DMAOMR_SR;
		writeReg(ETH->DMAOMR, tmp);
	}

	template <int descriptorTableSize>
	void
	Eth<descriptorTableSize>::stop()
	{
		// DMA transmission disable
		uint32_t tmp = ETH->DMAOMR & ~ETH_DMAOMR_ST;
		writeReg(ETH->DMAOMR, tmp);

		// DMA reception disable
		tmp = ETH->DMAOMR & ~ETH_DMAOMR_SR;
		writeReg(ETH->DMAOMR, tmp);

		// reception disable
		tmp = ETH->MACCR & ~ETH_MACCR_RE;
		writeReg(ETH->MACCR, tmp);

		// flush transmission fifo
		tmp = ETH->DMAOMR | ETH_DMAOMR_FTF;
		writeReg(ETH->DMAOMR, tmp);

		// transmission disable
		tmp = ETH->MACCR & ~ETH_MACCR_TE;
		writeReg(ETH->MACCR, tmp);
	};
	///////////////////////////////////7

	template <int descriptorTableSize>
	void
	Eth<descriptorTableSize>::setDmaTxDescriptorTable(uint32_t address)
	{
		ETH->DMATDLAR = address;
	};

	template <int descriptorTableSize>
	void
	Eth<descriptorTableSize>::setDmaRxDescriptorTable(uint32_t address)
	{
		ETH->DMARDLAR = address;
	};

	template <int descriptorTableSize>

	eth::InterruptFlags
	Eth<descriptorTableSize>::getInterruptFlags()
	{
		return InterruptFlags(ETH->DMASR);
	};

	template <int descriptorTableSize>
	void
	Eth<descriptorTableSize>::acknowledgeInterrupt(InterruptFlags_t irq)
	{
		// set only the bits you want to clear!
		// using an |= here would clear other fields as well
		ETH->DMASR = irq.value;
	};

	template <int descriptorTableSize>
	void
	Eth<descriptorTableSize>::enableInterrupt(Interrupt_t irq)
	{
		ETH->DMAIER |= irq.value;
	};

	template <int descriptorTableSize>
	template <class T>
	bool
	Eth<descriptorTableSize>::writePhyRegister(uint32_t Address, PHYBase::Register reg, T value)
	{
		// get only CR bits from MACMIIAR
		uint32_t tmp = ETH->MACMIIAR & ETH_MACMIIAR_CR_Msk;
		tmp |= (Address << 11) & ETH_MACMIIAR_PA;
		tmp |= (uint32_t(reg) << 6) & ETH_MACMIIAR_MR;
		tmp |= ETH_MACMIIAR_MW;
		tmp |= ETH_MACMIIAR_MB;

		ETH->MACMIIDR = value.value;
		ETH->MACMIIAR = tmp;

		int timeout = 0xffff;
		while (timeout-- > 0)
		{
			if ((ETH->MACMIIAR & ETH_MACMIIAR_MB) == 0)
				return true;
		}

		return false;
	};

	template <int descriptorTableSize>
	template <class T>
	bool
	Eth<descriptorTableSize>::readPhyRegister(uint32_t Address, PHYBase::Register reg, T &value)
	{
		// get only CR bits from MACMIIAR
		uint32_t tmp = ETH->MACMIIAR & ETH_MACMIIAR_CR_Msk;
		tmp |= (Address << 11) & ETH_MACMIIAR_PA;
		tmp |= (uint32_t(reg) << 6) & ETH_MACMIIAR_MR;
		tmp &= ~ETH_MACMIIAR_MW;
		tmp |= ETH_MACMIIAR_MB;

		ETH->MACMIIAR = tmp;

		int timeout = 0xffff;
		while (timeout-- > 0)
		{
			if ((ETH->MACMIIAR & ETH_MACMIIAR_MB) == 0)
			{
				// busy flag cleared, read data
				value.value = ETH->MACMIIDR;
				return true;
			}
		}

		return false;
	};
	/**
	 * @brief check if another RX Framebuffer can be read
	 *
	 * @return true
	 * @return false
	 */
	template <int descriptorTableSize>
	bool
	Eth<descriptorTableSize>::hasRXFrame()
	{
		return !(RDes0_t(RxDescriptor->Status) & RDes0::DmaOwned);
	}
	/**
	 * @brief get the next RX Framebuffer handle
	 *  this is the RecieveFrame in the procedual IEEE802.3 model
	 * @return DMADescriptorHandle
	 */
	template <int descriptorTableSize>
	eth::RxDMADescriptorHandle
	Eth<descriptorTableSize>::getRXFrame()
	{
		// advance until we get a descriptor that is not dma owned

		auto descriptor = RxDMADescriptorHandle(RxDescriptor);
		RxDescriptor = reinterpret_cast<DmaDescriptor_t *>(RxDescriptor->Buffer2NextDescAddr);
		return descriptor;
	};

	template <int descriptorTableSize>
	bool
	Eth<descriptorTableSize>::hasFinishedTXD()
	{

		return !(TDes0_t(DmaTxDescriptorToClear->Status) & TDes0::DmaOwned);
	};

	template <int descriptorTableSize>
	eth::TxDMADescriptorHandle
	Eth<descriptorTableSize>::getFinishedTXD()
	{
		auto descriptor = TxDMADescriptorHandle(DmaTxDescriptorToClear);
		DmaTxDescriptorToClear = reinterpret_cast<DmaDescriptor_t *>(DmaTxDescriptorToClear->Buffer2NextDescAddr);
		return descriptor;
	};

	template <int descriptorTableSize>
	bool
	Eth<descriptorTableSize>::hasNextTXD()
	{
		return !(TDes0_t(TxDescriptor->Status) & TDes0::DmaOwned);
	}

	template <int descriptorTableSize>
	eth::TxDMADescriptorHandle
	Eth<descriptorTableSize>::getNextTXD()
	{
		auto descriptor = TxDMADescriptorHandle(TxDescriptor);
		TxDescriptor = reinterpret_cast<DmaDescriptor_t *>(TxDescriptor->Buffer2NextDescAddr);
		return descriptor;
	}

	template <int descriptorTableSize>
	bool
	Eth<descriptorTableSize>::InitTXDescriptorTable()
	{
		std::memset(&DmaTxDescriptorTable, 0, sizeof(DmaTxDescriptorTable));
		static constexpr TDes0_t dmaDescriptorStatus{
			TDes0_t(TDes0::SecondAddressChained) |
			CrcControl_t(CrcControl::HardwareCalculated)};

		DmaDescriptor_t *dmaDescriptor{TxDescriptor};
		for (int index = 0; index < descriptorTableSize; ++index, ++dmaDescriptor)
		{
			dmaDescriptor->Status = dmaDescriptorStatus.value;
			if (index < descriptorTableSize - 1)
			{
				dmaDescriptor->Buffer2NextDescAddr = uint32_t(dmaDescriptor + 1);
			}
			else
			{
				dmaDescriptor->Buffer2NextDescAddr = uint32_t(TxDescriptor);
			}
		}

		setDmaTxDescriptorTable(uint32_t(TxDescriptor));
		return true;
	}

	template <int descriptorTableSize>
	bool
	Eth<descriptorTableSize>::InitRXDescriptorTable(uint32_t buffer_ptrs[descriptorTableSize], int buffer_size)
	{
		std::memset(&DmaRxDescriptorTable, 0, sizeof(DmaRxDescriptorTable));
		DmaDescriptor_t *dmaDescriptor{RxDescriptor};

		for (int index = 0; index < descriptorTableSize; ++index, ++dmaDescriptor)
		{
			dmaDescriptor->ControlBufferSize = uint32_t(RDes1::SecondAddressChained) |
											   uint32_t(buffer_size);
			if (!buffer_ptrs[index])
				return false;
			dmaDescriptor->Buffer1Addr = buffer_ptrs[index];
			dmaDescriptor->Status = uint32_t(RDes0::DmaOwned);
			if (index < descriptorTableSize - 1)
			{
				dmaDescriptor->Buffer2NextDescAddr = uint32_t(dmaDescriptor + 1);
			}
			else
			{
				dmaDescriptor->Buffer2NextDescAddr = uint32_t(RxDescriptor);
			}
		}
		setDmaRxDescriptorTable(uint32_t(RxDescriptor));
		return true;
	}

	template <int descriptorTableSize>
	void
	Eth<descriptorTableSize>::writeReg(volatile uint32_t& reg, uint32_t value)
	{
		reg = value;
		# pragma GCC diagnostic push
		# pragma GCC diagnostic ignored "-Wunused-value"
		(void*)(&reg); //cast to void no longer guarantees read in >c++14 but dereferencing seems to to so
		# pragma GCC diagnostic pop
		modm::delay_ms(1);
		reg = value;
	}

};