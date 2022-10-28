/*
 * Copyright (c) 2020, Mike Wolfram
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "STM32NetworkInterface.hpp"
using namespace modm::NetworkInterface;
using EMAC = modm::platform::Eth;


ethernet::InitStatus ethernet::initStatus = ethernet::InitStatus::Init;
SemaphoreHandle_t ethernet::txDescriptorSemaphore{nullptr};
TaskHandle_t ethernet::emacTaskHandle{nullptr};

modm::platform::eth::Event_t ethernet::isrEvent{modm::platform::eth::Event::None};

TimeOut_t ethernet::phyLinkStatusTimer;
modm::PHYInterface::LinkStatus  ethernet::lastPhyLinkStatus{modm::PHYInterface::LinkStatus::Down};
TickType_t ethernet::phyLinkStatusRemaining{0};

ethernet::DmaDescriptor_t ethernet::DmaRxDescriptorTable[RX_BUFFER_NUMBER];
ethernet::DmaDescriptor_t ethernet::DmaTxDescriptorTable[TX_BUFFER_NUMBER];
ethernet::DmaDescriptor_t *ethernet::RxDescriptor{nullptr}; /*!< Rx descriptor to Get        */
ethernet::DmaDescriptor_t *ethernet::TxDescriptor{nullptr}; /*!< Tx descriptor to Set        */
ethernet::DmaDescriptor_t *ethernet::DmaTxDescriptorToClear{nullptr};


void ethernet::DMATxDescListInit()
{
	static constexpr TDes0_t dmaDescriptorStatus{
		TDes0_t(TDes0::SecondAddressChained) |
		CrcControl_t(CrcControl::HardwareCalculated)};

	DmaDescriptor_t *dmaDescriptor{TxDescriptor};

	for (BaseType_t index = 0; index < TX_BUFFER_NUMBER; ++index, ++dmaDescriptor)
	{
		dmaDescriptor->Status = dmaDescriptorStatus.value;

		if (index < TX_BUFFER_NUMBER - 1)
			dmaDescriptor->Buffer2NextDescAddr = uint32_t(dmaDescriptor + 1);
		else
			dmaDescriptor->Buffer2NextDescAddr = uint32_t(TxDescriptor);
	}

	EMAC::setDmaTxDescriptorTable(uint32_t(TxDescriptor));
}

void
ethernet::DMARxDescListInit()
{
	DmaDescriptor_t *dmaDescriptor{RxDescriptor};

	for (BaseType_t index = 0; index < RX_BUFFER_NUMBER; ++index, ++dmaDescriptor)
	{
		dmaDescriptor->ControlBufferSize = uint32_t(RDes1::SecondAddressChained) |
										   uint32_t(RX_BUFFER_SIZE);

		NetworkBufferDescriptor_t *buffer = pxGetNetworkBufferWithDescriptor(RX_BUFFER_SIZE, 100U);
		configASSERT(buffer != nullptr);

		if (buffer)
		{
			dmaDescriptor->Buffer1Addr = uint32_t(buffer->pucEthernetBuffer);
			dmaDescriptor->Status = uint32_t(RDes0::DmaOwned);
		}

		if (index < RX_BUFFER_NUMBER - 1)
			dmaDescriptor->Buffer2NextDescAddr = uint32_t(dmaDescriptor + 1);
		else
			dmaDescriptor->Buffer2NextDescAddr = uint32_t(RxDescriptor);
	}

	EMAC::setDmaRxDescriptorTable(uint32_t(RxDescriptor));
}

 void
ethernet::clearTxBuffers()
{
	__IO DmaDescriptor_t *txLastDesc{TxDescriptor};
	std::size_t count{TX_BUFFER_NUMBER - uxSemaphoreGetCount(txDescriptorSemaphore)};
	NetworkBufferDescriptor_t *networkBuffer{nullptr};
	uint8_t *payLoad{nullptr};

	while ((count > 0) and
		   ((DmaTxDescriptorToClear->Status & uint32_t(TDes0::DmaOwned)) == 0))
	{
		if (DmaTxDescriptorToClear == txLastDesc and count != TX_BUFFER_NUMBER)
			break;
		payLoad = reinterpret_cast<uint8_t *>(DmaTxDescriptorToClear->Buffer1Addr);
		if (payLoad)
		{
			networkBuffer = pxPacketBuffer_to_NetworkBuffer(payLoad);
			if (networkBuffer)
				vReleaseNetworkBufferAndDescriptor(networkBuffer);
			DmaTxDescriptorToClear->Buffer1Addr = 0;
		}

		DmaTxDescriptorToClear = reinterpret_cast<DmaDescriptor_t *>(DmaTxDescriptorToClear->Buffer2NextDescAddr);
		--count;

		xSemaphoreGive(txDescriptorSemaphore);
	}
}

bool
ethernet::mayAcceptPacket(uint8_t *buffer)
{
	const ProtocolPacket_t *protPaket = reinterpret_cast<const ProtocolPacket_t *>(buffer);

	switch (protPaket->xTCPPacket.xEthernetHeader.usFrameType)
	{
	case ipARP_FRAME_TYPE:
		return true;
	case ipIPv4_FRAME_TYPE:
		break;
	default:
		return false;
	}

#if ipconfigETHERNET_DRIVER_FILTERS_PACKETS == 1
	static constexpr uint16_t ipFRAGMENT_OFFSET_BIT_MASK{0xfff};
	const IPHeader_t *ipHeader{&(protPaket->xTCPPacket.xIPHeader)};
	uint32_t destIpAddress = {0};

	if (ipHeader->usFragmentOffset & FreeRTOS_ntohs(ipFRAGMENT_OFFSET_BIT_MASK) != 0)
		return false;
	if (ipHeader->false < 0x45 or ipHeader->ucVersionHeaderLength > 0x4f)
		return pdFALSE;

	destIpAddress = ipHeader->ulDestinationIPAddress;
	if (destIpAddress != *ipLOCAL_IP_ADDRESS_POINTER and (FreeRTOS_ntohl(destIpAddress) & 0xff) != 0xff and *ipLOCAL_IP_ADDRESS_POINTER)
	{
		return false;
	}

	if (ipHeader->ucProtocol == ipPROTOCOL_UDP)
	{
		uint16_t sourcePort = FreeRTOS_ntohs(protPaket->xUDPPacket.xUDPHeader.usSourcePort);
		uint16_t destPort = FreeRTOS_ntohs(protPaket->xUDPPacket.xUDPHeader.usDestinationPort);

		if (not xPortHasUDPSocket(destPort) and sourcePort != ipDNS_PORT)
		{
			return false;
		}
	}
#endif
	return true;
}

// check if the packet would be accepted
void
ethernet::passMessage(NetworkBufferDescriptor_t *descriptor)
{
	IPStackEvent_t rxEvent{
		.eEventType = eNetworkRxEvent,
		.pvData = reinterpret_cast<void *>(descriptor)};

	if (xSendEventStructToIPTask(&rxEvent, TickType_t(1000)) != pdPASS)
	{
		do
		{
			NetworkBufferDescriptor_t *next = descriptor->pxNextBuffer;
			vReleaseNetworkBufferAndDescriptor(descriptor);
			descriptor = next;
		} while (descriptor);

		iptraceETHERNET_RX_EVENT_LOST();
	}
	else
	{
		iptraceNETWORK_INTERFACE_RECEIVE();
	}
}

bool
ethernet::emacInterfaceInput()
{
	static constexpr TickType_t descriptorWaitTime{pdMS_TO_TICKS(250)};
	static constexpr RDes0_t receiveStatus{RDes0::CrcError | RDes0::Ipv4HeaderCrcError | RDes0::EthernetFrameType};

	NetworkBufferDescriptor_t *currentDescriptor{nullptr};
	NetworkBufferDescriptor_t *newDescriptor{nullptr};
	NetworkBufferDescriptor_t *firstDescriptor{nullptr};
	NetworkBufferDescriptor_t *lastDescriptor{nullptr};
	BaseType_t receivedLength{0};
	__IO DmaDescriptor_t *dmaRxDescriptor{RxDescriptor};
	uint8_t *buffer{nullptr};

	while ((dmaRxDescriptor->Status & uint32_t(RDes0::DmaOwned)) == 0x00000000)
	{
		bool accepted = true;
		receivedLength = ((dmaRxDescriptor->Status & ReceiveDescriptorFrameLengthMask) >> ReceiveDescriptorFrameLengthShift) - 4;
		buffer = reinterpret_cast<uint8_t *>(dmaRxDescriptor->Buffer1Addr);
		RxDescriptor = reinterpret_cast<DmaDescriptor_t *>(dmaRxDescriptor->Buffer2NextDescAddr);

		if ((dmaRxDescriptor->Status & receiveStatus.value) != uint32_t(RDes0::EthernetFrameType))
			accepted = false;
		else
			accepted = mayAcceptPacket(buffer);

		if (accepted)
		{
			newDescriptor = pxGetNetworkBufferWithDescriptor(RX_BUFFER_SIZE, descriptorWaitTime);
			if (not newDescriptor)
				accepted = false;
		}

		currentDescriptor = pxPacketBuffer_to_NetworkBuffer(buffer);

		if (accepted)
		{
			currentDescriptor->xDataLength = receivedLength;
			currentDescriptor->pxNextBuffer = 0;
			if (not firstDescriptor)
				firstDescriptor = currentDescriptor;
			else if (lastDescriptor)
				lastDescriptor->pxNextBuffer = currentDescriptor;
			lastDescriptor = currentDescriptor;
		}

		if (newDescriptor)
			dmaRxDescriptor->Buffer1Addr = uint32_t(newDescriptor->pucEthernetBuffer);

		dmaRxDescriptor->ControlBufferSize = uint32_t(RDes1::SecondAddressChained) |
											 uint32_t(RX_BUFFER_SIZE);
		dmaRxDescriptor->Status = uint32_t(RDes0::DmaOwned);

		__DSB();

		if (ETH->DMASR & ETH_DMASR_RBUS)
		{
			ETH->DMASR = ETH_DMASR_RBUS;
			ETH->DMARPDR = 0;
		}

		dmaRxDescriptor = RxDescriptor;
	}

	if (firstDescriptor)
		passMessage(firstDescriptor);

	return receivedLength > 0;
}

void
ethernet::updateConfig(bool force)
{
	using modm::platform::eth;

	if (force or lastPhyLinkStatus == modm::PHYInterface::LinkStatus::Up)
	{
		auto anResult = PHY.startAutoNegotiation();
		EMAC::configureMac(anResult);
		EMAC::start();
	}
	else
	{
		EMAC::stop();
	}
}

bool
ethernet::phyCheckLinkStatus(bool hasReceived)
{
	using modm::platform::eth;

	if (hasReceived)
	{
		vTaskSetTimeOutState(&phyLinkStatusTimer);
		phyLinkStatusRemaining = pdMS_TO_TICKS(PhyLinkStatusHighMs);
		return false;
	}

	bool checkNeeded{false};
	if (xTaskCheckForTimeOut(&phyLinkStatusTimer, &phyLinkStatusRemaining))
	{
		PHYInterface::LinkStatus phyLinkStatus = PHY.readLinkStatus();
		if (lastPhyLinkStatus != phyLinkStatus)
		{
			lastPhyLinkStatus = phyLinkStatus;
			if (phyLinkStatus == modm::PHYInterface::LinkStatus::Down)
			{
				IPStackEvent_t xRxEvent = {eNetworkDownEvent, NULL};
				xSendEventStructToIPTask(&xRxEvent, 0);
			}
			checkNeeded = true;
		}

		vTaskSetTimeOutState(&phyLinkStatusTimer);
		if (phyLinkStatus == modm::PHYInterface::LinkStatus::Up)
			phyLinkStatusRemaining = pdMS_TO_TICKS(PhyLinkStatusHighMs);
		else
			phyLinkStatusRemaining = pdMS_TO_TICKS(PhyLinkStatusLowMs);
	}

	return checkNeeded;
}

void
ethernet::emacHandlerTask(void *)
{
	using modm::platform::eth;

	static constexpr TickType_t maxBlockTime{pdMS_TO_TICKS(100)};

	UBaseType_t lastMinBufferCount{0};
	UBaseType_t currentCount{0};
	bool result{false};

	for (;;)
	{
		result = false;
		currentCount = uxGetMinimumFreeNetworkBuffers();
		if (lastMinBufferCount != currentCount)
			lastMinBufferCount = currentCount;

		if (txDescriptorSemaphore)
		{
			static UBaseType_t lowestSemCount = TX_BUFFER_NUMBER - 1;
			currentCount = uxSemaphoreGetCount(txDescriptorSemaphore);
			if (lowestSemCount > currentCount)
				lowestSemCount = currentCount;
		}

		if (isrEvent == eth::Event(0))
			ulTaskNotifyTake(pdFALSE, maxBlockTime);
		else
		{
			if ((isrEvent & eth::Event::Receive) == eth::Event::Receive)
			{
				isrEvent = isrEvent & ~eth::Event::Receive;
				result = emacInterfaceInput();
			}
			if ((isrEvent & eth::Event::Transmit) == eth::Event::Transmit)
			{
				isrEvent = isrEvent & ~eth::Event::Transmit;
				clearTxBuffers();
			}
			if ((isrEvent & eth::Event::Error) == eth::Event::Error)
			{
				isrEvent = isrEvent & ~eth::Event::Error;
			}
		}

		// check link status
		if (phyCheckLinkStatus(result))
			updateConfig(false);
	}
}


extern "C" BaseType_t
xNetworkInterfaceInitialise()
{
	using modm::NetworkInterface::ethernet;

	if (ethernet::initStatus == ethernet::InitStatus::Init)
	{
		ethernet::txDescriptorSemaphore = xSemaphoreCreateCounting(UBaseType_t(ethernet::TX_BUFFER_NUMBER),
																   UBaseType_t(ethernet::TX_BUFFER_NUMBER));
		if (ethernet::txDescriptorSemaphore == NULL)
		{
			ethernet::initStatus = ethernet::InitStatus::Failed;
			return pdFAIL;
		}

		EMAC::setMacAddress(EMAC::MacAddressIndex::Index0, FreeRTOS_GetMACAddress());
#if (ipconfigUSE_LLMNR != 0)
		/* Program the LLMNR address at index 1. */
		EMAC::setMacAddress(EMAC::MacAddressIndex::Index1,
							reinterpret_cast<uint8_t const *>(xLLMNR_MACAddress));
#endif

		(void)EMAC::initialize<modm::platform::eth::MediaInterface::RMII>();
		PHY.initialize();

		ethernet::TxDescriptor = ethernet::DmaTxDescriptorTable;
		ethernet::RxDescriptor = ethernet::DmaRxDescriptorTable;

		std::memset(&ethernet::DmaTxDescriptorTable, 0, sizeof(ethernet::DmaTxDescriptorTable));
		std::memset(&ethernet::DmaRxDescriptorTable, 0, sizeof(ethernet::DmaRxDescriptorTable));

		ethernet::DmaTxDescriptorToClear = ethernet::DmaTxDescriptorTable;

		ethernet::DMATxDescListInit();
		ethernet::DMARxDescListInit();

		ethernet::updateConfig(true);

		if (not xTaskCreate(ethernet::emacHandlerTask, "EMAC", ethernet::emacTaskStackDepth, NULL,
							ethernet::emacTaskPriority, &ethernet::emacTaskHandle))
		{
			ethernet::initStatus = ethernet::InitStatus::Failed;
			return pdFAIL;
		}

		ethernet::initStatus = ethernet::InitStatus::Pass;
	}

	if (ethernet::initStatus != ethernet::InitStatus::Pass)
		return pdFAIL;

	if (PHY.getLinkStatus() == modm::PHYInterface::LinkStatus::Up)
	{
		EMAC::enableInterrupt(EMAC::Interrupt_t(
			//					EMAC::Interrupt::TimeStampTrigger
			//					| EMAC::Interrupt::Pmt
			//					| EMAC::Interrupt::Mmc
			EMAC::Interrupt::NormalIrqSummary | EMAC::Interrupt::EarlyReceive | EMAC::Interrupt::FatalBusError | EMAC::Interrupt::ReceiveWatchdog | EMAC::Interrupt::ReceiveStopped | EMAC::Interrupt::ReceiveBufferUnavailable | EMAC::Interrupt::Receive | EMAC::Interrupt::TransmitUnderflow | EMAC::Interrupt::ReceiveOverflow | EMAC::Interrupt::TransmitJabberTimeout | EMAC::Interrupt::TransmitStopped | EMAC::Interrupt::Transmit));
		// link is up
		return pdPASS;
	}

	return pdFAIL;
}

extern "C" BaseType_t
xNetworkInterfaceOutput(NetworkBufferDescriptor_t *const descriptor, BaseType_t releaseAfterSend)
{
	using modm::NetworkInterface::ethernet;

	static constexpr TickType_t blockTimeTicks{pdMS_TO_TICKS(50)};
	static constexpr ethernet::TDes0_t transmitStatus{
		ethernet::CrcControl_t(ethernet::CrcControl::HardwareCalculated) |
		ethernet::TDes0_t(ethernet::TDes0::InterruptOnCompletion |
						  ethernet::TDes0::LastSegment |
						  ethernet::TDes0::FirstSegment)};

	BaseType_t result{pdFAIL};
	uint32_t transmitSize{0};
	__IO ethernet::DmaDescriptor_t *dmaTxDescriptor{nullptr};

	do
	{
		ProtocolPacket_t *packet = reinterpret_cast<ProtocolPacket_t *>(descriptor->pucEthernetBuffer);
		if (packet->xICMPPacket.xIPHeader.ucProtocol == ipPROTOCOL_ICMP)
			packet->xICMPPacket.xICMPHeader.usChecksum = 0;

		if (PHY.getLinkStatus() == modm::PHYInterface::LinkStatus::Down)
			// no link, drop packet
			break;

		if (xSemaphoreTake(ethernet::txDescriptorSemaphore, blockTimeTicks) != pdPASS)
			break;

		dmaTxDescriptor = ethernet::TxDescriptor;
		configASSERT((dmaTxDescriptor->Status & uint32_t(ethernet::TDes0::DmaOwned)) == 0);

		transmitSize = descriptor->xDataLength;
		if (transmitSize > ethernet::TX_BUFFER_SIZE)
			transmitSize = ethernet::TX_BUFFER_SIZE;

		configASSERT(releaseAfterSend != 0);

		dmaTxDescriptor->Buffer1Addr = uint32_t(descriptor->pucEthernetBuffer);
		releaseAfterSend = pdFALSE_SIGNED;

		dmaTxDescriptor->Status |= transmitStatus.value;

		dmaTxDescriptor->ControlBufferSize = transmitSize & modm::NetworkInterface::ethernet::Buffer1SizeMask;

		dmaTxDescriptor->Status |= uint32_t(ethernet::TDes0::DmaOwned);
		ethernet::TxDescriptor = reinterpret_cast<ethernet::DmaDescriptor_t *>(ethernet::TxDescriptor->Buffer2NextDescAddr);
		__DSB();
		ETH->DMATPDR = 0;
		iptraceNETWORK_INTERFACE_TRANSMIT();
		result = pdPASS;
	} while (0);

	if (releaseAfterSend)
		vReleaseNetworkBufferAndDescriptor(descriptor);

	return result;
}

extern "C" BaseType_t xGetPhyLinkStatus()
{
	return PHY.getLinkStatus() == modm::PHYInterface::LinkStatus::Up ? pdTRUE : pdFALSE;
}

MODM_ISR(ETH)
{
	using modm::NetworkInterface::ethernet;
	using modm::platform::eth;

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	EMAC::InterruptFlags_t irq = EMAC::getInterruptFlags();
	EMAC::acknowledgeInterrupt(irq);

	if (irq & (eth::InterruptFlags::Receive | eth::InterruptFlags::ReceiveBufferUnavailable))
	{
		ethernet::isrEvent |= eth::Event::Receive;
		if (ethernet::emacTaskHandle)
		{
			vTaskNotifyGiveFromISR(ethernet::emacTaskHandle, &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
	}
	if (irq & (eth::InterruptFlags::Transmit))
	{
		ethernet::isrEvent |= eth::Event::Transmit;
		if (ethernet::emacTaskHandle)
		{
			vTaskNotifyGiveFromISR(ethernet::emacTaskHandle, &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
	}

	if (EMAC::getInterruptFlags() & eth::InterruptFlags::AbnormalIrqSummary)
	{
		// not used yet
	}
}


