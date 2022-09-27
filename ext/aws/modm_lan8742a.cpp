/*
 * Copyright (c) 2020, Mike Wolfram
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include <modm/platform.hpp>
#include <modm/driver/ethernet/LAN8742a.hpp>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"
#include "FreeRTOS_IP_Private.h"
#include "FreeRTOS_DNS.h"
#include "FreeRTOS_ARP.h"
#include "NetworkBufferManagement.h"

#include <cstring>

using EMAC = modm::platform::Eth<5>;
using EPHY = modm::LAN8742a<EMAC>;

namespace modm
{

	struct ethernet
	{
		static constexpr BaseType_t MAX_PACKET_SIZE{1536};
		static constexpr BaseType_t RX_BUFFER_SIZE{1536};
		static constexpr BaseType_t TX_BUFFER_SIZE{1536};
		static constexpr BaseType_t RX_BUFFER_NUMBER{5};
		static constexpr BaseType_t TX_BUFFER_NUMBER{5};

		static constexpr configSTACK_DEPTH_TYPE emacTaskStackDepth{configMINIMAL_STACK_SIZE * 2};
		static constexpr UBaseType_t emacTaskPriority{configMAX_PRIORITIES - 1};
		enum class
			InitStatus : uint8_t
		{
			Init,
			Pass,
			Failed
		};
		static InitStatus initStatus;
		static SemaphoreHandle_t txDescriptorSemaphore;
		static TaskHandle_t emacTaskHandle;

		static modm::platform::eth::Event_t isrEvent;

		static TimeOut_t phyLinkStatusTimer;
		static constexpr TickType_t PhyLinkStatusHighMs{pdMS_TO_TICKS(2'000)};
		static constexpr TickType_t PhyLinkStatusLowMs{pdMS_TO_TICKS(1'000)};
		static TickType_t phyLinkStatusRemaining;
		static modm::platform::eth::LinkStatus lastPhyLinkStatus;

		static void
		InitializeRecieveBuffers()
		{
			// DmaDescriptor_t *dmaDescriptor{RxDescriptor};
			uint32_t bparray[5];
			for (BaseType_t index = 0; index < RX_BUFFER_NUMBER; ++index)
			{
				NetworkBufferDescriptor_t *newBuf = pxGetNetworkBufferWithDescriptor(RX_BUFFER_SIZE, 100U);
				configASSERT(newBuf != nullptr);
				if (newBuf)
				{
					bparray[index] = uint32_t(newBuf->pucEthernetBuffer);
				}
			}
			EMAC::InitRXDescriptorTable(bparray, RX_BUFFER_SIZE);
			// EMAC::setDmaRxDescriptorTable(uint32_t(RxDescriptor));
		}

		static void
		clearTxBuffers()
		{
			std::size_t count{TX_BUFFER_NUMBER - uxSemaphoreGetCount(txDescriptorSemaphore)};
			NetworkBufferDescriptor_t *networkBuffer{nullptr};
			uint8_t *payLoad{nullptr};

			while (count > 0 && EMAC::hasFinishedTXD())
			{
				auto txcHandle = EMAC::getFinishedTXD();
				payLoad = reinterpret_cast<uint8_t *>(txcHandle.swapBufferPointer(0, 0));
				if (payLoad)
				{
					networkBuffer = pxPacketBuffer_to_NetworkBuffer(payLoad);
					if (networkBuffer)
						vReleaseNetworkBufferAndDescriptor(networkBuffer);
				}
				--count;
				xSemaphoreGive(txDescriptorSemaphore);
			}
		}

		static bool
		mayAcceptPacket(uint8_t *buffer)
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
		static void
		passMessage(NetworkBufferDescriptor_t *descriptor)
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

		static bool
		emacInterfaceInput()
		{
			static constexpr TickType_t descriptorWaitTime{pdMS_TO_TICKS(250)};

			NetworkBufferDescriptor_t *currentDescriptor{nullptr};
			NetworkBufferDescriptor_t *newDescriptor{nullptr};
			NetworkBufferDescriptor_t *firstDescriptor{nullptr};
			NetworkBufferDescriptor_t *lastDescriptor{nullptr};
			BaseType_t receivedLength{0};
			//__IO DmaDescriptor_t *dmaRxDescriptor { RxDescriptor };
			uint8_t *buffer{nullptr};

			// while ((dmaRxDescriptor->Status & uint32_t(RDes0::DmaOwned)) == 0x00000000) {
			while (EMAC::hasRXFrame())
			{
				auto rxdHandle = EMAC::getRXFrame();
				bool accepted = true;
				receivedLength = rxdHandle.getFrameLength();

				// if ((dmaRxDescriptor->Status & receiveStatus.value) != uint32_t(RDes0::EthernetFrameType))
				if (rxdHandle.hasValidEthernetFrame())
					newDescriptor = pxGetNetworkBufferWithDescriptor(RX_BUFFER_SIZE, descriptorWaitTime);
				if (not newDescriptor)
				{
					accepted = false;
				}
				else
				{
					uint32_t rxb_ptr = rxdHandle.swapBufferPointer(
						uint32_t(newDescriptor->pucEthernetBuffer),
						uint32_t(RX_BUFFER_SIZE));
					buffer = reinterpret_cast<uint8_t *>(rxb_ptr);
					accepted = mayAcceptPacket(buffer);
				}
				// RxDescriptor = reinterpret_cast<DmaDescriptor_t *>(dmaRxDescriptor->Buffer2NextDescAddr);

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

				// 7if (newDescriptor)
				// dmaRxDescriptor->Buffer1Addr = uint32_t(newDescriptor->pucEthernetBuffer);

				// dmaRxDescriptor->ControlBufferSize = uint32_t(RDes1::SecondAddressChained) |
				//		uint32_t(RX_BUFFER_SIZE);
				/// dmaRxDescriptor->Status = uint32_t(RDes0::DmaOwned);
				rxdHandle.handBackToDMA();
				__DSB();

				if (ETH->DMASR & ETH_DMASR_RBUS)
				{
					ETH->DMASR = ETH_DMASR_RBUS;
					ETH->DMARPDR = 0;
				}

				// dmaRxDescriptor = RxDescriptor;
			}

			if (firstDescriptor)
				passMessage(firstDescriptor);

			return receivedLength > 0;
		}

		static void
		updateConfig(bool force)
		{
			using namespace modm::platform;

			if (force or lastPhyLinkStatus == eth::LinkStatus::Up)
			{
				eth::ANResult negotiated = EPHY::startAutoNegotiation();
				EMAC::configureMac(negotiated);
				EMAC::start();
			}
			else
			{
				EMAC::stop();
			}
		}

		static bool
		phyCheckLinkStatus(bool hasReceived)
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
				eth::LinkStatus phyLinkStatus = EPHY::readLinkStatus();
				if (lastPhyLinkStatus != phyLinkStatus)
				{
					lastPhyLinkStatus = phyLinkStatus;
					if (phyLinkStatus == eth::LinkStatus::Down)
					{
						IPStackEvent_t xRxEvent = {eNetworkDownEvent, NULL};
						xSendEventStructToIPTask(&xRxEvent, 0);
					}
					checkNeeded = true;
				}

				vTaskSetTimeOutState(&phyLinkStatusTimer);
				if (phyLinkStatus == eth::LinkStatus::Up)
					phyLinkStatusRemaining = pdMS_TO_TICKS(PhyLinkStatusHighMs);
				else
					phyLinkStatusRemaining = pdMS_TO_TICKS(PhyLinkStatusLowMs);
			}

			return checkNeeded;
		}

		static void
		emacHandlerTask(void *)
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
	};

	ethernet::InitStatus ethernet::initStatus = ethernet::InitStatus::Init;
	SemaphoreHandle_t ethernet::txDescriptorSemaphore{nullptr};
	TaskHandle_t ethernet::emacTaskHandle{nullptr};

	modm::platform::eth::Event_t ethernet::isrEvent{modm::platform::eth::Event::None};

	TimeOut_t ethernet::phyLinkStatusTimer;
	modm::platform::eth::LinkStatus ethernet::lastPhyLinkStatus{modm::platform::eth::LinkStatus::Down};
	TickType_t ethernet::phyLinkStatusRemaining{0};

} // namespace modm

extern "C" BaseType_t
xNetworkInterfaceInitialise()
{
	using modm::ethernet;

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
		EPHY::initialize();
		EMAC::InitTXDescriptorTable();
		ethernet::InitializeRecieveBuffers();

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

	if (EPHY::getLinkStatus() == modm::platform::eth::LinkStatus::Up)
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
	using modm::ethernet;

	static constexpr TickType_t blockTimeTicks{pdMS_TO_TICKS(50)};

	BaseType_t result{pdFAIL};
	uint32_t transmitSize{0};
	// __IO ethernet::DmaDescriptor_t *dmaTxDescriptor{nullptr};

	do
	{
		ProtocolPacket_t *packet = reinterpret_cast<ProtocolPacket_t *>(descriptor->pucEthernetBuffer);
		if (packet->xICMPPacket.xIPHeader.ucProtocol == ipPROTOCOL_ICMP)
			packet->xICMPPacket.xICMPHeader.usChecksum = 0;

		if (EPHY::getLinkStatus() == modm::platform::eth::LinkStatus::Down)
			// no link, drop packet
			break;

		if (xSemaphoreTake(ethernet::txDescriptorSemaphore, blockTimeTicks) != pdPASS)
			break;

		// dmaTxDescriptor = ethernet::TxDescriptor;
		// configASSERT((dmaTxDescriptor->Status & uint32_t(ethernet::TDes0::DmaOwned)) == 0);
		configASSERT(EMAC::hasNextTXD());
		transmitSize = descriptor->xDataLength;
		if (transmitSize > ethernet::TX_BUFFER_SIZE)
			transmitSize = ethernet::TX_BUFFER_SIZE;

		configASSERT(releaseAfterSend != 0);

		// dmaTxDescriptor->Buffer1Addr = uint32_t(descriptor->pucEthernetBuffer);
		// dmaTxDescriptor->ControlBufferSize = transmitSize & modm::ethernet::Buffer1SizeMask;
		auto dHandle = EMAC::getNextTXD();
		dHandle.swapBufferPointer(
			uint32_t(descriptor->pucEthernetBuffer),
			transmitSize);
		releaseAfterSend = pdFALSE_SIGNED;
		dHandle.handBackToDMA();
		// dmaTxDescriptor->Status |= uint32_t(ethernet::TDes0::DmaOwned);
		// ethernet::TxDescriptor = reinterpret_cast<ethernet::DmaDescriptor_t *>(ethernet::TxDescriptor->Buffer2NextDescAddr);
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
	return EPHY::getLinkStatus() == modm::platform::eth::LinkStatus::Up ? pdTRUE : pdFALSE;
}

MODM_ISR(ETH)
{
	using modm::ethernet;
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
