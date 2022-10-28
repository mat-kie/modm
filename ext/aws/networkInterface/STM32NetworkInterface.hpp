#ifndef FREERTOS_STM32_NETWORK_INTERFACE
#define FREERTOS_STM32_NETWORK_INTERFACE
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


#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"
#include "FreeRTOS_IP_Private.h"
#include "FreeRTOS_DNS.h"
#include "FreeRTOS_ARP.h"
#include "NetworkBufferManagement.h"

#include <modm/driver/ethernet/IPHY.hpp>
#include <cstring>

namespace modm::NetworkInterface
{
	extern PHYInterface& PHY;

	struct ethernet
	{

		enum class
			InitStatus : uint8_t
		{
			Init,
			Pass,
			Failed
		};
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

		static constexpr BaseType_t MAX_PACKET_SIZE{1536};
		static constexpr BaseType_t RX_BUFFER_SIZE{1536};
		static constexpr BaseType_t TX_BUFFER_SIZE{1536};
		static constexpr BaseType_t RX_BUFFER_NUMBER{5};
		static constexpr BaseType_t TX_BUFFER_NUMBER{5};

		static constexpr configSTACK_DEPTH_TYPE emacTaskStackDepth{configMINIMAL_STACK_SIZE * 2};
		static constexpr UBaseType_t emacTaskPriority{configMAX_PRIORITIES - 1};

		static constexpr TickType_t PhyLinkStatusHighMs{pdMS_TO_TICKS(2'000)};
		static constexpr TickType_t PhyLinkStatusLowMs{pdMS_TO_TICKS(1'000)};


		static constexpr uint32_t ReceiveDescriptorFrameLengthMask{0x3fff0000};
		static constexpr uint32_t ReceiveDescriptorFrameLengthShift{16};
		static constexpr uint32_t Buffer1SizeMask{0x00001fff};
		static constexpr uint32_t Buffer2SizeMask{0x1fff0000};

		static InitStatus initStatus;
		static SemaphoreHandle_t txDescriptorSemaphore;
		static TaskHandle_t emacTaskHandle;

		static modm::platform::eth::Event_t isrEvent;

		static TimeOut_t phyLinkStatusTimer;
		static TickType_t phyLinkStatusRemaining;
		static modm::PHYInterface::LinkStatus lastPhyLinkStatus;


		/* Ethernet Rx DMA Descriptor */
		modm_aligned(32) static DmaDescriptor_t DmaRxDescriptorTable[RX_BUFFER_NUMBER];

		/* Ethernet Tx DMA Descriptor */
		modm_aligned(32) static DmaDescriptor_t DmaTxDescriptorTable[TX_BUFFER_NUMBER];

		static DmaDescriptor_t *RxDescriptor; /*!< Rx descriptor to Get        */
		static DmaDescriptor_t *TxDescriptor; /*!< Tx descriptor to Set        */
		static DmaDescriptor_t *DmaTxDescriptorToClear;

		static void
		DMATxDescListInit();
		static void
		DMARxDescListInit();
		static void
		clearTxBuffers();
		static bool
		mayAcceptPacket(uint8_t *buffer);
		// check if the packet would be accepted
		static void
		passMessage(NetworkBufferDescriptor_t *descriptor);

		static bool
		emacInterfaceInput();

		static void
		updateConfig(bool force);

		static bool
		phyCheckLinkStatus(bool hasReceived);
		static void
		emacHandlerTask(void *);
	};

} // namespace modm

#endif