/**
 * @file ethernet.hpp
 * @author Mattis Kieffer (mattis.kieffer@hotmail.de)
 * @brief communication via raw etherent frames.
 * @version 0.1
 * @date 2023-02-17
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef MODM_ETHERNET_TRANSPORT_HPP
#define MODM_ETHERNET_TRANSPORT_HPP

#include <modm/communication/ethernet/api_concepts.hpp>
#include <modm/platform/eth/eth.hpp>
#include <etl/array.h>
#include <etl/queue.h>

/**
 * NOTES:
 *  Implement Mac Filter
 * Ethernet If has MacFilters
 * MacFilters can have exactly one Handle or no Handle
 * A MacFilter can have one notification CB
 * Mac Filter has a fifo of pointers to RxDMA descriptors to empty
 * RX:
 *      RxDescriptor -(BufferHandle)-> MacFilter -(BufferHandle)-> MacFilterHandle.getRxFrame()
 *      - Rx Interrupt triggered
 *      - check dest. address against MacFilters
 *      - push reveived Descriptor references to Mac Filter FIFO
 *      - notify subscribers from ISR
 *      - subscriber retrieves Frames from FIFO (he has to!)
 *      - on getRXFrame we move the RxDescriptors BufferHandle out, assign a New one, reset DMAOwned and move-return the old one we moved out.
 *      - hasRxFrames queries the FIFO of the MacFIlter
 *
 * LINKSTATUS:
 *      - isConnected gets polled periodically by the NetworkInterface to determine LinkStatus
 *      - eNetwork Events get triggered from there accordingly
 *      - other users can just poll the EthernetTransport
 * TODO: add a notification Callback registry for link status changes to PHY / EthernetTransport
 *
 * HAL:
 *      - remove the Phy/LinkStatus specific functions
 *          - replace them with the configureMac(DuplexMode, Speed)
 *      - fix the ReadPhyRegister ans write... functions
 * PHY:
 *      - implement the nucleo phy with the new logic.
 */

namespace modm::ethernet
{
    // typedef etl::array<uint8_t, 6> MacAddr_t;
    struct Configuration
    {
        modm::platform::eth::MacAddr_t macAddress;
        // mac address
        // filtering
        // hardware offloading
    };

    /// @ingroup modm_platform_eth

    template <api::BufferHandle BufferHandle, size_t TxDescriptorCount, size_t RxDescriptorCount, size_t MacFilterCount, api::PhyAbstractionLayer Phy>
    class EthernetTransport
    {

    public:
        typedef void (*notificationCB_t)(void);

    private:
        struct MacFilter
        {
            notificationCB_t m_rxNotify{};
            modm::platform::eth::MacAddr_t m_macAddress{};
            etl::queue<BufferHandle, RxDescriptorCount> m_rxHandles{};
            bool inUse{false};
            // enqueue a copy of the handle if the dest mac matches
            // execute notification if set.
        };

        typedef etl::array<BufferHandle, TxDescriptorCount> TxBufferHandleTable_t;
        typedef etl::array<BufferHandle, RxDescriptorCount> RxBufferHandleTable_t;
        modm_aligned(32) typedef etl::array<modm::platform::eth::TxDmaDescriptor, TxDescriptorCount> TxDescriptorTable_t;
        modm_aligned(32) typedef etl::array<modm::platform::eth::RxDmaDescriptor, RxDescriptorCount> RxDescriptorTable_t;
        static etl::array<MacFilter, 4> m_macFilters;
        static TxBufferHandleTable_t TxBufferHandles;
        static RxBufferHandleTable_t RxBufferHandles;
        static TxDescriptorTable_t TxDescriptorTable;
        static RxDescriptorTable_t RxDescriptorTable;
        static etl::array<notificationCB_t, TxDescriptorCount> TxCompleteCallbacks;
        static size_t TxTableCleanupIndex;
        static size_t TxTableNotifiedIndex;
        static size_t TxTableTxIndex;
        static size_t RxTableRxIndex;

        static void setupTxDmaDescriptor(size_t index, void *bufferAddress, size_t bufferSize)
        {
            using namespace modm::platform;
            static constexpr eth::CrcControl_t checksumControl(eth::CrcControl::HardwareCalculated);
            static constexpr eth::TDes0_t DefaultTDes0{eth::TDes0::FirstSegment | eth::TDes0::SecondAddressChained | eth::TDes0::InterruptOnCompletion | eth::TDes0::TransmitTimestampEnable | eth::TDes0::LastSegment | checksumControl};
            size_t nextDescriptor = (index + 1) % TxDescriptorCount;
            TxDescriptorTable[index].BufferSize = eth::TxBuffer1Size_t(bufferSize).value;
            TxDescriptorTable[index].Buffer1Addr = (uint32_t)bufferAddress;
            TxDescriptorTable[index].Buffer2NextDescAddr = (uint32_t)&TxDescriptorTable[nextDescriptor];
            TxDescriptorTable[index].Status = DefaultTDes0.value;
            if (bufferAddress != nullptr)
            {
                TxDescriptorTable[index].Status |= uint32_t(eth::TDes0::DmaOwned);
            }
        }
        static void setupRxDmaDescriptor(size_t index, void *bufferAddress, size_t bufferSize)
        {
            using namespace modm::platform;
            static constexpr eth::RDes1_t DefaultRDes1{eth::RDes1::SecondAddressChained};
            size_t nextDescriptor = (index + 1) % RxDescriptorCount;

            RxDescriptorTable[index].ControlBufferSize = (DefaultRDes1 | eth::RxBuffer1Size_t(bufferSize)).value;
            RxDescriptorTable[index].Buffer1Addr = (uint32_t)bufferAddress;
            RxDescriptorTable[index].Buffer2NextDescAddr = (uint32_t)&RxDescriptorTable[nextDescriptor];
            RxDescriptorTable[index].Status = uint32_t(eth::RDes0::DmaOwned);
        }

    public:
        /// @brief struct for tx status information and timestamps
        struct TxStatus;

        /// reference counting handle to manage source address filtering
        class MacFilterHandle
        {
            MacFilter *m_filter{nullptr};

        public:
            MacFilterHandle(MacFilter *filter = nullptr) : m_filter(filter)
            {
                if (filter != nullptr)
                {
                    filter->inUse = true;
                }
            };
            bool hasRXFrame()
            {
                processReceivedFrames();
                if (m_filter == nullptr)
                    return false;
                bool hasD = m_filter->m_rxHandles.size() > 0;
                return hasD;
            };
            /**
             * @brief get the next RX Framebuffer handle
             *  this is the RecieveFrame in the procedual IEEE802.3 model
             * @return DMADescriptorHandle
             */
            BufferHandle getRXFrame()
            {
                if (not hasRXFrame())
                    return {};

                BufferHandle buffer(std::move(m_filter->m_rxHandles.front()));
                m_filter->m_rxHandles.pop();
                return buffer;
            };

            /**
             * @brief reset this handle to empty. (unsubscribe)
             *
             */
            void reset()
            {
                if (m_filter == nullptr)
                    return;
                /// TODO: unregister from HW mac filter!!!
                m_filter = nullptr;
            };
        };

        static bool isConnected();

        static modm::ethernet::api::LinkStatus getLinkStatus()
        {
            return Phy::getLinkStatus();
        };

        static void rxIsrNotifier()
        {
            using namespace modm::platform;
            size_t notI = RxTableRxIndex;
            while (eth::RDes0_t(RxDescriptorTable[notI].Status).none(eth::RDes0::DmaOwned))
            {
                notI = (notI + 1) % RxDescriptorCount;
                for (auto &filter : m_macFilters)
                {
                    // if we have data and the mac matches, add to macfilters rx queue
                    // if (byteCount>0 && memcmp(filter.m_macAddress.data(), buffer.data().data(), sizeof(MacAddr_t)) == 0)
                    //{
                    if (filter.m_rxNotify != nullptr)
                    {
                        filter.m_rxNotify();
                    }
                    //}
                };
                // buffer goes out of scope decrementing the ref count.
            };
        };

        static void initializeDmaTables()
        {
            using namespace modm::platform;
            size_t idx{0};
            for (idx = 0; idx < TxDescriptorCount; ++idx)
            {
                setupTxDmaDescriptor(idx, nullptr, 0);
            }
            for (idx = 0; idx < RxDescriptorCount; ++idx)
            {
                RxBufferHandles[idx] = BufferHandle(1536);
                auto &buffer(RxBufferHandles[idx]);
                setupRxDmaDescriptor(idx, buffer.data().data(), buffer.size());
            }
        }

        static bool initialize(const Configuration &cfg)
        {

            using namespace modm::platform;
            eth::MacConfiguration_t macCfg{
                eth::MacConfiguration::AutomaticPad | eth::MacConfiguration::Ipv4ChecksumOffLoad | eth::MacConfiguration::RetryDisable | eth::Speed_t(eth::Speed::Speed100M) | eth::DuplexMode_t(eth::DuplexMode::Full)};

            eth::MacFlowControl_t macFcr{
                eth::MacFlowControl::ZeroQuantaPauseDisable};

            eth::MacFrameFilter_t macFfr{
                eth::MacFrameFilter::HashOrPerfect | eth::PassControlFrame_t(eth::PassControlFrame::BlockAll) | eth::MacFrameFilter::PassAllMulticast};

            Eth::configureMac(macCfg, macFfr, macFcr);

            Eth::setMacFilter(eth::MacAddressIndex::Index0, cfg.macAddress);
            memcpy(m_macFilters[0].m_macAddress, cfg.macAddress, sizeof(modm::platform::eth::MacAddr_t));

            initializeDmaTables();
            Eth::setDmaRxDescriptorTable(RxDescriptorTable.data());
            Eth::setDmaTxDescriptorTable(TxDescriptorTable.data());

            eth::DmaOperationMode_t dmaOmr{
                eth::DmaOperationMode::ReceiveStoreAndForward | eth::DmaOperationMode::TransmitStoreAndForward | eth::DmaOperationMode::OperateOnSecondFrame};

            eth::DmaBusMode_t dmaBmr{
                eth::DmaBusMode::EnhancedDescFormat | eth::DmaBusMode::AddressAlignedBeats | eth::DmaBusMode::FixedBurst | eth::DmaBusMode::UseSeparatePbl | eth::BurstLength_t(eth::BurstLength::Length32Beats) | eth::RxDmaBurstLength_t(eth::BurstLength::Length32Beats)};

            Eth::configureDma(dmaBmr, dmaOmr);
            eth::DmaInterruptEnable_t interrupts{
                eth::DmaInterruptEnable::NormalIrqSummary | eth::DmaInterruptEnable::Receive | eth::DmaInterruptEnable::Transmit};
            Eth::enableInterrupt(eth::DmaInterruptEnable::Receive, rxIsrNotifier);
            Eth::enableInterrupt(eth::DmaInterruptEnable::Transmit, txCompleteCallbackProcessing);
            Eth::start();
            return true;
        };

        /// @brief subscribe to incoming ethernet frames with the devices source Mac addr.
        /// @param callback that notifies the subscribing component of the inbound packet
        /// @return the handle to the MacFilter for the default address
        static MacFilterHandle subscribe(notificationCB_t callback = nullptr)
        {
            m_macFilters[0].m_rxNotify = callback;
            return MacFilterHandle(&m_macFilters[0]);
        };

        /// @brief subscribe to incoming ethernet frames with the specified source Mac addr.
        /// @param callback that notifies the subscribing component of the inbound packet
        /// @return the handle to the MacFilter for the subscribed mac/ multicast/broadcast
        static MacFilterHandle subscribeToMacAddress(modm::platform::eth::MacAddr_t mac, notificationCB_t callback = nullptr);

        /// @brief transmit a Ethernet frame. you are responsible for the header!
        /// @param buffer the buffer containing the frame to transmit
        /// @param callback notification executed on the transmit complete interrupt.
        /// @return
        static bool transmitFrame(BufferHandle &&buffer, notificationCB_t callback = nullptr)
        {
            using namespace modm::platform;
            TxCompleteCallbacks[TxTableTxIndex] = callback;
            setupTxDmaDescriptor(TxTableTxIndex, buffer.data().data(), buffer.size());
            TxBufferHandles[TxTableTxIndex] = std::move(buffer);
            TxTableTxIndex = (TxTableTxIndex + 1) % TxDescriptorCount;
            // force descriptor poll by dma to make it recognize the pending transmission
            __DSB();
            ETH->DMATPDR = 0;
            return true;
        };

        static bool transmitFrameUnsafe(BufferHandle::pointer_type buffer, notificationCB_t callback = nullptr)
        {

            using namespace modm::platform;

            TxCompleteCallbacks[TxTableTxIndex] = callback;
            setupTxDmaDescriptor(TxTableTxIndex, BufferHandle::getPayloadAddress(buffer), BufferHandle::getPayloadSizeBytes(buffer));
            TxTableTxIndex = (TxTableTxIndex + 1) % TxDescriptorCount;
            // force descriptor poll by dma to make it recognize the pending transmission
            __DSB();
            ETH->DMATPDR = 0;
            return true;
        };

        /// @brief get the Transmission status for a given Buffer
        /// @param buffer
        /// @return
        static TxStatus getTransmitStatus(const BufferHandle &buffer);
        static TxStatus getTransmitStatus(const BufferHandle::pointer_type buffer);

        static void txCompleteCallbackProcessing()
        {
            while (TxTableNotifiedIndex != TxTableTxIndex)
            {
                if (TxCompleteCallbacks[TxTableNotifiedIndex] != nullptr)
                {
                    TxCompleteCallbacks[TxTableNotifiedIndex]();
                }
                TxTableNotifiedIndex = (TxTableNotifiedIndex + 1) % TxDescriptorCount;
            }
        }

        /// @brief reset the BufferHandles for all finished transmissions might free the buffer.
        static void clearFinishedTransmissions()
        {
            using namespace modm::platform;
            while (TxTableCleanupIndex != TxTableTxIndex)
            {
                if (eth::TDes0_t(TxDescriptorTable.at(TxTableCleanupIndex).Status).none(modm::platform::eth::TDes0::DmaOwned))
                {
                    TxBufferHandles[TxTableCleanupIndex].reset();
                    TxTableCleanupIndex = (TxTableCleanupIndex + 1) % TxDescriptorCount;
                }
                else
                {
                    return;
                }
            }
        };

        /// @brief forward the BufferHandles of the receiverd Frames to the matching MacFilters
        /// might allocate and / or free buffers
        static void processReceivedFrames()
        {
            using namespace modm::platform;
            while (eth::RDes0_t(RxDescriptorTable[RxTableRxIndex].Status).none(eth::RDes0::DmaOwned))
            {
                // store copy of buffer (increase ref count)
                BufferHandle buffer(std::move(RxBufferHandles[RxTableRxIndex]));

                buffer.resize(eth::RxFrameLength_t::get(eth::RDes0_t(RxDescriptorTable[RxTableRxIndex].Status)));
                // assign new buffer
                RxBufferHandles[RxTableRxIndex] = BufferHandle(1536);
                setupRxDmaDescriptor(RxTableRxIndex, RxBufferHandles[RxTableRxIndex].data().data(), 1536);
                // increment index wrap around when end is reached
                RxTableRxIndex = (RxTableRxIndex + 1) % RxDescriptorCount;
                static constexpr eth::MacAddr_t bcMac{0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
                if (buffer.hasData() && m_macFilters[0].inUse && memcmp(bcMac, buffer.data().data(), sizeof(eth::MacAddr_t)) == 0)
                {
                    m_macFilters[0].m_rxHandles.emplace(std::move(buffer));
                    return;
                }

                for (MacFilter &filter : m_macFilters)
                {
                    // if we have data and the mac matches, add to macfilters rx queue
                    if (buffer.hasData() && memcmp(filter.m_macAddress, buffer.data().data(), sizeof(eth::MacAddr_t)) == 0)
                    {
                        if (filter.inUse)
                        {
                            filter.m_rxHandles.emplace(std::move(buffer));
                            return;
                        }
                    }
                };
                // buffer goes out of scope decrementing the ref count.
            };
        }
    };

} // namespace modm::communication
#include <modm/communication/ethernet/ethernet_impl.hpp>
#endif