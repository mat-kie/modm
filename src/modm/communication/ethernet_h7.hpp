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
/**
 * NOTES:
 * reset Context Descriptor in Rx Handler to normal.
 *
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
        static inline etl::array<MacFilter, 4> m_macFilters{};
        static inline TxBufferHandleTable_t TxBufferHandles{};
        static inline RxBufferHandleTable_t RxBufferHandles{};
        static inline TxDescriptorTable_t TxDescriptorTable{};
        static inline RxDescriptorTable_t RxDescriptorTable{};
        static inline etl::array<notificationCB_t, TxDescriptorCount> TxCompleteCallbacks{};
        static inline size_t TxTableCleanupIndex{};
        static inline size_t TxTableNotifiedIndex{};
        static inline size_t TxTableTxIndex{};
        static inline size_t RxTableRxIndex{};

        static void setupTxDmaDescriptor(size_t index, void *bufferAddress, size_t bufferSize)
        {
            using namespace modm::platform;
            static constexpr eth::CrcPadControl_t ethCrcControl(eth::CrcPadControl::AppendPadAndCrc);
            static constexpr eth::IPChecksumControl_t IPchecksumControl(eth::IPChecksumControl::HardwareCalculated);
            eth::NormalTxDmaDescriptor Descriptor{
                .buffer1Address = (uint32_t)bufferAddress,
                .buffer2Address = 0,
                .controlBufferSize = eth::NormalTDes2_t(eth::NormalTDes2::InterruptOnCompletion) | eth::TxBuffer1Size_t(bufferSize),
                .control = eth::NormalTDes3_t(eth::NormalTDes3::FirstDescriptor | eth ::NormalTDes3::LastDescriptor | eth::NormalTDes3::DmaOwned) | ethCrcControl | IPchecksumControl};

            if (bufferAddress != nullptr)
            {
                TxDescriptorTable[index] = Descriptor;
            }
        }
        static void setupRxDmaDescriptor(size_t index, void *bufferAddress)
        {
            using namespace modm::platform;
            eth::NormalRxDmaDescriptor descriptor{
                .buffer1Address = (uint32_t)bufferAddress,
                ._reserved{},
                .buffer2Address = 0,
                .control = eth::NormalRDes3_t(eth::NormalRDes3::Buffer1AddressValid | eth::NormalRDes3::EnableInterruptOnCompletition | eth::NormalRDes3::DmaOwned)};
            RxDescriptorTable[index] = descriptor;
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
            // while (RxDescriptorTable[notI].getType() != eth::DescriptorType::Normal)
            //{
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
            //};
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
                setupRxDmaDescriptor(idx, buffer.data().data());
            }
        }

        static bool initialize(const Configuration &cfg)
        {

            using namespace modm::platform;
            eth::MacConfiguration_t macCfg{eth::MacConfiguration::FullDuplexMode | eth::MacConfiguration::FastEthernetSpeed | eth::MacConfiguration::DisableRetry | eth::MacConfiguration::AutoCrcStripping};
            eth::TxQueueFlowControl_t macFcr{
                eth::TxQueueFlowControl::DisableZeroQuantaPause};

            eth::MacPacketFilteringControl_t macFfr{
                eth::MacPacketFilteringControl::HashOrPerfectFilter | eth::MacPacketFilteringControl::PassAllMulticast | eth::PassControlPacket_t(eth::PassControlPacket::BlockAll)};

            memcpy(m_macFilters[0].m_macAddress, cfg.macAddress, sizeof(modm::platform::eth::MacAddr_t));
            Eth::setMacFilter(eth::MacAddressIndex::MacAddress0, cfg.macAddress);
            Eth::configureMac(macCfg, macFfr, macFcr);

            initializeDmaTables();
            Eth::configureDma(TxDescriptorTable, RxDescriptorTable, 1536);

            Eth::enableInterrupt(eth::DmaChannelInterruptEnable::RxInt, rxIsrNotifier);
            Eth::enableInterrupt(eth::DmaChannelInterruptEnable::TxInt, txCompleteCallbackProcessing);
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
            ETH->DMACTDTPR = 0;
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
            ETH->DMACTDTPR = 0;
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
                if (TxDescriptorTable.at(TxTableCleanupIndex).getType() != eth::DescriptorType::Normal)
                {
                    TxBufferHandles[TxTableCleanupIndex].reset();
                    setupTxDmaDescriptor(TxTableCleanupIndex, nullptr, 0);
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
            eth::DescriptorType type{};
            do
            {
                type = RxDescriptorTable[RxTableRxIndex].getType();

                switch (type)
                {
                case eth::DescriptorType::Writeback:
                {
                    // move bufferhandle of this index to corresponding queue, assign new and reform normal descriptor
                    // store copy of buffer (increase ref count)
                    const eth::WritebackRxDmaDescriptor &desc(RxDescriptorTable[RxTableRxIndex].asWriteback());
                    BufferHandle buffer(std::move(RxBufferHandles[RxTableRxIndex]));
                    buffer.resize(desc.getSize());
                    RxBufferHandles[RxTableRxIndex] = BufferHandle(1536);
                    setupRxDmaDescriptor(RxTableRxIndex, RxBufferHandles[RxTableRxIndex].data().data());

                    static constexpr eth::MacAddr_t bcMac{0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
                    if (buffer.hasData() && m_macFilters[0].inUse && memcmp(bcMac, buffer.data().data(), sizeof(eth::MacAddr_t)) == 0)
                    {
                        m_macFilters[0].m_rxHandles.emplace(std::move(buffer));
                    }
                    else
                    {

                        for (MacFilter &filter : m_macFilters)
                        {
                            // if we have data and the mac matches, add to macfilters rx queue
                            if (buffer.hasData() && memcmp(filter.m_macAddress, buffer.data().data(), sizeof(eth::MacAddr_t)) == 0)
                            {
                                if (filter.inUse)
                                {
                                    filter.m_rxHandles.emplace(std::move(buffer));
                                }
                            }
                        };
                    }
                }
                break;
                case eth::DescriptorType::Context:
                    /** extract context info for prev. descriptor, reset with the old handle of this index*/
                    /// TODO: store context
                    setupRxDmaDescriptor(RxTableRxIndex, RxBufferHandles[RxTableRxIndex].data().data());
                    // reset the descriptor to be a normal one

                    break;
                case eth::DescriptorType::Normal:
                    // no more data to process
                    return;
                default:
                    setupRxDmaDescriptor(RxTableRxIndex, RxBufferHandles[RxTableRxIndex].data().data());
                    break;
                }

                RxTableRxIndex = (RxTableRxIndex + 1) % RxDescriptorCount;
            } while (type != eth::DescriptorType::Normal);
        }
    };

} // namespace modm::communication
#endif