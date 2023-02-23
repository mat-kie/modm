#ifndef MODM_ETHERNET_TRANSPORT_HPP
#include <modm/communication/ethernet.hpp>
#else
namespace modm::ethernet{
template <api::BufferHandle BufferManager, size_t TxDescriptorCount, size_t RxDescriptorCount, size_t MacFilterCount, api::PhyAbstractionLayer Phy>
size_t EthernetTransport<BufferManager, TxDescriptorCount, RxDescriptorCount, MacFilterCount, Phy>::TxTableCleanupIndex{0};

template <api::BufferHandle BufferManager, size_t TxDescriptorCount, size_t RxDescriptorCount, size_t MacFilterCount, api::PhyAbstractionLayer Phy>
size_t EthernetTransport<BufferManager, TxDescriptorCount, RxDescriptorCount, MacFilterCount, Phy>::TxTableNotifiedIndex{0};

template <api::BufferHandle BufferManager, size_t TxDescriptorCount, size_t RxDescriptorCount, size_t MacFilterCount, api::PhyAbstractionLayer Phy>
size_t EthernetTransport<BufferManager, TxDescriptorCount, RxDescriptorCount, MacFilterCount, Phy>::TxTableTxIndex{0};

template <api::BufferHandle BufferManager, size_t TxDescriptorCount, size_t RxDescriptorCount, size_t MacFilterCount, api::PhyAbstractionLayer Phy>
size_t EthernetTransport<BufferManager, TxDescriptorCount, RxDescriptorCount, MacFilterCount, Phy>::RxTableRxIndex{0};

template <api::BufferHandle BufferManager, size_t TxDescriptorCount, size_t RxDescriptorCount, size_t MacFilterCount, api::PhyAbstractionLayer Phy>
EthernetTransport<BufferManager, TxDescriptorCount, RxDescriptorCount, MacFilterCount, Phy>::TxBufferHandleTable_t
EthernetTransport<BufferManager, TxDescriptorCount, RxDescriptorCount, MacFilterCount, Phy>::TxBufferHandles{};

template <api::BufferHandle BufferManager, size_t TxDescriptorCount, size_t RxDescriptorCount, size_t MacFilterCount, api::PhyAbstractionLayer Phy>
EthernetTransport<BufferManager, TxDescriptorCount, RxDescriptorCount, MacFilterCount, Phy>::RxBufferHandleTable_t
EthernetTransport<BufferManager, TxDescriptorCount, RxDescriptorCount, MacFilterCount, Phy>::RxBufferHandles{};


template <api::BufferHandle BufferManager, size_t TxDescriptorCount, size_t RxDescriptorCount, size_t MacFilterCount, api::PhyAbstractionLayer Phy>
EthernetTransport<BufferManager, TxDescriptorCount, RxDescriptorCount, MacFilterCount, Phy>::TxDescriptorTable_t
EthernetTransport<BufferManager, TxDescriptorCount, RxDescriptorCount, MacFilterCount, Phy>::TxDescriptorTable{};

template <api::BufferHandle BufferManager, size_t TxDescriptorCount, size_t RxDescriptorCount, size_t MacFilterCount, api::PhyAbstractionLayer Phy>
EthernetTransport<BufferManager, TxDescriptorCount, RxDescriptorCount, MacFilterCount, Phy>::RxDescriptorTable_t
EthernetTransport<BufferManager, TxDescriptorCount, RxDescriptorCount, MacFilterCount, Phy>::RxDescriptorTable{};

template <api::BufferHandle BufferManager, size_t TxDescriptorCount, size_t RxDescriptorCount, size_t MacFilterCount, api::PhyAbstractionLayer Phy>
etl::array<typename EthernetTransport<BufferManager, TxDescriptorCount, RxDescriptorCount, MacFilterCount, Phy>::notificationCB_t, TxDescriptorCount>
EthernetTransport<BufferManager, TxDescriptorCount, RxDescriptorCount, MacFilterCount, Phy>::TxCompleteCallbacks{};

template <api::BufferHandle BufferManager, size_t TxDescriptorCount, size_t RxDescriptorCount, size_t MacFilterCount, api::PhyAbstractionLayer Phy>
etl::array<typename EthernetTransport<BufferManager, TxDescriptorCount, RxDescriptorCount, MacFilterCount, Phy>::MacFilter, 4>
EthernetTransport<BufferManager, TxDescriptorCount, RxDescriptorCount, MacFilterCount, Phy>::m_macFilters{};


};

#endif

