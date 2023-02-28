#include <modm/platform.hpp>
using namespace modm::platform;

Eth::IsrCallbacks_t Eth::EthIsrCallbacks{};

void Eth::NotifyFromISR(DmaChannelStatus_t s)
{
    if (EthIsrCallbacks.Receive != nullptr && s.any(DmaChannelStatus::RxInterrupt))
    {
        EthIsrCallbacks.Receive();
    }
    if (EthIsrCallbacks.Transmit != nullptr && s.any(DmaChannelStatus::TxInterrupt))
    {
        EthIsrCallbacks.Transmit();
    }
}

void Eth::NotifyFromISR(MacInterruptStatus_t s)
{
    /// TODO: Implement
}

MODM_ISR(ETH)

{
    using modm::platform::eth;
    eth::DmaInterruptStatus_t istat = Eth::getInterruptSource();
    if(istat.any(eth::DmaInterruptStatus::DmaChan0IntStatus))
    {
        eth::DmaChannelStatus_t istat(ETH->DMACSR);
        Eth::NotifyFromISR(istat);
        Eth::acknowlegeDmaChanIntStatus(istat);
    }
    if(istat.any(eth::DmaInterruptStatus::MacIntStatus))
    {
        eth::MacInterruptStatus_t istat(ETH->MACISR);
        Eth::NotifyFromISR(istat);
        Eth::acknowlegeMacIntStatus(istat);
    }
}