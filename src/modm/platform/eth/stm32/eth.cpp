#include <modm/platform.hpp>
using namespace modm::platform;

Eth::IsrCallbacks_t Eth::EthIsrCallbacks{};
void Eth::enableInterrupt(eth::DmaInterruptEnable irq, notificationCB_t callback)
{
    DmaInterruptEnable_t dmaIer(irq);

    if (irq == DmaInterruptEnable::Transmit)
    {
        EthIsrCallbacks.Transmit = callback;
        dmaIer |= DmaInterruptEnable::NormalIrqSummary;
    }
    else if (irq == DmaInterruptEnable::Receive)
    {
        EthIsrCallbacks.Receive = callback;
        dmaIer |= DmaInterruptEnable::NormalIrqSummary;
    }
    ETH->DMAIER |= dmaIer.value;
};

// enable the mac interrupt irq
void Eth::enableInterrupt(eth::MacInterruptMask irq, notificationCB_t callback)
{
    MacInterruptMask_t macImr(ETH->MACIMR);

    if (irq == MacInterruptMask::TimestampTrigger)
    {
        EthIsrCallbacks.TimestampTrigger = callback;
        macImr.reset(irq);
    }
    writeSafe(macImr);
}

void Eth::NotifyFromISR(DmaStatus s)
{
    if (EthIsrCallbacks.Receive != nullptr && s == DmaStatus::Receive)
    {
        EthIsrCallbacks.Receive();
        return;
    }
    else if (EthIsrCallbacks.Transmit != nullptr && s == DmaStatus::Transmit)
    {
        EthIsrCallbacks.Transmit();
        return;
    }
    else if (EthIsrCallbacks.TimestampTrigger != nullptr && s == DmaStatus::TimeStampTrigger)
    {
        EthIsrCallbacks.TimestampTrigger();
        return;
    }
}

MODM_ISR(ETH)

{
    using modm::platform::eth;

    eth::DmaStatus_t irq = Eth::getInterruptFlags();
    Eth::acknowledgeInterrupt(irq);
    if (irq.any(eth::DmaStatus::Receive))
    {
        Eth::NotifyFromISR(eth::DmaStatus::Receive);
    }
    if (irq.any(eth::DmaStatus::Transmit))
    {
        Eth::NotifyFromISR(eth::DmaStatus::Transmit);
    }
    if (irq.any(eth::DmaStatus::TimeStampTrigger))
    {
        Eth::NotifyFromISR(eth::DmaStatus::TimeStampTrigger);
    }
}