/*
 * udpReceiver.cpp
 *
 *  Created on: May 25, 2017
 *      Author: M.Venus
 */

#include "udpReceiver.hpp"

#include <stdheader.h>
#include <stdtypes.h>

#include <facilityLayer.hpp>
#include <memory>

namespace WaveApp
{

UdpReceiver::UdpReceiver(int port, std::size_t maxSize, ReceiveCall cb, bool verbose)
    : m_verbose(verbose)
    , m_ctrlPort(port)
    , m_bufSize(maxSize)
    , m_rxBuf(std::make_unique<uint8_t[]>(m_bufSize))
    , m_rxCb(cb)
{
}

void UdpReceiver::onCtrlPoll(int fd, Int32u events)
{
    int rxLen = -1;
    if (events & POLLIN)
    {
        socklen_t fromLen = sizeof(sockaddr_in);
        rxLen = ::recvfrom(fd, m_rxBuf.get(), m_bufSize, 0, nullptr, &fromLen);
        if (rxLen <= 0)
        {
            ITSAPP_ERROR("UDP-CTRL[%u] recvfrom  %m", m_ctrlPort);
            return;
        }
        m_rxBuf[rxLen] = 0;
        if (m_verbose)
            ITSAPP_TRACE("UDP-CTRL[%u] recv %d byte(s)", m_ctrlPort, rxLen);
    }
    else if (KDBUS_POLL_ERROR(events))
    {
        if (m_ctrlSocket > 0)
            ::close(m_ctrlSocket);
        m_ctrlSocket = -1;
        ITSAPP_WRN("UDP-CTRL[%u] poll exit: 0x%08x", m_ctrlPort, events);
        return;
    }
    else
    {
        ITSAPP_WRN("UDP-CTRL[%u] poll other: 0x%08x", m_ctrlPort, events);
        return;
    }

    m_rxCb(m_rxBuf.get(), static_cast<std::size_t>(rxLen));
}

bool UdpReceiver::start()
{
    if (m_ctrlSocket > 0)
        ::close(m_ctrlSocket);
    m_ctrlSocket = -1;

    if (m_ctrlPort <= 0)
        return false;

    m_ctrlSocket = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (m_ctrlSocket < 0)
    {
        stop();
        ITSAPP_ERROR("CTRL UDP socket failed: %m");
        return false;
    }
    sockaddr_in saddr;
    memset(&saddr, 0, sizeof(sockaddr_in));
    saddr.sin_port = htons(m_ctrlPort);
    if (::bind(m_ctrlSocket, (const sockaddr*)&saddr, sizeof(sockaddr_in)) < 0)
    {
        ITSAPP_ERROR("CTRL UDP bind failed: %m");
        stop();
        return false;
    }

    m_ctrlPoll =
        siekdbus::pollConnect(m_ctrlSocket, sigc::mem_fun(*this, &UdpReceiver::onCtrlPoll), POLLIN);

    ITSAPP_LOG("Listen on CTRL port %u", m_ctrlPort);
    return true;
}

void UdpReceiver::stop()
{
    m_ctrlPoll.disconnect();
    if (m_ctrlSocket > 0)
        siekdbus::pollDisconnect(m_ctrlSocket);
    if (m_ctrlSocket > 0)
        ::close(m_ctrlSocket);
    m_ctrlSocket = -1;
}

} // namespace WaveApp
