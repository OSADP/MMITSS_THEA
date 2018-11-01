/*
 * udpReceiver.hpp
 *
 *  Created on: May 25, 2017
 *      Author: M.Venus
 */

#pragma once

#include <facilityLayer.hpp>
#include <facilityBase.hpp>
#include <memory>

namespace WaveApp
{

class UdpReceiver
{
  public:
    using ReceiveCall = std::function<void(const void*, std::size_t)>;

    /**
     * Create a UdpReceiver
     *
     * @param port UDP port to listen to
     * @param maxSize maximum receive buffer size
     * @param cb callback function for data reception
     */
    UdpReceiver(int port, std::size_t maxSize, ReceiveCall cb, bool verbose = false);

    /**
     * Initialize UDP connection and start listening for data
     * @return true on success, false otherwise
     */
    bool start();

    /**
     * Stop listening for data and close socket
     */
    void stop();

  private:
    using BufPtr = std::unique_ptr<uint8_t[]>;

    void onCtrlPoll(int fd, Int32u events);

    bool m_verbose{false};
    int m_ctrlPort;
    int m_ctrlSocket{-1};
    sigc::connection m_ctrlPoll;

    std::size_t m_bufSize;
    BufPtr m_rxBuf;
    ReceiveCall m_rxCb;
};

} // namespace WaveApp
