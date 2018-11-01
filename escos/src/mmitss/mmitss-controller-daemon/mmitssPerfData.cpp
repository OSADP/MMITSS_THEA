/*
 * mmitssPerfData.cpp
 *
 *  Created on: May 25, 2017
 *      Author: M.Venus
 */

#include "stdheader.h"
#include "mmitssPerfData.hpp"
#include "facilityLayer.hpp"
#include "Poco/JSON/JSON.h"
#include "Poco/JSON/Parser.h"
#include "facilityDatabaseAdapter.hpp"

namespace WaveApp
{
namespace MmitssCtrl
{

static const size_t MMITSS_MAX_PERF_DATA_JSON = (64 * 1024);

MmitssPerfData::MmitssPerfData(const MmitssConfig& cfg, its::ItsFacilityApplication& parent)
    : m_dataReceiver(::SieMmitss::getMmitssCtlPerfDataPort(),
          MMITSS_MAX_PERF_DATA_JSON,
          [this](const void* data, size_t len) { return this->onPerfDataReceive(data, len); })
    , m_config(cfg)
    , m_parent(parent)
{
}

bool MmitssPerfData::start()
{
    return m_dataReceiver.start();
}

void MmitssPerfData::stop()
{
    m_dataReceiver.stop();
}

void MmitssPerfData::onPerfDataReceive(const void* data, size_t len)
{
    NOTUSED(len);
    std::string msg = std::string(static_cast<const char*>(data));
    assert(len > 0);
    try
    {
        Poco::JSON::Parser parser;
        auto json = parser.parse(msg).extract<Poco::JSON::Object::Ptr>();
        if (json->has("status"))
        {
            if (json->getObject("status")->has("numVeh"))
            {
                ITSAPP_VERIFY(m_parent.database().write("/its/us/mmitss/performance/num_veh",
                    json->getObject("status")->getValue<int>("numVeh")));
            }
        }
        if (json->has("queue"))
        {
            auto arr = json->getArray("queue");
            its::FacilityDatabaseAdapter::IndexList queueLaneIdList;
            ITSAPP_VERIFY(m_parent.database().getListIndices(
                "/its/us/mmitss/performance/queue_list", queueLaneIdList));

            for (auto obj : *arr)
            { // store in db
                auto qObj = std::make_shared<its::DatabaseListItem>();
                auto j = obj.extract<Poco::JSON::Object::Ptr>();
                auto laneId = m_config.getMmitssMap().getLaneId(
                    j->getValue<int>("appr"), j->getValue<int>("lane"));
                qObj->content["queue_len"] = j->getValue<std::string>("queueLen");
                qObj->content["queue_count"] = j->getValue<std::string>("queueCnt");
                qObj->content["vehicle_count"] = j->getValue<std::string>("vehCnt");
                qObj->content["approach"] = j->getValue<std::string>("appr");
                qObj->content["lane"] = j->getValue<std::string>("lane");
                if (laneId > 0)
                {
                    ITSAPP_VERIFY(m_parent.database().setListItem(
                        "/its/us/mmitss/performance/queue_list", qObj, laneId));
                    // remove id from current id list
                    auto it = std::find(queueLaneIdList.begin(), queueLaneIdList.end(), laneId);
                    if (it != queueLaneIdList.end())
                        queueLaneIdList.erase(it);
                }
            }
            // remove remaining old items
            for (const auto& id : queueLaneIdList)
                ITSAPP_VERIFY(
                    m_parent.database().delListItem("/its/us/mmitss/performance/queue_list", id));
        }
        else if (json->has("ttAcc"))
        {
            its::DatabaseListData perfList;
            auto arr = json->getArray("ttAcc");
            for (auto obj : *arr)
            {
                auto perfObj = std::make_shared<its::DatabaseListItem>();
                auto j = obj.extract<Poco::JSON::Object::Ptr>();
                perfObj->content["movement"] = j->getValue<std::string>("move");
                perfObj->content["travel_time"] = j->getValue<std::string>("tt");
                perfObj->content["delay"] = j->getValue<std::string>("delay");
                perfObj->content["num_stops"] = j->getValue<std::string>("numstops");
                perfObj->content["throughput"] = j->getValue<std::string>("throughput");
                perfList.emplace_back(perfObj);
            }
            if (perfList.size())
            {
                ITSAPP_VERIFY(m_parent.database().replaceList(
                    "/its/us/mmitss/performance/traf_perf_list", perfList));
            }
            else
            {
                ITSAPP_VERIFY(
                    m_parent.database().clearList("/its/us/mmitss/performance/traf_perf_list"));
            }
        }
        else
        {
            ITSAPP_WRN("Unknown JSON data: %s", msg.c_str());
        }
    }
    catch (const Poco::Exception& e)
    {
        ITSAPP_WRN("Error processing JSON: %s", e.displayText().c_str());
    }
    catch (const std::exception& e)
    {
        ITSAPP_WRN("Error %s", e.what());
    }
}

} // namespace MmitssCtrl
} // namespace WaveApp
