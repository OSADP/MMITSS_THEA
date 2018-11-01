/*
 * spatMapProxy.cpp
 *
 *  Created on: May 25, 2017
 *      Author: M.Venus
 */

#include "stdheader.h"
#include "stdtypes.h"

#include "spatMapProxy.hpp"
#include "Poco/JSON/JSON.h"
#include "Poco/JSON/Parser.h"
#include "Poco/DynamicStruct.h"
#include "base64.hpp"
#include "facilityBase.hpp"
#include "facilityLayer.hpp"

namespace WaveApp
{
namespace MmitssCtrl
{

void SpatMapProxy::readMap()
{
    Poco::Dynamic::Var result;
    siekdbus::Message reply;
    if (siekdbus::Method::call(reply, "com.siemens.c2x.spatmap", "/com/siemens/c2x/spatmap",
            "com.siemens.c2x.spatmap.Service", "Command", "ss", "json",
            "{\"command\":\"edit_map\"}"))
    {
        const char* replyText = nullptr;
        if (reply.read("s", &replyText) > 0)
        {
            try
            {
                Poco::JSON::Parser parser;
                Poco::JSON::Object::Ptr json =
                    parser.parse(replyText).extract<Poco::JSON::Object::Ptr>();
                if (json->has("error"))
                {
                    ITSAPP_WRN("JSON error getting MAP from spatmap: %s",
                        json->get("error").toString().c_str());
                }
                else if (json->has("edit_map"))
                {
                    std::string mapXer = Base64::decode(json->get("edit_map").toString());
                    if (!mapXer.empty())
                    {
                        asnMemoryStream is((asnbyte*)const_cast<char*>(mapXer.c_str()),
                            mapXer.length(), asnFSTREAM_READ);
                        m_map = std::make_shared<saeMapData>();
                        m_map->XERdecode(m_parent.getAsnContext(), &is);
                        return;
                    }
                }
            }
            catch (const asnException& ex)
            {
                ITSAPP_WRN("MAP decode failed (ASN exception) %s", m_parent.toString(ex));
            }
            catch (const std::exception& ex)
            {
                ITSAPP_WRN("MAP decode failed %s", ex.what());
            }
        }
        else
        {
            ITSAPP_WRN("Internal error getting MAP from spatmap: %s", reply.getError());
        }
    }
    m_map = nullptr;
}

void SpatMapProxy::clearMap()
{
    m_map = nullptr;
}

} // namespace MmitssCtrl
} // namespace WaveApp
