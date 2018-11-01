#include "asnHelper.hpp"

#include "Poco/NumberFormatter.h"

#ifdef C2X_UNIT_TESTS
#include "catch.hpp"
#endif

namespace WaveApp
{
namespace AsnHelper
{

void initBitString(asnBitString& bs, std::size_t len)
{
    if (bs.value && bs.valueMustBeReleased)
        asn_free(static_cast<void*>(bs.value));

    std::size_t numBytes = sizeof(asnbyte) * ((len + 7) / 8);
    bs.value = static_cast<asnbyte*>(asn_malloc(numBytes));
    memset(bs.value, 0, numBytes);
    bs.length = len;
    bs.valueMustBeReleased = asnTRUE;
}

std::string octetStringToString(asnOctetString& s)
{
    unsigned int len;
    unsigned char* str;
    s.getOctetString(&len, &str);
    return std::string(reinterpret_cast<char*>(str), len);
}

std::string latLonToString(int val)
{
    double fval = static_cast<double>(val) / 10000000;
    return Poco::NumberFormatter::format(fval, 7);
}

#ifdef C2X_UNIT_TESTS
TEST_CASE("AsnHelper::latLogToString", "[latlontostring]")
{
    REQUIRE(WaveApp::AsnHelper::latLonToString(0) == "0.0000000");
    REQUIRE(WaveApp::AsnHelper::latLonToString(1) == "0.0000001");
    REQUIRE(WaveApp::AsnHelper::latLonToString(1234567) == "0.1234567");
    REQUIRE(WaveApp::AsnHelper::latLonToString(1234567890) == "123.4567890");
    REQUIRE(WaveApp::AsnHelper::latLonToString(10000000) == "1.0000000");
    REQUIRE(WaveApp::AsnHelper::latLonToString(-1800000000) == "-180.0000000");
}
#endif

} // namespace AsnHelper
} // namespace WaveApp
