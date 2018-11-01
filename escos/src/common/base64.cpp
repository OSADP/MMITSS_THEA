#include "base64.hpp"

#include <cstddef>
#include <sstream>
#include <string>

#include "Poco/Base64Encoder.h"
#include "Poco/Base64Decoder.h"
#include "Poco/String.h"

namespace WaveApp
{

std::string Base64::decode(const std::string& b64_str)
{
    std::istringstream ss(b64_str);
    Poco::Base64Decoder bd(ss);
    return std::string(std::istreambuf_iterator<char>(bd), {});
}

std::string Base64::encode(const std::string& str)
{
    std::ostringstream ss;
    Poco::Base64Encoder be(ss);
    be.write(str.data(), static_cast<std::streamsize>(str.size()));
    be.close();
    return Poco::replace(ss.str(), "\r\n", "");
}

std::string Base64::encode(char* data, std::size_t size)
{
    std::ostringstream ss;
    Poco::Base64Encoder be(ss);
    be.write(data, static_cast<std::streamsize>(size));
    be.close();
    return Poco::replace(ss.str(), "\r\n", "");
}

} // namespace WaveApp
