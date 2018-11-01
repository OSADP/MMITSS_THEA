#pragma once

#include <cstddef>
#include <string>

namespace WaveApp
{

class Base64
{
  public:
    static std::string decode(const std::string& b64_str);
    static std::string encode(const std::string& str);
    static std::string encode(char* data, std::size_t size);
};

} // namespace WaveApp
