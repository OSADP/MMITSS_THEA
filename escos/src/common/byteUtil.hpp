#pragma once

#include <cstdio>
#include <openssl/md5.h>
#include <string>

namespace WaveApp
{

inline std::string hexify(const uint8_t* src, const size_t len)
{
    char hex[len * 2 + 1];
    for (size_t i = 0; i < len; ++i)
    {
        sprintf(hex + i * 2, "%02x", src[i]);
    }
    hex[len * 2] = 0;
    return std::string(hex);
}

inline std::string hexify(const std::string& src)
{
    return hexify(reinterpret_cast<const uint8_t*>(src.c_str()), src.length() + 1);
}

inline std::string md5sum(const uint8_t* src, const size_t len)
{
    uint8_t md5_buf[MD5_DIGEST_LENGTH];
    MD5(src, len, md5_buf);
    return hexify(md5_buf, MD5_DIGEST_LENGTH);
}

} // namespace WaveApp
