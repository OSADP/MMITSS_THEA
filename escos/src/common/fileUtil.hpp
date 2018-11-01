#pragma once

#include <sys/stat.h>
#include <cstddef>
#include <string>

#define MAX_PATHNAME 512
#define MAX_FILE_SIZE (128 * 1024)

namespace WaveApp
{

class FileUtil
{
  public:
    static constexpr std::size_t DEFAULT_MAX_FILE_SIZE{5 * 1024 * 1024};
    static constexpr mode_t DEFAULT_MODE{S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH};

    static bool isFile(const char* path, const char* prefix = nullptr);

    static bool readFile(
        std::string& str, const char* path, std::size_t maxFileSize = DEFAULT_MAX_FILE_SIZE);

    static bool writeFile(
        const char* str, const char* path, std::size_t len = 0, bool append = false);

    static std::size_t size(const char* path);

    // TODO(C++17) std::filesystem
    static int makeDir(const char* path, mode_t mode);
    static int makePath(const char* path, mode_t mode);
};

} // namespace WaveApp
