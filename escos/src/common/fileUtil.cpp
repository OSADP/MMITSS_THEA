#include "fileUtil.hpp"

#include <sys/file.h>
#include <sys/stat.h>
#include <cstddef>
#include <cstring>
#include <fstream>
#include <string>

#define MAX_LINE_BUF 80
#define MAX_WRITE_BUF 1024

namespace WaveApp
{

static bool getStat(const char* path, const char* prefix, struct stat& fstat)
{
    char buffer[MAX_PATHNAME + 1];
    const char* _path = path;
    if (prefix)
    {
        snprintf(buffer, MAX_PATHNAME, "%s/%s", prefix, path);
        _path = buffer;
    }
    return (::stat(_path, &fstat) == 0);
}

bool FileUtil::isFile(const char* path, const char* prefix)
{
    struct stat fstat;
    return getStat(path, prefix, fstat) && !S_ISDIR(fstat.st_mode);
}

std::size_t FileUtil::size(const char* path)
{
    std::size_t result = 0;
    struct stat stat_buf;
    if (::stat(path, &stat_buf) == 0)
    {
        result = stat_buf.st_size;
    }
    return result;
}

bool FileUtil::readFile(std::string& data, const char* path, std::size_t maxFileSize)
{
    struct stat stat_buf;
    if (::stat(path, &stat_buf) != 0 || S_ISDIR(stat_buf.st_mode))
        return false;

    std::size_t fSize = stat_buf.st_size;
    if (fSize > maxFileSize)
    {
        errno = EFBIG;
        return false;
    }

    data.clear();
    data.reserve(fSize);

    std::ifstream ifs(path, std::ios::in | std::ios::binary);
    data.assign(std::istreambuf_iterator<char>{ifs}, {});

    return true;
}

bool FileUtil::writeFile(const char* str, const char* path, std::size_t len, bool append)
{
    if (!str)
        return false;

    int nWritten = 0;
    int bufLen = (len == 0) ? strlen(str) : len;
    FILE* myHandle = ::fopen(path, (append ? "a+" : "w+"));
    if (myHandle)
    {
        nWritten = ::fwrite(str, 1, bufLen, myHandle);
        ::fflush(myHandle);
        ::fclose(myHandle);
    }
    return (nWritten == bufLen);
}

int FileUtil::makeDir(const char* path, mode_t mode)
{
    int res{0};
    struct stat stat_buf;
    if (::stat(path, &stat_buf) != 0)
    {
        // EEXIST for race condition
        if ((res = mkdir(path, mode)) != 0 && errno == EEXIST)
            res = 0;
    }
    else if (!S_ISDIR(stat_buf.st_mode))
        return ENOTDIR;
    return res;
}

int FileUtil::makePath(const char* path, mode_t mode)
{
    int res{0};
    char path_copy[strlen(path) + 1];
    char* done_ptr = path_copy;
    char* sep_ptr;
    strcpy(path_copy, path);
    while (res == 0 && (sep_ptr = strchr(done_ptr, '/')) != NULL)
    {
        if (sep_ptr != done_ptr)
        {
            *sep_ptr = '\0';
            res = makeDir(path_copy, mode);
            *sep_ptr = '/';
        }
        done_ptr = sep_ptr + 1;
    }
    if (res == 0)
        res = makeDir(path, mode);
    return res;
}

} // namespace WaveApp
