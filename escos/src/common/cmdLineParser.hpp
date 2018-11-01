#pragma once

#include <cstddef>

namespace WaveApp
{

class CmdLineParser
{
  public:
    explicit CmdLineParser(const char* line);
    virtual ~CmdLineParser();
    CmdLineParser(const CmdLineParser&) = delete;
    CmdLineParser& operator=(const CmdLineParser&) = delete;

    std::size_t parse(std::size_t argc, char* argv[]);

  private:
    char* m_buffer;
};

} // namespace WaveApp
