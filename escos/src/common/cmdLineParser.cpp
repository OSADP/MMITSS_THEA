#include "cmdLineParser.hpp"

#include <cstddef>
#include <cstring>

namespace WaveApp
{

CmdLineParser::CmdLineParser(const char* line)
{
    std::size_t line_len = strlen(line);
    m_buffer = new char[line_len + 1];
    memcpy(m_buffer, line, line_len + 1);
}

CmdLineParser::~CmdLineParser()
{
    delete[] m_buffer;
}

std::size_t CmdLineParser::parse(std::size_t argc, char* argv[])
{
    std::size_t no = 0, i = 0;
    std::size_t len = strlen(m_buffer);

    for (std::size_t j = 0; j < argc; ++j)
        argv[j] = 0;

    while ((i < len) && (no < argc))
    {
        if (m_buffer[i] != ' ')
        {
            // we've hit a char - set argv if still zero
            if (argv[no] == 0)
                argv[no] = m_buffer + i;
        }
        else if (argv[no])
        {
            // hit a blank - force string end here
            m_buffer[i] = 0;
            no++;
        }
        i++;
    }
    return (no == argc || argv[no] == 0) ? no : no + 1;
}

} // namespace WaveApp
