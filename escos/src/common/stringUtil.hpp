#pragma once

#include <algorithm>
#include <string>
#include <vector>

namespace WaveApp
{

/**
 * @return true if text starts with substring
 */
inline bool starts_with(const std::string& text, const std::string& substring)
{
    if (substring.size() > text.size())
        return false;
    return std::equal(substring.begin(), substring.end(), text.begin());
}

/**
 * @return true if text ends with substring
 */
inline bool ends_with(const std::string& text, const std::string& substring)
{
    if (substring.size() > text.size())
        return false;
    return std::equal(substring.rbegin(), substring.rend(), text.rbegin());
}

namespace _string_private
{
constexpr static const char* delims = " \t\r\n";
} // namespace _string_private

/**
 * modify string by trimming whitespace chars
 * @return trimmed string reference
 */
inline std::string& trim(std::string& text, bool left = true, bool right = true)
{
    if (right)
        text.erase(text.find_last_not_of(_string_private::delims) + 1);
    if (left)
        text.erase(0, text.find_first_not_of(_string_private::delims));
    return text;
}

/**
 * copy string and trim whitespace chars
 * @return trimmed string copy
 */
inline std::string trimmed(std::string text, bool left = true, bool right = true)
{
    if (right)
        text.erase(text.find_last_not_of(_string_private::delims) + 1);
    if (left)
        text.erase(0, text.find_first_not_of(_string_private::delims));
    return text;
}

/**
 * Splits the string on delimiters. Consecutive delimiters are not merged.
 *
 * @param text       input string
 * @param delimiters chars to split on
 * @param maxSplits  split into at most maxSplits+1 result strings if >0
 * @return vector of strings
 */
inline std::vector<std::string> split(
    const std::string& text, const std::string& delimiters = "\t\n ", int maxSplits = 0)
{
    std::vector<std::string> result;
    std::size_t text_size = text.size();
    if (text_size == 0)
        return result;

    // 10 is guessed capacity for most cases
    result.reserve(maxSplits ? maxSplits + 1 : 10);
    int numSplits = 0;

    std::size_t start{0}, pos{0};
    do
    {
        pos = text.find_first_of(delimiters, start);
        if (pos == std::string::npos || (maxSplits && numSplits == maxSplits))
        {
            // Copy the rest of the string
            result.emplace_back(text.substr(start));
            break;
        }

        // Copy up to delimiter
        result.emplace_back(text.substr(start, pos - start));
        ++numSplits;
        start = pos + 1;
    } while (start < text_size + 1);

    return result;
}

/**
 * Splits the string on delimiters. Consecutive delimiters are merged.
 *
 * @param text       input string
 * @param delimiters chars to split on
 * @param maxSplits  split into at most maxSplits+1 result strings if >0
 * @return vector of strings
 */
inline std::vector<std::string> split_merge(
    const std::string& text, const std::string& delimiters = "\t\n ", int maxSplits = 0)
{
    std::vector<std::string> result;
    std::size_t start{text.find_first_not_of(delimiters, 0)};
    if (start == std::string::npos)
        return result;

    // 10 is guessed capacity for most cases
    result.reserve(maxSplits ? maxSplits + 1 : 10);
    int numSplits = 0;

    std::size_t pos{0};
    do
    {
        pos = text.find_first_of(delimiters, start);
        if (pos == std::string::npos || (maxSplits && numSplits == maxSplits))
        {
            // Copy the rest of the string
            result.emplace_back(text.substr(start));
            return result;
        }

        // Copy up to delimiter
        result.emplace_back(text.substr(start, pos - start));
        ++numSplits;
        start = text.find_first_not_of(delimiters, pos + 1);
    } while (start != std::string::npos);

    return result;
}

inline int char2int(char input)
{
    if (input >= '0' && input <= '9')
        return input - '0';
    if (input >= 'A' && input <= 'F')
        return input - 'A' + 10;
    if (input >= 'a' && input <= 'f')
        return input - 'a' + 10;
    throw std::invalid_argument("Invalid input string");
}

// This function assumes src to be a zero terminated sanitized string with
// an even number of [0-9a-f] characters, and target to be sufficiently large
// @throws invalid_argument
inline int hex2bin(const std::string& src, uint8_t* target, size_t targetLen)
{
    int cnt = 0;
    if (src.length() % 2)
        throw std::invalid_argument("Invalid input string");
    if ((src.length() / 2) > targetLen)
        throw std::invalid_argument("Input string to big");
    const char* c = src.c_str();
    while (c[0] && c[1])
    {
        *(target++) = (char2int(c[0]) << 4) + char2int(c[1]);
        c += 2;
        cnt++;
    }
    return cnt;
}

} // namespace WaveApp
