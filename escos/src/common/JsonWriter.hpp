#pragma once

#include <cstddef>
#include <iterator>
#include <string>

namespace WaveApp
{
namespace Json
{
namespace _detail
{

template <class TT>
class BaseWriter
{
  protected:
    TT _target;
    bool _firstElement = true;

  private:
    inline void comma()
    {
        if (!_firstElement)
            _target.push_back(',');
    }

  public:
    BaseWriter() {}
    explicit BaseWriter(std::string& target)
        : _target(target)
    {
    }
    BaseWriter(std::string& target, std::size_t size)
        : _target(target)
    {
        _target.reserve(size);
    }

    void clear()
    {
        _target = "";
        _firstElement = true;
    }

    // Reserves size in memory
    void reserve(std::size_t size) { _target.reserve(size); }

    // Hands over ownership of created JSON string
    std::string&& moveText()
    {
        _firstElement = true;
        return std::move(_target);
    }

    void startObject()
    {
        comma();
        _target.append("{");
        _firstElement = true;
    }

    void startObject(const std::string& name)
    {
        comma();
        _target.push_back('"');
        _target.append(name);
        _target.append("\":{");
        _firstElement = true;
    }

    void endObject()
    {
        _target.append("}");
        _firstElement = false;
    }

    void startArray()
    {
        comma();
        _target.append("[");
        _firstElement = true;
    }

    void startArray(const std::string& name)
    {
        comma();
        _target.push_back('"');
        _target.append(name);
        _target.append("\":[");
        _firstElement = true;
    }

    void endArray()
    {
        _target.append("]");
        _firstElement = false;
    }

    template <class T>
    void put(const std::string& name, const T value)
    {
        comma();
        _target.push_back('"');
        _target.append(name);
        _target.append("\":");
        _target.append(std::to_string(value));
        _firstElement = false;
    }

    template <class T>
    void put(const T value)
    {
        comma();
        _target.append(std::to_string(value));
        _firstElement = false;
    }

    // const char* specialization
    void put(const std::string& name, const char* value)
    {
        comma();
        _target.push_back('"');
        _target.append(name);
        _target.append("\":\"");
        _target.append(value);
        _target.push_back('"');
        _firstElement = false;
    }

    void put(const char* value)
    {
        comma();
        _target.push_back('"');
        _target.append(value);
        _target.push_back('"');
        _firstElement = false;
    }

    // string specialization
    void put(const std::string& name, const std::string& value)
    {
        comma();
        _target.push_back('"');
        _target.append(name);
        _target.append("\":\"");
        _target.append(value);
        _target.push_back('"');
        _firstElement = false;
    }

    void put(const std::string& value)
    {
        comma();
        _target.push_back('"');
        _target.append(value);
        _target.push_back('"');
        _firstElement = false;
    }

    // bool specialization
    void put(const std::string& name, bool value)
    {
        comma();
        _target.push_back('"');
        _target.append(name);
        _target.append((value) ? "\":true" : "\":false");
        _firstElement = false;
    }

    void put(bool value)
    {
        comma();
        _target.append((value) ? "true" : "false");
        _firstElement = false;
    }

    void putRaw(const std::string& name, const std::string value)
    {
        comma();
        _target.push_back('"');
        _target.append(name);
        _target.append("\":");
        _target.append(value);
        _firstElement = false;
    }

    const std::string& get() const { return _target; }
};

} // namespace _detail

/**
 * Basic json writer, allows to construct Json object in string without much overhead.
 * This object contains its string. String can be retrieved with `const string& get()`
 * or can be moved from object with `std::string moveText()`.
 */
using Writer = _detail::BaseWriter<std::string>;

/**
 * Json writer to some external string. In constructor you can provide string to which
 * it will write. Writer then does not own the string so you are responsible for keeping
 * the string alive as long as you use writer.
 */
class WriterTo : public _detail::BaseWriter<std::string&>
{
  public:
    WriterTo() = delete;
    explicit WriterTo(std::string& target)
        : BaseWriter(target)
    {
    }
    WriterTo(std::string& target, std::size_t size)
        : BaseWriter(target, size)
    {
    }
};

static inline std::string escape(const std::string& in)
{
    std::string target;
    target.reserve(in.size());
    for (auto ch = in.cbegin(); ch != in.cend(); ++ch)
    {
        switch (*ch)
        {
            case '"':
                target.append("\\\"");
                break;
            case '\b':
                target.append("\\b");
                break;
            case '\n':
                target.append("\\n");
                break;
            case '\f':
                target.append("\\f");
                break;
            case '\r':
                target.append("\\r");
                break;
            case '\t':
                target.append("\\t");
                break;
            case '\\':
                target.append("\\\\");
                break;
            default:
                // ignoring control chars without short representation
                // https://stackoverflow.com/questions/7724448/#33799784
                target.push_back(*ch);
        }
    }
    return target;
}

} // namespace Json
} // namespace WaveApp
