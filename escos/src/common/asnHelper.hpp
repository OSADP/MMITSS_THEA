#pragma once

#include <cstddef>
#include <iterator>
#include <string>

#include "sae.hpp"

namespace WaveApp
{
namespace AsnHelper
{

void initBitString(asnBitString& bs, std::size_t len);
std::string octetStringToString(asnOctetString& s);
std::string latLonToString(int val);

/**
 * Iterator wrapper over SEQUENCE from ASN. For usage description see iter_asn.
 */
template <class T, class SEQUENCE>
class AsnSequence
{
  private:
    SEQUENCE& sequence;

  public:
    class iterator
    {
      private:
        T* pos{nullptr};

      public:
        typedef std::forward_iterator_tag iterator_category;
        typedef T value_type;
        typedef T difference_type;
        typedef T* pointer;
        typedef T& reference;

        explicit iterator(T* _pos)
            : pos(_pos)
        {
        }

        iterator& operator++()
        {
            pos = dynamic_cast<T*>(pos->getNextElement());
            return *this;
        }
        bool operator!=(const iterator& rhs) { return pos != rhs.pos; }
        T& operator*() { return *pos; }
    };

    explicit AsnSequence(SEQUENCE& _sequence)
        : sequence(_sequence)
    {
    }

    iterator begin() { return iterator{dynamic_cast<T*>(sequence.getFirstElement())}; }
    iterator end() { return iterator{nullptr}; }
};

/**
 * Helper function to get iterator over ASN sequence. This will allow us
 * to use range for and standard algorithms.
 *
 * Only the first template parameter is needed - it is the type of the item in
 * the sequence you want to dynamic_cast to. The second one is deduced from the
 * function argument.
 *
 * The usage is:
 * ~~~
 * saeRoadSideAlert* rsa = ... ;
 * for (auto& itis : iter_asn<asncsaeRoadSideAlert__descriptions>(rsa->description))
 *     cout << "ITIS: "<< itis.value << endl;
 * ~~~
 *
 * Note: This helper function is needed because template argument deduction from
 * constructors needs C++17.
 */
template <class T, class SEQUENCE>
AsnSequence<T, SEQUENCE&> iter_asn(SEQUENCE& sequence)
{
    return AsnSequence<T, SEQUENCE&>(sequence);
}

} // namespace AsnHelper
} // namespace WaveApp
