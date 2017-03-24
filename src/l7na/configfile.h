#pragma once

#include <string>
#include <unordered_set>
#include <unordered_map>
#include <type_traits>

#include <boost/lexical_cast.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>

#include "exceptions.h"

namespace Config {

DECLARE_EXCEPTION(Exception, common::Exception);

/*!
 * \brief       Simple config storage
 */
class Storage {
public:
    // 1st - axis, 2nd - index, 3rd - sub index
    typedef boost::tuple<uint16_t, uint16_t, uint8_t> Key;
    // 1st - value, 2nd - value byte size
    typedef boost::tuple<int64_t, uint8_t> Value;

    struct KeyHash
        : public std::unary_function<KeyHash, std::size_t>
    {
        std::size_t operator()(const Key& k) const {
            std::size_t seed = 0;

            boost::hash_combine(seed, boost::get<0>(k));
            boost::hash_combine(seed, boost::get<1>(k));
            boost::hash_combine(seed, boost::get<2>(k));

            return seed;
        }
    };

    struct KeyComp {
        bool operator()(const Key& k1, const Key& k2) const {
            return (boost::get<0>(k1) == boost::get<0>(k2))
                    && (boost::get<1>(k1) == boost::get<1>(k2))
                    && (boost::get<2>(k1) == boost::get<2>(k2));
        }
    };

    typedef boost::unordered_map<Key, Value, KeyHash, KeyComp> KeyValueDict;

    Storage();

    /*!
     * \brief           Read config file of the specified format
     *
     * File format is as follows:
     *
     * Key1=Value1
     * Key2=Value2
     * # Comment
     * Key1=Value3
     * Key3=Multiword value
     * Multiword key=Value4
     * ...
     * KeyN=ValueM
     *
     * \attention       Keys may recur. In this case all values of the same key are stored.
     *                  Keys and values may be multiword.
     *                  All the leading and trailing spaces are trimmed.
     *
     * \param filepath  Path to the file to read.
     *
     * \throw ConfigException
     */
    void ReadFile(const std::string& filepath);

    bool IsEmpty() const;

    /*!
     * \brief       Check if the key is in the config
     * \param key   Key to find
     * \return      Flag showing the presence of the key in the config
     */
    bool HasKey(const Key& key) const;

    /*!
     * \brief       Get first value assosiated with the key
     * \param key   Key to find
     * \return      Value
     */
    const Value& GetValue(const Key& key) const;

    /*!
     * \brief       Get all key-value pairs
     * \return      Key-value dictionary
     */
    const KeyValueDict& GetWholeDict() const;

private:
    KeyValueDict m_kvdict;              //!< Key-value storage
};

bool ValueToBool(const Storage::Value& val);

} // namespaces
