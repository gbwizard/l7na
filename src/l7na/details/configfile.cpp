#include <fstream>

#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/case_conv.hpp>

#include "l7na/configfile.h"

namespace Config {

Storage::Storage() = default;

void Storage::ReadFile(const std::string& filepath) {
    // Empty the key-value storage
    m_kvdict.clear();

    std::ifstream input(filepath);
    if (! input.good()) {
        BOOST_THROW_EXCEPTION(Exception("Failed to open file: ") << filepath);
    }

    const char kKeyValueDelim = '=';
    const char kKeyDelim = ':';
    const char kCommentStart = '#';
    std::string cur_line;
    int32_t linenum = 1;
    while (std::getline(input, cur_line)) {
        boost::algorithm::trim(cur_line);
        if (cur_line.empty() || cur_line[0] == kCommentStart) {
            continue;
        }
        const size_t kv_delim_pos = cur_line.find(kKeyValueDelim);
        if (kv_delim_pos == std::string::npos) {
            BOOST_THROW_EXCEPTION(Exception("No key-value delimiter found in line ") << linenum);
            continue;
        }

        // Process key
        std::string key_str = cur_line.substr(0, kv_delim_pos);
        boost::algorithm::trim(key_str);
        // Find 1st key part
        const size_t key1_delim_pos = cur_line.find(kKeyDelim);
        if (key1_delim_pos == std::string::npos) {
            BOOST_THROW_EXCEPTION(Exception("No 1st key delimiter found in line ") << linenum);
            continue;
        }
        std::string key1_str = key_str.substr(0, key1_delim_pos);
        boost::algorithm::trim(key1_str);
        // Find 2nd key part
        const size_t key2_delim_pos = cur_line.find(kKeyDelim, key1_delim_pos + 1);
        if (key2_delim_pos == std::string::npos) {
            BOOST_THROW_EXCEPTION(Exception("No 2nd key delimiter found in line ") << linenum);
            continue;
        }
        std::string key2_str = key_str.substr(key1_delim_pos + 1, key2_delim_pos - key1_delim_pos);
        boost::algorithm::trim(key2_str);
        // Find 3rd key part
        std::string key3_str = key_str.substr(key2_delim_pos + 1);
        boost::algorithm::trim(key3_str);

        /* Process value
         * @attention It's safe to pass values to substr with length [0, size].
         * Here value is taken from find method which returns [0, size - 1] or npos.
         * So value + 1 is [0, size] at maximum.
         */
        std::string val_str = cur_line.substr(kv_delim_pos + 1);
        boost::algorithm::trim(val_str);

        // Result key-value
        Key key;
        Value val;
        try {
            const uint8_t key1 = boost::lexical_cast<uint8_t>(key1_str);
            const uint16_t key2 = boost::lexical_cast<uint8_t>("0x" + key2_str);
            const uint8_t key3 = boost::lexical_cast<uint8_t>(key3_str);
            key = boost::make_tuple(key1, key2, key3);
            val = boost::lexical_cast<Value>(val_str);
        } catch (const boost::bad_lexical_cast&) {
            BOOST_THROW_EXCEPTION(Exception("Invalid key detected in line number ") << linenum);
        }

        // The last read value with the same key is stored
        m_kvdict[key] = val;

        ++linenum;
    }
    input.close();
}

bool Storage::IsEmpty() const {
    return m_kvdict.empty();
}

bool Storage::HasKey(const Key& key) const {
    return m_kvdict.find(key) != m_kvdict.end();
}

const Storage::Value& Storage::GetValue(const Key& key) const {
    const auto& found_it = m_kvdict.find(key);
    if (found_it != m_kvdict.end()) {
        return found_it->second;
    }

    BOOST_THROW_EXCEPTION(Exception("GetValue(): key was not found"));
}

const Storage::KeyValueDict& Storage::GetWholeDict() const {
    return m_kvdict;
}

} // namespace
