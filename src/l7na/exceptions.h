#pragma once

#include <exception>

#include <boost/preprocessor/seq/for_each.hpp>
#include <boost/exception/info.hpp>
#include <boost/lexical_cast.hpp>

namespace common {

/*!
 * \brief   Базовый класс для исключений.
 *
 *          Наследуется от std::exception, что позволяет ловить все исключения через catch (std::exception&)
 */

#define EXCEPTION_STREAM_OP_TYPES (bool) (int8_t) (uint8_t) (int16_t) (uint16_t) (int32_t) (uint32_t) (int64_t) (uint64_t) (float) (double)
#define EXCEPTION_STREAM_OP(unused, data, type) \
    data& operator<<(const type rhs) { return append(rhs); }

class Exception
        : virtual public std::exception
        , virtual public boost::exception {
public:
    Exception(const std::string& what = "Exception")
        : what_(what)
    {}

    ~Exception() throw () {}

    const char* what() const throw () {
        return what_.c_str();
    }

    Exception& operator<<(const std::string& rhs) {
        what_.append(rhs);
        return *this;
    }

    template<typename T>
    Exception& operator<<(const T* rhs) {
        return append(rhs);
    }

    BOOST_PP_SEQ_FOR_EACH(EXCEPTION_STREAM_OP, Exception, EXCEPTION_STREAM_OP_TYPES)

protected:
    std::string what_;

private:
    template<typename T>
    Exception& append(const T rhs)
    {
        what_.append(boost::lexical_cast<std::string>(rhs));
        return *this;
    }
};

} // namespaces

//! \brief      Макрос для определения собственных исключений.
#define DECLARE_EXCEPTION(name, base) \
    class name \
        : public base \
    { \
        public: \
            name(const std::string & what = # name) \
                : base(what) \
            {} \
            name& operator<<(const std::string& rhs) { \
                what_.append(rhs); \
                return *this; \
            } \
            template<typename T> \
            name& operator<<(const T* rhs) { \
                return append(rhs); \
            } \
            \
            BOOST_PP_SEQ_FOR_EACH(EXCEPTION_STREAM_OP, name, EXCEPTION_STREAM_OP_TYPES) \
        private: \
            template<typename T> \
            name& append(const T rhs) { \
                what_.append(boost::lexical_cast<std::string>(rhs)); \
                return *this; \
            } \
    }
