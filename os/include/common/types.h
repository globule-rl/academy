#pragma once

/* unsigned: cant be negative
            unsigned 0 - 1 -> wrap around 2^32-1
        default/signed: pos neg
    char 1 byte
        short 2 bytes
        int 4 bytes
        long 4/8 bytes
        long long 8 bytes
*/
namespace os {
    namespace common {
        typedef unsigned char   uint8_t;
        typedef unsigned short  uint16_t; 
        typedef unsigned int    uint32_t;
        typedef unsigned long long int uint64_t;

        typedef char            int8_t;
        typedef short           int16_t;
        typedef int             int32_t;
        typedef long long int   int64_t;

        typedef const char*     string;
        typedef uint32_t        size_t_32;
    }
}

