#pragma once

#include "log.h"

#include <etl/string_view.h>
#include <etl/delegate.h>
#include <etl/array.h>

#include <cstdint>

namespace line_processor {

    using callback_t =  etl::delegate<void(const char*, size_t)>;

    /**
     * Splits input stream into lines and feeds lines into callback function.
     */
    template<size_t BUF_SIZE = 32>
    class LineProcessor {
    public:

        LineProcessor(callback_t cb): cb{cb} {

        }

        void add(char c) {
            if(c == '\n') {
                buf[pos] = 0;
                cb(buf.data(), pos);
                pos = 0;
            } else {
                if(pos<BUF_SIZE) {
                    buf[pos++] = c;
                } else {
                    pos = 0;
                    logs("Buffer overflow!\n");
                }
            }
        }
    private:
        etl::array<char, BUF_SIZE> buf;
        size_t pos = 0;
        callback_t cb;
    };

};
