#pragma once

#include "log.h"

#include <etl/string_view.h>
#include <etl/delegate.h>
#include <etl/array.h>

#include <cstdint>

namespace line_processor {

    using callback_t =  etl::delegate<void(etl::string_view)>;

    /**
     * Splits input stream into lines and feeds lines into callback function.
     */
    template<size_t BUF_SIZE = 32>
    class LineProcessor {
    public:

        LineProcessor(callback_t cb): cb{cb} {
        }

        void add(char rd) {
            if(rd=='\n') {
                if(pos>0 && buf[pos-1]=='\r') pos--;
                cb(etl::string_view{buf.data(), pos});
                pos = 0;
            } else {
                if(pos<BUF_SIZE) {
                    buf[pos++] = rd;
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
