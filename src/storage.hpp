#ifndef STORAGE_H_
#define STORAGE_H_

#include <etl/string_view.h>
#include <etl/optional.h>

namespace storage {

    void init();

    constexpr size_t DEVNAME_LEN = 64;
    constexpr char DEFAULT_DEVNAME[] = "\xF0\x9F\x9A\x9A MicroRC";// "NanoRC";
    etl::optional<etl::string_view> get_dev_name();
    bool set_dev_name(const etl::string_view name);

};


#endif // STORAGE_H_