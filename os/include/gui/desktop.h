#pragma once

#include <gui/widget.h>
#include <drivers/mouse.h>

namespace os {
    namespace gui {
        class Desktop: public CompositeWidget, public drivers::MouseEvhandler {
            public:
                Desktop(common::int32_t w, common::int32_t h, common::uint8_t r, common::uint8_t g, common::uint8_t b);
                ~Desktop();
        };
    }
}