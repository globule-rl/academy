#pragma once

#include <gui/widget.h>
#include <drivers/mouse.h>

namespace os {
    namespace gui {
        class Desktop: public CompositeWidget, public drivers::MouseEvhandler {
            public:
                Desktop(int32_t w, int32_t h, uint8_t r, uint8_t g, uint8_t b);
                ~Desktop();
        };
    }
}