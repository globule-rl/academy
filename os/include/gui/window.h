#pragma once

#include <gui/widget.h>
#include <drivers/mouse.h>

namespace os {
    namespace gui {
        class Window: public CompositeWidget {
            public:
                Window(Widget* widget, common::int32_t x, common::int32_t y, common::int32_t w, common::int32_t h, common::uint8_t r, common::uint8_t g, common::uint8_t b);
                ~Window();
        };
    }
}