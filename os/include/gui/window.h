#pragma once

#include <gui/widget.h>
#include <drivers/mouse.h>

namespace os {
    namespace gui {
        class Window: public CompositeWidget {
            public:
                Window(Widget* widget, int32_t x, int32_t y, int32_t w, int32_t h, uint8_t r, uint8_t g, uint8_t b);
                ~Window();
        };
    }
}