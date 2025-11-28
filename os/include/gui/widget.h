#pragma once

#include <common/types.h>
#include <common/graphicsContext.h>
#include <drivers/keyboard.h>

namespace os {
    namespace gui {
        class Widget : public drivers::KeyboardEvHandler {
            protected:
                Widget* container;
                common::int32_t x;
                common::int32_t y;
                common::int32_t w;
                common::int32_t h;

                common::int32_t r;
                common::int32_t g;
                common::int32_t b;
                bool Focusable;
            public:
                Widget(Widget* container, common::int32_t x, common::int32_t y, common::int32_t w, common::int32_t h,
                    common::uint8_t r, common::uint8_t g, common::uint8_t b);
                ~Widget();
                virtual void GetFocus(Widget* widget);
                virtual void ModelToScreen(common::int32_t &x, common::int32_t &y);
                virtual void ContainsCoord(common::int32_t x, common::int32_t y);
                virtual void Draw(common::GraphicsContext* gc);
                virtual void OnMouseDown(common::int32_t x, common::int32_t y, common::uint8_t btn);
                virtual void OnMouseUp(common::int32_t x, common::int32_t y, common::uint8_t btn);
                virtual void OnMouseMove(common::int32_t oldX, common::int32_t oldY, common::int32_t newX, common::int32_t newY);
        };
        class CompositeWidget : public Widget {
            private:
                Widget* scenes[100];
                int numScene;
                Widget* focused;
            public:
               CompositeWidget(Widget* container, common::int32_t x, common::int32_t y, common::int32_t w, common::int32_t h,
                    common::uint8_t r, common::uint8_t g, common::uint8_t b);
                ~CompositeWidget();
                virtual void Attach(Widget* scene);
                virtual void GetFocus(Widget* widget);
                virtual void Draw(common::GraphicsContext* gc);
                virtual void OnMouseDown(common::int32_t x, common::int32_t y, common::uint8_t btn);
                virtual void OnMouseUp(common::int32_t x, common::int32_t y, common::uint8_t btn);
                /* mouse leave, enter: local->relative pos oldx-x/outside newx-x/inside
                    leave: execute every time, =/then assign as old/same pos, old in/new not
                    enter: only when -1 != 0/new area/not same->avoid repetition ev call, new in/old not
                */
                virtual void OnMouseMove(common::int32_t oldX, common::int32_t oldY, common::int32_t newX, common::int32_t newY); 
                virtual void OnKeyDown(char);
                virtual void OnKeyUp(char);
        };
    }
}