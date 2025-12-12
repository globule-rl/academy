#pragma once

#include <common/types.h>
#include <common/graphicsContext.h>
#include <drivers/keyboard.h>

namespace os {
    namespace gui {
        class Widget : public drivers::KeyboardEvHandler {
            protected:
                Widget* container;
                int32_t x;
                int32_t y;
                int32_t w;
                int32_t h;

                int32_t r;
                int32_t g;
                int32_t b;
                bool Focusable;
            public:
                Widget(Widget* container, int32_t x, int32_t y, int32_t w, int32_t h,
                    uint8_t r, uint8_t g, uint8_t b);
                ~Widget();
                virtual void GetFocus(Widget* widget);
                virtual void ModelToScreen(int32_t &x, int32_t &y);
                virtual void ContainsCoord(int32_t x, int32_t y);
                virtual void Draw(GraphicsContext* gc);
                virtual void OnMouseDown(int32_t x, int32_t y, uint8_t btn);
                virtual void OnMouseUp(int32_t x, int32_t y, uint8_t btn);
                virtual void OnMouseMove(int32_t oldX, int32_t oldY, int32_t newX, int32_t newY);
        };
        class CompositeWidget : public Widget {
            private:
                Widget* scenes[100];
                int numScene;
                Widget* focused;
            public:
               CompositeWidget(Widget* container, int32_t x, int32_t y, int32_t w, int32_t h,
                    uint8_t r, uint8_t g, uint8_t b);
                ~CompositeWidget();
                virtual void Attach(Widget* scene);
                virtual void GetFocus(Widget* widget);
                virtual void Draw(GraphicsContext* gc);
                virtual void OnMouseDown(int32_t x, int32_t y, uint8_t btn);
                virtual void OnMouseUp(int32_t x, int32_t y, uint8_t btn);
                /* mouse leave, enter: local->relative pos oldx-x/outside newx-x/inside
                    leave: execute every time, =/then assign as old/same pos, old in/new not
                    enter: only when -1 != 0/new area/not same->avoid repetition ev call, new in/old not
                */
                virtual void OnMouseMove(int32_t oldX, int32_t oldY, int32_t newX, int32_t newY); 
                virtual void OnKeyDown(char);
                virtual void OnKeyUp(char);
        };
    }
}