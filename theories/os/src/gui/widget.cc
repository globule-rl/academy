#include <gui/widget.h>

using namespace os::common;
using namespace os::gui;

CompositeWidget::CompositeWidget(Widget* container, int32_t x, int32_t y, int32_t w, int32_t h, uint8_t r, uint8_t g, uint8_t b)
: Widget(container, x,y,w,h, r,g,b) {

}
CompositeWidget::~CompositeWidget() {}

void CompositeWidget::Attach(Widget* widget) {

}