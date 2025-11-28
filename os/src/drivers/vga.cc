#pragma once

#include<drivers/vga.h>

using namespace os::common;
hwCom
/* initialize mem vars
*/
Vga::Vga() 
: miscPort(0x3C2), crtcIndexPort(0x3D4), crtcDataPort(0x3D5), seqIndexPort(0x3C4), seqDataPort(0x3C5),
    gpuIndexPort(0x3CE), gpuDataPort(0x3CF), attrcIndexPort(0x3C0), attrcReadPort(0x3C1), attrcWritePort(0x3C0), attrcRstPort(0x3DA) {}
Vga::~Vga() {}
/* switch from real mode to 320x200x256 color vga mode 13h 
        63 bytes total, 1 byte = 1 pixel(0-255)
    misc - sequencerport - crtcport - gpu - attr
    misc: 1 entry
        (N/A, 0x63)
    sequencer: (index, data/+1)
        5 entries, 0x3C4 index, 0x3C5 data
            sr0: 0x03 8-dot char
            sr1: 0x01 bit0=1 dot 9->8 dot/pixels width, screen width 360
                -> 320px width
            sr2: 0x0F write to all 4-planes at once
            sr3: 0x06/0x0E bit3=1, mem mode register, diff font 8x14/256chars+graphics vs 8x8
                     bit1=1, 256-color mode, chain4 disable
                        linear addressing/framebuffer: 
                            A0000h + yx320 + x -> pixel(x, y) directly
    crtc: bit7 protect bit
            crtc[3]: set bit7, allow write protected bits
            crtc[0x11]: enable write crtc[0-7]
        25 entries
        change horizontal timing
            0x80: 1000 0000
                ~0x80:0111 111 bit7 cleared
            unlocking:
                0x03: allow setting protect bit7  
                    | 0x80: force bit7=1 -> set crtc[3] bit7
                0x11: unlock crtc 0-7
                    ~0x80: clear bit7  bit7=0 unlock
            patching/apply to the upcoming arr/loop 
                0X03: 0x82->0x8A bit7=1
                0x11: 0xA3->0x23 bit7=0, unlock 0-7
        write
            standard 720x350 timing: H total 95/100chars, hDisp 80chars 640px, chained 320px, etc
    registers[]: enable write, modify table in ram    
    gpu: 9 entries, 0x3CE index, 0x3CF data
        graphic mode 
    attr: 21 entries, 0x3C0 index+data, 0x3C0 data
            palette 0-15
        dummy read 0x3DA: reset flip-flop
        final 0x20: re-enable video output, bit6=1
*/
void Vga::WriteRegisters(uint8_t* registers) {
    miscPort.Write(*(registers++));
    for (uint8_t i=0; i<5; i++) {
        seqIndexPort.Write(i);
        seqDataPort.Write(*(registers++));
    }
    crtcIndexPort.Write(0x03);
    crtcDataPort.Write(crtcDataPort.Read() | 0x80);
    crtcIndexPort.Write(0x11);
    crtcDataPort.Write(crtcDataPort.Read() & ~0x80);
    registers[0x03] = registers[0x03] | 0x80;
    registers[0x11] = registers[0x11] & ~0x80;
    for (uint8_t i=0; i<25; i++) {
        crtcIndexPort.Write(i);
        crtcDataPort.Write(*(registers++));
    }
    for(uint8_t i=0; i<9; i++) {
        gpuIndexPort.Write(i);
        gpuDataPort.Write(*(registers++));
    }
    for(uint8_t i=0; i<21; i++) {
        attrcRstPort.Read();
        attrcIndexPort.Write(i);
        attrcWritePort.Write(*(registers++));
    }
    attrcRstPort.Read();
    attrcIndexPort.Write(0x20);
}
/* & (3<<2): keep only bits 3 2 /masks
    0<<2 0
    1<<2 0100 4 graphical mode/standard vga 256-color/mode 13h 
    2<<2 1000 8 monochrome text mode/planar
    3<<2 10000 16 color text mode/bga text buffer/video ram 80x25 640x200px 8x8char/font
    attr/color/blinking/higher byte + char/code point 2-byte pair
*/ 
uint8_t* Vga::GetFrameBufferSeg() {
    gpuIndexPort.Write(0x06);
    uint8_t seg = gpuDataPort.Read() & (3<<2);
    switch(seg) {
        default:
        case 0<<2: return (uint8_t*)0x00000;
        case 1<<2: return (uint8_t*)0xA0000;
        case 2<<2: return (uint8_t*)0xB0000;
        case 3<<2: return (uint8_t*)0xB8000;
    }
}
/* 0xa8: 168
    0xff: 255
*/
uint8_t Vga::GetColorIndex(uint8_t r, uint8_t g, uint8_t b) {
    if (r==0x00 && g==0x00 && b==0x00) return 0x00;
    if (r==0x00 && g==0x00 && b==0xA8) return 0x01;
    if (r==0x00 && g==0xA8 && b==0x00) return 0x02;
    if (r==0xA8 && g==0x00 && b==0x00) return 0x03;
    if (r==0xFF && g==0xFF && b==0xFF) return 0x3F;
    return 0x00;
}

bool Vga::SetMode(uint32_t width, uint32_t height, uint32_t colorDepth) {
    if (!SupportsMode(width, height, colorDepth)) return false;
    unsigned char g_320x200x256[] = {
        0x63,

        0x03, 0x01, 0x0F, 0x00, 0x06,

        0x5F, 0x4F, 0x50, 0x82, 0x54, 0x80, 0x8F, 0x1F,
        0x00, 0x41, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x9C, 0x8E, 0x8F, 0x28, 0x40, 0x96, 0xB9, 0xA3, 0xFF,

        0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x05, 0x0F, 0xFF,

        0x00, 0x01, 0x02, 0x03, 0x04, 0x04, 0x06, 0x07, 
        0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 
        0x41, 0x00, 0x0F, 0x00, 0x00
    };
    WriteRegisters(g_320x200x256);
    return true;
}
bool Vga::SupportsMode(uint32_t width, uint32_t height, uint32_t colorDepth) {
    return width==320 && height==200 && colorDepth==0;
}

void Vga::PutPixel(int32_t x, int32_t y, uint8_t colorIndex) {
    if (x<0 || 320<=x || y<0 || 200<=y) return;
    uint8_t* pxAddr = GetFrameBufferSeg() + y*320 + x;
    *pxAddr = colorIndex;
}
void Vga::PutPixel(int32_t x, int32_t y, uint8_t r, uint8_t g, uint8_t b) {
    PutPixel(x, y, GetColorIndex(r, g, b));
}

void Vga::FillRectangle(uint32_t x, uint32_t y, uint32_t w, uint32_t h,
                uint8_t r, uint8_t g, uint8_t b) {
    for (int32_t Y=y; Y<y+h; Y++) for (int32_t X=x; X<x+w; X++) PutPixel(X, Y, r, g, b);
}