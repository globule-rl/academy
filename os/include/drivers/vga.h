#pragma once

#include <common/types.h>
#include <hwCom/port.h>
#include <drivers/driver.h>

/* crtc: cathode-ray tube controller
    seq: sequencer
        dac: color logic
        palette:
    mem: 4 planes of 64k
    gpu: graphics controller
        bus interface: system bus
    (index, data) index, next port/port+1 write data
    miscport: port 0x3C2, miscellaneous output
        0x3C2 for write, 0x3CC read, bit0 control other registers
            cleared: 0x3D4 mapped to 0x3B4
                        0x3DA mapped to 0x3BA
        (N/A, 0x63)
    sequencerport: port 0x3C4
             index, port+1->data, 2bytes/16-bit
        (0x01, 0x01) clk mode
        (0x01, 0x00) 9/8 dot mode
            each char = 9/8 dot    
        (0x03, 0x00) char select
        (0x04, 0x0E) mem mode
            index
                0x02, bit3:0 mem plane write enable
                0x04, bit3:2 chain4 odd/even disable 
    crtcport: 
            0x3D4: require misc set, 0-7 protected bit
                bit7 of index 0x11 
        (0x00, 0x5F) total horizontal
        (0x01, 0x4F) end display horizontal
        (0x02, 0x50) start horizontal blank
        (0x03, 0x82) end horizontal blank
        (0x04, 0x54) start horizontal retrace
        (0x05, 0x80) end horizontal retrace
        (0x06, 0xBF) total vertical
        (0x07, 0x1F) overflow register
        (0x08, 0x00) preset row scan
        (0x09, 0x41) max scan line
        (0x10, 0x9C) vertical retrace start
        (0x11, 0x8E) vertical retrace end
        (:0x17, :0xA3) mode control
    gpuport: 0x3CE
        (0x05, 0x40) mode register: read/write mode 0
        (0x06, 0x05) graphic mode
            index 
                0x00 bit3:0 set/reset val
                0x01 bit3:0 enable set/reset
                0x02 bit3:0 meme plane write enable
                0x03 bit4:3 logical op bit2:0 rotate cnt
                0x05 bit3 read mode, bit1:0 write mode
                0x08 bit7:0 bit mask
    attrc: attrcibute controller
        port 0x3C0 write/read index/data bytes
            read from 0x3DA, go to index state 
            feed to 0x3C0, read from 0xC1, read 0x3DA data/index 
        (0x10, 0x41) mode control
        (0x11, 0x00) overscan
        (0x12, 0x0F) color plane enable
        (0x13, 0x00) horizontal panning
        (0x14, 0x00) color select  
    & 0x3F/blank 0x1F/retrace
    port 0x3C6: DAC mask
    port 0x3C7: control DAC/digital-to-analog converter 
            18-bit, 6-bit->each color               
         0x3C8: write to 0x3C8, write 3bytes/others to 0x3C9 red, green, blue...
         0x3C9
    text mode
        4-color mode
        256-color mode
    gpu modes
        3 80x25 text mode
        12h 640x480 planar 16-color mode
        13h 320x200 linear 256-color mode
        x   320x240 planar 256-color mode
    4 planes/chain 4 bit: access planar mode -> linear mode
    datasheet: https://wiki.osdev.org/VGA_Hardware
*/
namespace os {
    namespace drivers {
        class Vga {
            protected:
                hwCom::Port8Bit miscPort;
                hwCom::Port8Bit crtcIndexPort;
                hwCom::Port8Bit crtcDataPort;
                hwCom::Port8Bit seqIndexPort;
                hwCom::Port8Bit seqDataPort;
                hwCom::Port8Bit gpuIndexPort;
                hwCom::Port8Bit gpuDataPort;
                hwCom::Port8Bit attrcIndexPort;
                hwCom::Port8Bit attrcReadPort;
                hwCom::Port8Bit attrcWritePort;
                hwCom::Port8Bit attrcRstPort;

                void WriteRegisters(uint8_t* registers);
                uint8_t* GetFrameBufferSeg();
                virtual uint8_t GetColorIndex(uint8_t r, uint8_t g, uint8_t b);
            public:
                Vga();
                ~Vga();
                virtual bool SetMode(uint32_t width, uint32_t height, uint32_t colorDepth);
                virtual bool SupportsMode(uint32_t width, uint32_t height, uint32_t colorDepth);
                
                virtual void PutPixel(int32_t x, int32_t y, uint8_t colorIndex);virtual void PutPixel(int32_t x, int32_t y, uint8_t r, uint8_t g, uint8_t b);
                virtual void PutPixel(int32_t x, int32_t y, uint8_t colorIndex);
                virtual void FillRectangle(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint8_t r, uint8_t g, uint8_t b);
        };
    }
}