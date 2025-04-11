#include <FastEPD.h>
#include <HTTPClient.h>

enum class ZEncoding {Z1, Z2, Z3, UNKNOWN};


struct Z1Pixel
{
    uint8_t px_color;
    uint8_t px_count;
    Z1Pixel(uint16_t data)
    {
        px_color = data >> 8;
        px_count = data;
    }
    inline uint8_t color() { return px_color; }
    inline uint8_t count() { return px_count; }
};


struct Z2Pixel
{
    uint8_t data;
    Z2Pixel(uint8_t data) : data(data) {}
    inline uint8_t color() { return data >> 6; }
    inline uint8_t count() { return data & (0xFF >> 2); }
};


struct Z3Pixel
{
    uint8_t data;
    Z3Pixel(uint8_t data) : data(data) {}
    inline uint8_t color() { return data >> 5; }
    inline uint8_t count() { return data & (0xFF >> 3); }
};


struct PixelPosition
{
    size_t x;
    size_t y;

    PixelPosition() : x(0), y(0) {}
    PixelPosition(size_t x, size_t y) : x(x), y(y) {}

    bool is_inside(FASTEPD& display)
    {
        return this->x < display.width() && this->y < display.height();
    }

    /**
     * Update the position to the next pixel on the display.
     * The position is updated row by row.
     * 
     * @return bool False if position is out of bounds otherwise true.
    */
    bool next(FASTEPD& display)
    {
        if (++this->x >= display.width()) {
            this->x = 0;
            if (++this->y >= display.height()) {
                return false;
            }
        }
        return true;
    }
};


/**
 * Map color from z1/2/3 encoded pixel space to display
 * color space. Custom made mapping.
*/
uint8_t map_color_from_z(uint8_t z_color);


/**
 * Draws content of a buffer encoded in either one
 * of Z image encodings.
*/
size_t draw_z_image_chunk(FASTEPD& display,
                          uint8_t buffer[],
                          size_t buflen,
                          PixelPosition& positon,
                          ZEncoding encoding);



/**
 * Draws contents of Z2 encoded buffer onto a display.
 * 
 * @return int The number of drawn pixels.
*/
size_t draw_z2_image_chunk(FASTEPD& display,
                          uint8_t buffer[],
                          size_t buflen,
                          PixelPosition& positon);



/**
 * Draws contents of Z3 encoded buffer onto a display.
 * 
 * @return int The number of drawn pixels.
*/
size_t draw_z3_image_chunk(FASTEPD& display,
                          uint8_t buffer[],
                          size_t buflen,
                          PixelPosition& positon);



uint8_t* readPNG(String url, HTTPClient& client);
