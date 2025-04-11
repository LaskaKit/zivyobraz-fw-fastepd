#include <utils.hpp>


uint8_t map_color_from_z(uint8_t z_color)
{
    // color picker (8 shades of gray)  0 3 5 7 9 a c f
    switch (z_color) {
        case 0:  // white
            return 0xf;
        case 1:  // black
            return 0x0;
        case 2:  // light gray
            return 0xc;
        case 3:  // dark gray
            return 0x7;
        case 4:  // light gray 2
            return 0xa;
        case 5:  // light gray 3
            return 0x9;
        case 6:  // dark gray 2
            return 0x5;
        case 7:  // dark gray 3
            return 0x3;
        default:
            return 0xf;  // default white
    }
}

size_t draw_z_image_chunk(FASTEPD& display,
                          uint8_t buffer[],
                          size_t buflen,
                          PixelPosition& position,
                          ZEncoding encoding)
{
    switch (encoding) {
        case ZEncoding::Z2:
            return draw_z2_image_chunk(display, buffer, buflen, position);
        case ZEncoding::Z3:
            return draw_z3_image_chunk(display, buffer, buflen, position);
    }
    return 0;
}


size_t draw_z2_image_chunk(FASTEPD& display,
                          uint8_t buffer[],
                          size_t buflen,
                          PixelPosition& position)
{
    if (!position.is_inside(display)) {
        return 0;  // do not draw out of bounds
    }

    size_t drawn_pixels = 0;
    for (size_t i = 0; i < buflen; i++) {
        Z2Pixel px(buffer[i]);
        uint8_t mapped_color = map_color_from_z(px.color());
        for (size_t count = 0; count < px.count(); count++) {
            // Serial.print(position.x); Serial.print(" "); Serial.println(position.y);
            if (!position.next(display)) {
                return drawn_pixels;
            }
            drawn_pixels++;
            display.drawPixelFast(position.x, position.y, mapped_color);
        }
    }
    return drawn_pixels;
}


size_t draw_z3_image_chunk(FASTEPD& display,
                          uint8_t buffer[],
                          size_t buflen,
                          PixelPosition& position)
{
    if (!position.is_inside(display)) {
        return 0;  // do not draw out of bounds
    }

    size_t drawn_pixels = 0;
    for (size_t i = 0; i < buflen; i++) {
        Z3Pixel px(buffer[i]);
        uint8_t mapped_color = map_color_from_z(px.color());
        for (size_t count = 0; count < px.count(); count++) {
            // Serial.print(position.x); Serial.print(" "); Serial.println(position.y);
            if (!position.next(display)) {
                return drawn_pixels;
            }
            drawn_pixels++;
            display.drawPixelFast(position.x, position.y, mapped_color);
        }
    }
    return drawn_pixels;
}

