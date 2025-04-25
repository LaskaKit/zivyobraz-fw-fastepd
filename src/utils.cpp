#include <HTTPClient.h>
#include <WiFi.h>

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
        case ZEncoding::Z1:
            return draw_z1_image_chunk(display, buffer, buflen, position);
        case ZEncoding::Z2:
            return draw_z2_image_chunk(display, buffer, buflen, position);
        case ZEncoding::Z3:
            return draw_z3_image_chunk(display, buffer, buflen, position);
    }
    return 0;
}

size_t draw_z1_image_chunk(FASTEPD& display,
                           uint8_t buffer[],
                           size_t buflen,
                           PixelPosition& position)
{
    if (buflen % 2 != 0) {
        Serial.print("WARNING: Buflen is odd.");
    }
    if (!position.is_inside(display)) {
        return 0;
    }
    size_t drawn_pixels = 0;
    for (size_t i = 0; i < buflen - 1; i += 2) {
        Z1Pixel px(buffer[i], buffer[i+1]);
        uint8_t mapped_color = px.color();  // TODO: color mapping for Z1
        for (size_t count = 0; count < px.count(); count++) {
            if (!position.next(display)) {
                return drawn_pixels;
            }
            drawn_pixels++;
            display.drawPixelFast(position.x, position.y, mapped_color);
        }
    }
    return drawn_pixels;
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


uint8_t* readPNG(String url, HTTPClient& http_client)
{
    http_client.begin("https://patrikn.pythonanywhere.com/image/png");
    int httpResponseCode = http_client.GET();
        
    if (httpResponseCode == HTTP_CODE_OK) {
        int len = http_client.getSize();
        Serial.print("Payload size: ");
        Serial.println(http_client.getSize());

        uint8_t* buff = new uint8_t[1000000];

        WiFiClient *stream = http_client.getStreamPtr();
        size_t totalSize = 0;

        unsigned long start = millis();
        while (http_client.connected() && (len > 0 || len == -1)) {
            size_t size = stream->available();
            // Serial.println("AVAILABLE");
            // Serial.println(size);
            if (size) {
                int c = stream->readBytes(buff + totalSize, ((size > 1000000 - totalSize) ? 1000000 - totalSize : size));
                totalSize += c;

                // for (int i = 0; i < 128; i++) {
                //   Serial.print(buff[i]);
                // }
                // Serial.println("CHUNK read.");

                if (len > 0) {
                len -= c;
                }
            }
            yield();
            delay(100);
        }
        unsigned long end = millis();
        Serial.print("Time taken (ms): ");
        Serial.println(end - start);

        Serial.print(totalSize / 1024);
        Serial.println(" Kb");
        return buff;
    }
    return nullptr;
}

