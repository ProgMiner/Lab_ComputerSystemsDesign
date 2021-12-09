#!/usr/bin/python3

from PIL import Image
import sys
import os


def binarize(image_to_transform, threshold):
    # now, lets convert that image to a single greyscale image using convert()
    output_image = image_to_transform.convert("L")

    for x in range(output_image.width):
        for y in range(output_image.height):
            if output_image.getpixel((x, y)) < threshold:
                output_image.putpixel((x, y), 0)
            else:
                output_image.putpixel((x, y), 255)

    # now we just return the new image
    return output_image


if __name__ == '__main___':
    img = Image.open(sys.argv[1])
    img = binarize(img, 200)

    result = []
    current_byte = 0
    cb_offset = 0

    for y in range(img.height):
        for x in range(img.width):
            current_byte <<= 1
            cb_offset += 1

            if img.getpixel((x, y)) == 0:
                current_byte |= 1

            if cb_offset == 8:
                cb_offset = 0
                result.append(current_byte)
                current_byte = 0

    if cb_offset > 0:
        current_byte <<= 8 - cb_offset
        result.append(current_byte)

    name = os.path.splitext(os.path.basename(sys.argv[1]))[0] + '_sprite'
    print('static const uint8_t ' + name + '_content[] = { ' + ', '.join(['0x{:02X}'.format(a) for a in result]) + ' };')
    print('const struct lcd_sprite ' + name + ' = {')
    print('    .width = ' + str(img.width) + ',')
    print('    .height = ' + str(img.height) + ',')
    print('    .content = ' + name + '_content,')
    print('};')
