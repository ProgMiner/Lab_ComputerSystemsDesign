#!/usr/bin/python3

from PIL import Image
import sys
import os


def binarize(image_to_transform, threshold):
    # now, lets convert that image to a single greyscale image using convert()
    output_image=image_to_transform.convert("L")
    # the threshold value is usually provided as a number between 0 and 255, which
    # is the number of bits in a byte.
    # the algorithm for the binarization is pretty simple, go through every pixel in the
    # image and, if it's greater than the threshold, turn it all the way up (255), and
    # if it's lower than the threshold, turn it all the way down (0).
    # so lets write this in code. First, we need to iterate over all of the pixels in the
    # image we want to work with
    for x in range(output_image.width):
        for y in range(output_image.height):
            # for the given pixel at w,h, lets check its value against the threshold
            if output_image.getpixel((x,y)) < threshold: #note that the first parameter is actually a tuple object
                # lets set this to zero
                output_image.putpixel( (x,y), 0 )
            else:
                # otherwise lets set this to 255
                output_image.putpixel( (x,y), 255 )
    #now we just return the new image
    return output_image


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
