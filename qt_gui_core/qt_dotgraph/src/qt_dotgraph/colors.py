import colorsys


def get_color_for_string(name):
    hue = float(name.__hash__() % 0xFFFFFFFFFFFF) / 0xFFFFFFFFFFFF
    # choose colors with high saturation and value
    (r, g, b) = colorsys.hsv_to_rgb(hue, 1, 0.8)
    # convert floats into hexadecimal
    (r, g, b) = (int(r * 0xFF), int(g * 0xFF), int(b * 0xFF))
    # build a hex string like #a3bbc0
    code = '#%s%s%s' % (hex(r)[-2:], hex(g)[-2:], hex(b)[-2:])
    # single digit hexes yield 'x4''
    return code.replace('x', '0')
