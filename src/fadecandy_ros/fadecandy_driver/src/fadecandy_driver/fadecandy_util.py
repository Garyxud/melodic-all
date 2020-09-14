from fadecandy_msgs.msg import LEDStrip
from std_msgs.msg import ColorRGBA


def clamp(v, v_min, v_max):
    return max(v_min, min(v_max, v))


def array_to_strip(color_array):
    """
    Convenience function for converting a 3xN array into an LEDStrip message.
    """
    led_strip = LEDStrip()
    for r, g, b in color_array:
        r = clamp(r, 0, 255)
        g = clamp(g, 0, 255)
        b = clamp(b, 0, 255)
        led_strip.colors.append(ColorRGBA(r, g, b, 255))
    return led_strip
