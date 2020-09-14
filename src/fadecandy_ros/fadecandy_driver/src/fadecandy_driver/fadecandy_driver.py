import struct

import usb.core
import usb.util

LEDS_PER_PACKET = 21
LOOKUP_VALUES_PER_PACKET = 31
LOOKUP_VALUES_PER_CHANNEL = 257
USB_PACKET_SIZE = 64
PACKET_TYPE_VIDEO = 0x00
PACKET_TYPE_LUT = 0x40
FINAL_PACKET_BIT = 0x20

USB_VENDOR_ID = 0x1d50
USB_PRODUCT_ID = 0x607a
USB_ENDPOINT = 1

LEDS_PER_STRIP = 64
NUM_STRIPS = 8

# 1 control byte, then 21 led values: r1, g1, b1, r2, b2, c2, ...., r21, g21, c21
VIDEO_PACKET_STRUCT = struct.Struct('B' + 'B' * 3 * LEDS_PER_PACKET)
LOOKUP_TABLE_PACKET_STRUCT = struct.Struct('BB' + 'H' * LOOKUP_VALUES_PER_PACKET)


def make_video_usb_packets(led_array_colors):
    """
    Construct the USB packets to set all LED strips to the given colors.

    To simplify things, we always send values for all 8 * 64 LEDs. If the physical strips
    are shorter, or there are less then 8 strips, the extra data doesn't do anything.

    If the user gives us values for less than the total number of strips, or less than the
    total number of LEDs in any given strip, all unspecified LEDs are left dark.
    """
    all_led_colors = [(0, 0, 0)] * LEDS_PER_STRIP * NUM_STRIPS
    for led_strip_i, led_strip_colors in enumerate(led_array_colors):
        for led_i, led_color in enumerate(led_strip_colors):
            led_index = led_strip_i * LEDS_PER_STRIP + led_i
            all_led_colors[led_index] = led_color
    assert (len(all_led_colors) == LEDS_PER_STRIP * NUM_STRIPS)

    packets = []
    remaining_leds = all_led_colors
    while len(remaining_leds) > 0:
        packet_leds = remaining_leds[:LEDS_PER_PACKET]
        remaining_leds = remaining_leds[LEDS_PER_PACKET:]

        # The first byte of each packet is the control byte. For the first N-1 packets, this is
        # is just the index (starting at 0). For the last packet, we also set a bit to indicate
        # that we're done sending.
        control = len(packets) | PACKET_TYPE_VIDEO
        if len(remaining_leds) == 0:
            control |= FINAL_PACKET_BIT

        color_bytes = []
        for r, g, b in packet_leds:
            color_bytes.extend((r, g, b))

        # Pad with zeros to make an entire 64 byte packet. The fadecandy should ignore these bytes
        # anyway. (padding should only happen on the last packet)
        color_bytes.extend([0] * (63 - len(color_bytes)))

        packet = VIDEO_PACKET_STRUCT.pack(control, *color_bytes)
        assert (len(packet) == USB_PACKET_SIZE)
        packets.append(packet)
    return packets


def make_lookup_table_packets(red_lookup_values, green_lookup_values, blue_lookup_values):
    """
    Create USB packets for a simple color lookup table.

    The entire red lookup table comes first, then the entire green channel, then the entire red
    channel.
    """
    assert (len(red_lookup_values) == LOOKUP_VALUES_PER_CHANNEL)
    assert (len(green_lookup_values) == LOOKUP_VALUES_PER_CHANNEL)
    assert (len(blue_lookup_values) == LOOKUP_VALUES_PER_CHANNEL)
    remaining_lookup_values = red_lookup_values + green_lookup_values + blue_lookup_values
    packets = []
    while len(remaining_lookup_values) > 0:
        packet_lookup_values = remaining_lookup_values[:LOOKUP_VALUES_PER_PACKET]
        remaining_lookup_values = remaining_lookup_values[LOOKUP_VALUES_PER_PACKET:]

        # The first byte of each packet is the control byte. For the first N-1 packets, this is
        # is just the index (starting at 0) or'd with the "lookup packet type" bit. For the
        # last packet, we also set a bit to indicate that we're done sending.
        control = len(packets) | PACKET_TYPE_LUT
        if len(remaining_lookup_values) == 0:
            control |= FINAL_PACKET_BIT

        # Pad with zeros to make an entire 64 byte packet. The fadecandy should ignore these bytes
        # anyway. (padding should only happen on the last packet)
        packet_lookup_values.extend([0] * (LOOKUP_VALUES_PER_PACKET - len(packet_lookup_values)))

        packet = LOOKUP_TABLE_PACKET_STRUCT.pack(control, 0, *packet_lookup_values)
        assert (len(packet) == USB_PACKET_SIZE)
        packets.append(packet)
    return packets


def make_default_lookup_table():
    """
    Return lookup tables as 3 lists of lookup values - one for the red channel, one for the green
    channel, and one for the blue channel.
    """
    # Color correction curve borrowed from the USB example in the main fadecandy repo:
    #
    #  https://github.com/scanlime/fadecandy/blob/master/examples/python/usb-lowlevel.py
    #
    lookup_values = [min(0xFFFF, int(pow(row / 256.0, 2.2) * 0x10000)) for row in range(257)]
    return lookup_values, lookup_values, lookup_values


class FadecandyDriver:
    def __init__(self):
        # Find usb device.
        self._device = usb.core.find(idVendor=USB_VENDOR_ID, idProduct=USB_PRODUCT_ID)
        if not self._device:
            raise IOError('No Fadecandy interfaces found')

        # Connect to device.
        try:
            self._device.set_configuration()
            self.serial_number = usb.util.get_string(self._device, self._device.iSerialNumber)
        except usb.core.USBError as e:
            raise IOError('Found Fadecandy device but could not connect')

        # Set configuration flags.
        flags = 0x00
        self._device.write(USB_ENDPOINT, '\x80' + chr(flags) + ('\x00' * 62))

        # Setup basic color lookup table.
        red_lookup_values, green_lookup_values, blue_lookup_values = make_default_lookup_table()
        lookup_table_packets = make_lookup_table_packets(red_lookup_values, green_lookup_values,
                                                         blue_lookup_values)
        for packet in lookup_table_packets:
            self._device.write(USB_ENDPOINT, packet)

    def set_colors(self, led_colors):
        usb_packets = make_video_usb_packets(led_colors)
        for packet in usb_packets:
            self._device.write(1, packet)
