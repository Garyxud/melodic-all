# -*- Python -*-
# -*- coding: utf-8 -*-

'''rtctree

Copyright (C) 2009-2014
    Geoffrey Biggs
    RT-Synthesis Research Group
    Intelligent Systems Research Institute,
    National Institute of Advanced Industrial Science and Technology (AIST),
    Japan
    All rights reserved.
Licensed under the Eclipse Public License -v 1.0 (EPL)
http://www.opensource.org/licenses/eclipse-1.0.txt

Objects and functions used to build and store a tree representing a hierarchy
of name servers, directories, managers and components.

'''


from omniORB import any
import SDOPackage
import sys


##############################################################################
## API functions


term_attributes = {'reset': '00',
                   'bold': '01',
                   'faint': '02',
                   'underline': '04',
                   'blink': '05',
                   'blinkfast': '06',
                   'negative': '07',
                   'normal': '22',
                   'nounderline': '24',
                   'noblink': '25',
                   'positive': '27',
                   'black': '30',
                   'red': '31',
                   'green': '32',
                   'brown': '33',
                   'blue': '34',
                   'purple': '35',
                   'cyan': '36',
                   'white': '37',
                   'bgblack': '40',
                   'bgred': '41',
                   'bggreen': '42',
                   'bgbrown': '43',
                   'bgblue': '44',
                   'bgpurple': '45',
                   'bgcyan': '46',
                   'bgwhite': '47',
                   }

from traceback import extract_stack

def build_attr_string(attrs, supported=True):
    '''Build a string that will turn any ANSI shell output the desired
    colour.

    attrs should be a list of keys into the term_attributes table.

    '''
    if not supported:
        return ''
    if type(attrs) == str:
        attrs = [attrs]
    result = '\033['
    for attr in attrs:
        result += term_attributes[attr] + ';'
    return result[:-1] + 'm'


def colour_supported(term):
    if sys.platform == 'win32':
        return False
    return term.isatty()


def get_num_columns_and_rows(widths, gap_width, term_width):
    '''Given a list of string widths, a width of the minimum gap to place
    between them, and the maximum width of the output (such as a terminal
    width), calculate the number of columns and rows, and the width of each
    column, for the optimal layout.

    '''
    def calc_longest_width(widths, gap_width, ncols):
        longest = 0
        rows = [widths[s:s + ncols] for s in range(0, len(widths), ncols)]
        col_widths = rows[0] # Column widths start at the first row widths
        for r in rows:
            for ii, c in enumerate(r):
                if c > col_widths[ii]:
                    col_widths[ii] = c
            length = sum(col_widths) + gap_width * (ncols - 1)
            if length > longest:
                longest = length
        return longest, col_widths

    def calc_num_rows(num_items, cols):
        div, mod = divmod(num_items, cols)
        return div + (mod != 0)

    # Start with one row
    ncols = len(widths)
    # Calculate the width of the longest row as the longest set of item widths
    # ncols long and gap widths (gap_width * ncols - 1) that fits within the
    # terminal width.
    while ncols > 0:
        longest_width, col_widths = calc_longest_width(widths, gap_width, ncols)
        if longest_width < term_width:
            # This number of columns fits
            return calc_num_rows(len(widths), ncols), ncols, col_widths
        else:
            # This number of columns doesn't fit, so try one less
            ncols -= 1
    # If got here, it all has to go in one column
    return len(widths), 1, 0


def get_terminal_size():
    '''Finds the width of the terminal, or returns a suitable default value.'''
    def read_terminal_size_by_ioctl(fd):
        try:
            import struct, fcntl, termios
            cr = struct.unpack('hh', fcntl.ioctl(1, termios.TIOCGWINSZ,
                                                            '0000'))
        except ImportError:
            return None
        except IOError, e:
            return None
        return cr[1], cr[0]

    cr = read_terminal_size_by_ioctl(0) or \
            read_terminal_size_by_ioctl(1) or \
            read_terminal_size_by_ioctl(2)
    if not cr:
        try:
            import os
            fd = os.open(os.ctermid(), os.O_RDONLY)
            cr = read_terminal_size_by_ioctl(fd)
            os.close(fd)
        except:
            pass
    if not cr:
        import os
        cr = [80, 25] # 25 rows, 80 columns is the default value
        if os.getenv('ROWS'):
            cr[1] = int(os.getenv('ROWS'))
        if os.getenv('COLUMNS'):
            cr[0] = int(os.getenv('COLUMNS'))

    return cr[1], cr[0]


def dict_to_nvlist(dict):
    '''Convert a dictionary into a CORBA namevalue list.'''
    result = []
    for item in dict.keys() :
        result.append(SDOPackage.NameValue(item, any.to_any(dict[item])))
    return result


def nvlist_to_dict(nvlist):
    '''Convert a CORBA namevalue list into a dictionary.'''
    result = {}
    for item in nvlist :
        result[item.name] = item.value.value()
    return result


def filtered(path, filter):
    '''Check if a path is removed by a filter.

    Check if a path is in the provided set of paths, @ref filter. If
    none of the paths in filter begin with @ref path, then True is
    returned to indicate that the path is filtered out. If @ref path is
    longer than the filter, and starts with the filter, it is
    considered unfiltered (all paths below a filter are unfiltered).

    An empty filter ([]) is treated as not filtering any.

    '''
    if not filter:
        return False
    for p in filter:
        if len(path) > len(p):
            if path[:len(p)] == p:
                return False
        else:
            if p[:len(path)] == path:
                return False
    return True


def trim_filter(filter, levels=1):
    '''Trim @ref levels levels from the front of each path in @filter.'''
    trimmed = [f[levels:] for f in filter]
    return [f for f in trimmed if f]


# vim: tw=79

