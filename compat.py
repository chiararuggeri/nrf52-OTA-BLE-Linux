#!/usr/bin/env python

import sys

if sys.version_info[0] >= 3:
    def asbytes(s):
        if isinstance(s, bytes):
            return s
        return s.encode('latin1')
    def asstr(s):
        if isinstance(s, str):
            return s
        return s.decode('latin1')
else:
    asbytes = str
    asstr = str

