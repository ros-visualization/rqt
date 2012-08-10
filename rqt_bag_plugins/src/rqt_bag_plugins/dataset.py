# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

PKG = 'rxbag_plugins'
import roslib; roslib.load_manifest(PKG)

import bisect

class DataSet(object):
    def __init__(self):
        self._points = []
        
        self._num_points = 0

        self._min_x = None
        self._max_x = None
        self._min_y = None
        self._max_y = None

    @property
    def points(self): return self._points

    @property
    def num_points(self): return self._num_points

    @property
    def min_x(self): return self._min_x

    @property
    def max_x(self): return self._max_x

    @property
    def min_y(self): return self._min_y

    @property
    def max_y(self): return self._max_y

    def add(self, x, y):
        pt = (x, y)

        index = bisect.bisect_right(self._points, pt)

        self._points.insert(index, pt)

        self._num_points += 1

        if self._num_points == 1:
            self._min_x = x
            self._max_x = x
            self._min_y = y
            self._max_y = y
        else:
            if x < self._min_x:
                self._min_x = x
            elif x > self._max_x:
                self._max_x = x
            if y < self._min_y:
                self._min_y = y
            elif y > self._max_y:
                self._max_y = y

    def set(self, pts):
        self._points = pts
        
        self._num_points = len(self._points)

        if self._num_points == 0:
            self._min_x = None
            self._max_x = None
            self._min_y = None
            self._max_y = None
        else:
            xs = [x for x, y in pts]
            ys = [y for x, y in pts]
            
            self._min_x = min(xs)
            self._max_x = max(xs)
            self._min_y = min(ys)
            self._max_y = max(ys)
