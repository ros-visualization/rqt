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

from __future__ import with_statement

PKG = 'rxbag_plugins'
import roslib; roslib.load_manifest(PKG)
import rospy

import bisect
import collections
import csv
import itertools
import sys
import threading
import time

from dataset import DataSet

class PlotDataLoader(threading.Thread):
    def __init__(self, timeline, topic):
        threading.Thread.__init__(self, target=self._run)

        self._timeline = timeline
        self._topic    = topic

        self._start_stamp        = self._timeline.start_stamp
        self._end_stamp          = self._timeline.end_stamp
        self._paths              = []
        self._max_interval       = 0.0
        self._extension_fraction = 0.1
        self._min_reload_secs    = 0.4                            # minimum time to wait before loading entries
        self._use_header_stamp   = False                          # use the timestamp from the header

        self._dirty         = True
        self._dirty_cv      = threading.Condition()
        self._last_reload   = None                           # time that entries were reloaded
        self._data          = {}
        self._load_complete = False

        self._progress_listeners = []
        self._complete_listeners = []

        self._stop_flag = False
        self.setDaemon(True)

    @property
    def data(self): return self._data

    @property
    def is_load_complete(self): return self._load_complete

    # listeners

    def add_progress_listener(self, listener):    self._progress_listeners.append(listener)
    def remove_progress_listener(self, listener): self._progress_listeners.remove(listener)

    def add_complete_listener(self, listener):    self._complete_listeners.append(listener)
    def remove_complete_listener(self, listener): self._complete_listeners.remove(listener)

    def invalidate(self):
        with self._dirty_cv:
            self._dirty = True
            self._dirty_cv.notify()

    # property: start_stamp

    def _get_start_stamp(self): return self._start_stamp
    
    def _set_start_stamp(self, start_stamp):
        with self._dirty_cv:
            if start_stamp != self._start_stamp:
                self._start_stamp = start_stamp
                self._dirty = True
                self._dirty_cv.notify()

    start_stamp = property(_get_start_stamp, _set_start_stamp)

    # property: end_stamp

    def _get_end_stamp(self): return self._end_stamp
    
    def _set_end_stamp(self, end_stamp):
        with self._dirty_cv:
            if end_stamp != self._end_stamp:
                self._end_stamp = end_stamp
                self._dirty = True
                self._dirty_cv.notify()
        
    end_stamp = property(_get_end_stamp, _set_end_stamp)

    def set_interval(self, start_stamp, end_stamp):
        with self._dirty_cv:
            updated = False
            if start_stamp != self._start_stamp:
                self._start_stamp = start_stamp
                updated = True
            if end_stamp != self._end_stamp:
                self._end_stamp = end_stamp
                updated = True
            if updated:
                self._dirty = True
                self._dirty_cv.notify()

    # property: paths

    def _get_paths(self): return self._paths

    def _set_paths(self, paths):
        with self._dirty_cv:
            if set(paths) != set(self._paths):
                self._data = {}
                self._paths = paths
                self._dirty = True
                self._dirty_cv.notify()

    paths = property(_get_paths, _set_paths)

    # property: max_interval
    
    def _get_max_interval(self): return self._max_interval
    
    def _set_max_interval(self, max_interval):
        with self._dirty_cv:
            if max_interval != self._max_interval:
                self._max_interval = max_interval
                self._dirty = True
                self._dirty_cv.notify()

    max_interval = property(_get_max_interval, _set_max_interval)

    def stop(self):
        self._stop_flag = True
        with self._dirty_cv:
            self._dirty_cv.notify()
        self.join()

    ##

    def _run(self):
        entry_queue = []
        
        while not self._stop_flag:
            # If dirty, then throw away data which doesn't fit in current view, and regenerate the entries to load
            with self._dirty_cv:
                if self._dirty and (self._last_reload is None or time.time() - self._last_reload >= self._min_reload_secs):
                    self._trim_data(self._extension_fraction, self._max_interval)

                    entry_queue = self._get_entries_to_load(self._extension_fraction, self._max_interval)

                    self._last_reload = time.time()
                    self._load_complete = False
                    self._dirty = False

            # Check to see if we've finished loading
            if len(entry_queue) == 0 or len(self._paths) == 0:
                self._load_complete = True
                for listener in itertools.chain(self._progress_listeners, self._complete_listeners):
                    listener()

                # Wait for dirty flag to be set
                with self._dirty_cv:
                    if not self._dirty:
                        self._dirty_cv.wait()
                        
                continue

            # Pop the next entry to load, and process it (insert it into _data)
            try:
                bag, entry = entry_queue.pop()

                self._process(bag, entry)
            except Exception:
                pass

    def _trim_data(self, extension_fraction=None, max_interval=None):
        """
        Toss out data outside of (extended) view range, and closer than max_interval seconds apart.
        """
        if extension_fraction is None:
            start_stamp = self._start_stamp
            end_stamp   = self._end_stamp
        else:
            extension = rospy.Duration((self._end_stamp - self._start_stamp).to_sec() * extension_fraction)
            if extension.to_sec() >= self._start_stamp.to_sec():
                start_stamp = rospy.Time(0, 1)
            else:
                start_stamp = self._start_stamp - extension
            end_stamp = self._end_stamp + extension

        min_x = (start_stamp - self._timeline.start_stamp).to_sec()
        max_x = (end_stamp   - self._timeline.start_stamp).to_sec()

        for series in list(self._data.keys()):
            points     = self._data[series].points
            num_points = len(points)

            trimmed_points = []

            if num_points > 0 and points[0][0] < max_x and points[-1][0] > min_x:
                first_index = None
                last_x = None
                for i, (x, y) in enumerate(points):
                    if x >= min_x:
                        trimmed_points.append((x, y))
                        first_index = i
                        last_x = x
                        break

                if first_index is not None:
                    for i, (x, y) in enumerate(points[first_index + 1:]):
                        if x > max_x:
                            break

                        if (max_interval is None) or (x - last_x >= max_interval):
                            trimmed_points.append((x, y))
                            last_x = x

            new_data = DataSet()
            new_data.set(trimmed_points)

            self._data[series] = new_data

    def _get_entries_to_load(self, extension_fraction=None, max_interval=None):
        """
        Returns a list of (Bag, IndexEntry) tuples to load.

        @param extension: extra proportion of the view range to load
        @param max_interval: maximum 
        """
        # Load data outside of the view range
        if extension_fraction is None:
        	start_stamp = self._start_stamp
        	end_stamp   = self._end_stamp
        else:
            extension = rospy.Duration((self._end_stamp - self._start_stamp).to_sec() * extension_fraction)
            if extension.to_sec() >= self._start_stamp.to_sec():
                start_stamp = rospy.Time(0, 1)
            else:
                start_stamp = self._start_stamp - extension
            end_stamp = self._end_stamp + extension

        # Get the entries
        view_entries = list(self._timeline.get_entries_with_bags(self._topic, start_stamp, end_stamp))

        # Toss out those entries too close to each other
        if max_interval is not None:
            spaced_entries = []
            max_interval_duration = rospy.Duration(max_interval)
            
            last_time = None
            for bag, entry in view_entries:
                if last_time is None or (entry.time - last_time) > max_interval_duration:
                    spaced_entries.append((bag, entry))
                    last_time = entry.time
        else:
            spaced_entries = view_entries

        # Toss out entries too close to existing data
        if max_interval is not None:
            far_entries = []
            loaded_xs = set()
            for series in list(self._data.keys()):
                for x, y in self._data[series].points:
                    loaded_xs.add(x)
            loaded_xs = list(loaded_xs)
            loaded_xs.sort()
            loaded_stamps = [self._timeline.start_stamp + rospy.Duration(x) for x in loaded_xs]
            for bag, entry in spaced_entries:
                closest_stamp_left_index = bisect.bisect_left(loaded_stamps, entry.time) - 1
                if closest_stamp_left_index >= 0 and abs((entry.time - loaded_stamps[closest_stamp_left_index]).to_sec()) < max_interval / 2:
                    continue
                closest_stamp_right_index = closest_stamp_left_index + 1
                if closest_stamp_right_index < len(loaded_stamps) - 1 and abs((entry.time - loaded_stamps[closest_stamp_right_index]).to_sec()) < max_interval / 2:
                    continue
                far_entries.append((bag, entry))
        else:
            far_entries = spaced_entries

        # Reorder to load using bisection
        bin_search_entries = []
        if len(far_entries) > 0:
            for i in _subdivide(0, len(far_entries) - 1):
                bin_search_entries.append(far_entries[i])

        entry_queue = list(reversed(bin_search_entries))  # reversed so that pop from front of list
        
        return entry_queue

    def _process(self, bag, entry):
        # Read the message from the bag file
        _, msg, msg_stamp = self._timeline.read_message(bag, entry.position)
        if not msg:
            return False

        # Extract the field data from the message
        for path in self._paths:
            # Get Y value
            try:
                y = eval('msg.' + path)
            except Exception:
                continue

            # Get X value
            if self._use_header_stamp:
                if msg.__class__._has_header:
                    header = msg.header
                else:
                    header = _get_header(msg, path)

                stamp = header.stamp
            else:
                stamp = msg_stamp
            x = (stamp - self._timeline.start_stamp).to_sec()

            # Store data
            if path not in self._data:
                self._data[path] = DataSet()
            self._data[path].add(x, y)

        # Notify listeners of progress
        for listener in self._progress_listeners:
            listener()

        return True

def _get_header(msg, path):
    fields = path.split('.')
    if len(fields) <= 1:
        return None

    parent_path = '.'.join(fields[:-1])

    parent = eval('msg.' + parent_path)
    
    for slot in parent.__slots__:
        subobj = getattr(parent, slot)
        if subobj is not None and hasattr(subobj, '_type') and getattr(subobj, '_type') == 'roslib/Header':
            return subobj

    return _get_header(msg, parent_path)

def _subdivide(left, right):
    if left == right:
        yield left
        return
    
    yield left
    yield right

    intervals = collections.deque([(left, right)])
    while True:
        try:
            (left, right) = intervals.popleft()
        except Exception:
            break

        mid = (left + right) / 2

        if right - left <= 1:
            continue

        yield mid

        if right - left <= 2:
            continue

        intervals.append((left, mid))
        intervals.append((mid,  right))
