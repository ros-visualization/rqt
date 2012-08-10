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
import rospy

import sys
import threading
import time

import wx

import Image

from rxbag import bag_helper, TimelineCache, TimelineRenderer

import image_helper

class ImageTimelineRenderer(TimelineRenderer):
    """
    Draws thumbnails of sensor_msgs/Image or sensor_msgs/CompressedImage in the timeline.
    """
    def __init__(self, timeline, thumbnail_height=160):
        TimelineRenderer.__init__(self, timeline, msg_combine_px=40.0)

        self.thumbnail_height     = thumbnail_height

        self.thumbnail_combine_px = 20.0                 # use cached thumbnail if it's less than this many pixels away
        self.min_thumbnail_width  = 8                    # don't display thumbnails if less than this many pixels across
        self.quality              = Image.NEAREST        # quality hint for thumbnail scaling

        self.thumbnail_cache = TimelineCache(self._load_thumbnail, lambda topic, msg_stamp, thumbnail: wx.CallAfter(self.timeline.Refresh))

    # TimelineRenderer implementation

    def get_segment_height(self, topic):
        return self.thumbnail_height

    def draw_timeline_segment(self, dc, topic, stamp_start, stamp_end, x, y, width, height):
        max_interval_thumbnail = self.timeline.map_dx_to_dstamp(self.thumbnail_combine_px)
        
        max_interval_thumbnail = max(0.1, max_interval_thumbnail)

        thumbnail_gap = 6

        thumbnail_x, thumbnail_y, thumbnail_height = x + 1, y + 1, height - 2 - thumbnail_gap  # leave 1px border

        dc.set_source_rgb(1, 1, 1)
        dc.rectangle(x, y, width, height - thumbnail_gap)
        dc.fill()

        thumbnail_width = None
        thumbnail_right = None
            
        while True:
            available_width = (x + width) - thumbnail_x

            # Check for enough remaining to draw thumbnail
            if width > 1 and available_width < self.min_thumbnail_width:
                break

            # Try to display the thumbnail, if its right edge is to the right of the timeline's left side
            if not thumbnail_width or thumbnail_x + thumbnail_width >= self.timeline.history_left:
                stamp = self.timeline.map_x_to_stamp(thumbnail_x, clamp_to_visible=False)
    
                thumbnail_bitmap = self.thumbnail_cache.get_item(topic, stamp, max_interval_thumbnail)
    
                # Cache miss
                if not thumbnail_bitmap:
                    thumbnail_details = (thumbnail_height,)
                    self.thumbnail_cache.enqueue((topic, stamp, max_interval_thumbnail, thumbnail_details))

                    if not thumbnail_width:
                        break
                else:
                    thumbnail_width = thumbnail_bitmap.get_width()

                    if width > 1:
                        if available_width < thumbnail_width:
                            # Space remaining, but have to chop off thumbnail
                            thumbnail_width = available_width - 1
        
                    dc.set_source_surface(thumbnail_bitmap, thumbnail_x, thumbnail_y)
                    dc.rectangle(thumbnail_x, thumbnail_y, thumbnail_width, thumbnail_height)
                    dc.fill()

            thumbnail_x += thumbnail_width

            if width == 1:
                break

        # Draw 1px black border
        dc.set_line_width(1)
        dc.set_source_rgb(0, 0, 0)
        if width == 1:
            dc.rectangle(x, y, thumbnail_x - x, height - thumbnail_gap - 1)
        else:
            dc.rectangle(x, y, width, height - thumbnail_gap - 1)
        dc.stroke()

        return True
    
    def close(self):
        if self.thumbnail_cache:
            self.thumbnail_cache.stop()
            self.thumbnail_cache.join()

    #

    def _load_thumbnail(self, topic, stamp, thumbnail_details):
        """
        Loads the thumbnail from the bag
        """
        (thumbnail_height,) = thumbnail_details
        
        # Find position of stamp using index
        t = rospy.Time.from_sec(stamp)
        bag, entry = self.timeline.get_entry(t, topic)
        if not entry:
            return None, None
        pos = entry.position

        # Not in the cache; load from the bag file
        
        with self.timeline._bag_lock:
            msg_topic, msg, msg_stamp = bag._read_message(pos)
        
        # Convert from ROS image to PIL image
        try:
            pil_image = image_helper.imgmsg_to_pil(msg)
        except Exception, ex:
            print >> sys.stderr, 'Error loading image on topic %s: %s' % (topic, str(ex)) 
            return None, None
        
        if not pil_image:
            return None, None
        
        # Calculate width to maintain aspect ratio
        try:
            pil_image_size = pil_image.size
            thumbnail_width = int(round(thumbnail_height * (float(pil_image_size[0]) / pil_image_size[1])))
    
            # Scale to thumbnail size
            thumbnail = pil_image.resize((thumbnail_width, thumbnail_height), self.quality)
    
            # Convert from PIL Image to Cairo ImageSurface
            thumbnail_bitmap = image_helper.pil_to_cairo(thumbnail)
            
            return msg_stamp, thumbnail_bitmap
        
        except Exception, ex:
            print >> sys.stderr, 'Error loading image on topic %s: %s' % (topic, str(ex))
            raise
            return None, None
