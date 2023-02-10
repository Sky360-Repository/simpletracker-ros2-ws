# Original work Copyright (c) 2022 Sky360
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

from enum import IntEnum

class DayNightEnum(IntEnum):
    Unknown = 0
    Day = 1
    Night = 2

class TrackingStateEnum(IntEnum):
    PROVISIONARY_TARGET = 1
    ACTIVE_TARGET = 2
    LOST_TARGET = 3