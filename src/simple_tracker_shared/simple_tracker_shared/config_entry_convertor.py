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

import sys

class ConfigEntryConvertor():

    # Method to populate a configuration dictionary for use throughout the simple tracker application
    @staticmethod
    def Convert(type: any, value: any):

        if type == 'int':
            return int(value)

        if type == 'float':
            return float(value)

        if type == 'str':
            return value            

        if type == 'bool':
            return True if value == 'True' else False

        if type == 'null':
            return None

        raise Exception(f"Unknown type [{type}] value [{value}].")
