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
