'''
Author: HJX
Date: 2025-04-22 09:40:57
LastEditors: 
LastEditTime: 2025-04-22 09:41:35
FilePath: /piper_aloha/src/piper_aloha/scripts/utils/color_msg.py
Description: 
symbol_custom_string_obkorol_copyright: 
'''
#! /usr/bin/env python3

import time

class ColorMsg():
    def __init__(self,msg: str,color: str = '', timestamp: bool = True) -> None:
        self.msg = msg
        self.color = color
        self.timestamp = timestamp
        self.colorMsg(msg=self.msg, color=self.color, timestamp=self.timestamp)

    def colorMsg(self,msg: str, color: str = '', timestamp: bool = True):
        str = ""
        if timestamp:
            str += time.strftime('%Y-%m-%d %H:%M:%S',
                                time.localtime(time.time())) + "  "
        if color == "red":
            str += "\033[1;31;40m"
        elif color == "green":
            str += "\033[1;32;40m"
        elif color == "yellow":
            str += "\033[1;33;40m"
        else:
            print(str + msg)
            return
        str += msg + "\033[0m"
        print(str)