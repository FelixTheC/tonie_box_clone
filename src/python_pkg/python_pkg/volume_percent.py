#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@created: 18.09.21
@author: felix
"""
import subprocess


def extract_number(text, remove_suffix: str):
    return int(text.replace(remove_suffix, ''))


def read_volume_info():
    process = subprocess.Popen(["amixer", "-c", "0", "cget", "numid=1"], stdout=subprocess.PIPE)
    lines = [line.decode('utf8').strip() for line in process.stdout.readlines()][1:3]
    current_value = extract_number(lines[-1], ': values=')
    min_value = 0
    max_value = 0
    for val in lines[0].split(","):
        if val.startswith('min'):
            min_value = extract_number(val, 'min=')
        elif val.startswith('max'):
            max_value = extract_number(val, 'max=')

    return min_value, max_value, current_value


def calculate_init_percent(min_val, max_val, curr_val):
    # min=-10239, max=400, values=-4919 <- current_value
    g = abs(min_val) - max_val
    w = abs(curr_val)
    return 100 - round((w / g) * 100)


def get_init_volume_percent():
    return calculate_init_percent(*read_volume_info())
