#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pyrealsense2 as rs
import os

# Create a context
ctx = rs.context()

# Get list of connected devices
devices = ctx.query_devices()

serials = [dev.get_info(rs.camera_info.serial_number) for dev in devices]

for idx, serial in enumerate(serials):
    print(f"Device {idx}: {serial}")

if len(serials) >= 1:
    os.environ["LEFT_WRIST_CAMERA_SERIAL_NO"] = serials[0]
    print(f"LEFT_WRIST_CAMERA_SERIAL_NO={serials[0]}")

if len(serials) >= 2:
    os.environ["RIGHT_WRIST_CAMERA_SERIAL_NO"] = serials[1]
    print(f"RIGHT_WRIST_CAMERA_SERIAL_NO={serials[1]}")

with open(os.path.expanduser("~/.bashrc"), "a") as f:
    if len(serials) >= 1:
        f.write(f'\nexport LEFT_WRIST_CAMERA_SERIAL_NO="{serials[0]}"\n')
    if len(serials) >= 2:
        f.write(f'export RIGHT_WRIST_CAMERA_SERIAL_NO="{serials[1]}"\n')

print("✅ Environment variables written to ~/.bashrc")
print("Please restart your terminal or run 'source ~/.bashrc' to apply the changes.")
print("Done.")