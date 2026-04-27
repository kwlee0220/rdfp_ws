#!/usr/bin/env bash

ros2 topic list | grep -v '_[0-9]' | grep -v collision | grep -v _event | grep -v plan
