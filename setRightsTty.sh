#!/bin/bash

mv /dev/ttyS0 /dev/ttyS0.backup
ln -s /dev/ttyUSB0 /dev/ttyS0
chmod 777 /dev/ttyUSB0
