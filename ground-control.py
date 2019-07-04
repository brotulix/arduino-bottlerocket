# coding: utf-8
# Ground control station script for arduino-bottlerocket
# See https://github.com/brotulix/arduino-bottlerocket for details.
# In essence, this script should:
# * Accept input commands from keyboard
# * Forward commands to bottle rocket flight computer via serial port (and RF link)
#   - Set up the RF link via special bytes written to serial port
# * Receive status messages from bottle rocket flight computer
# * Compute relevant values from status messages (eg. convert pressure in hPa to meters altitude)
# * Keep historical data, store each launch to its own file, and plot the current launch in realtime.
# * Allow reading previous launches from file to plot and study (thrust, peak altitude, etc)

# Let's start getting the serial port handled.
