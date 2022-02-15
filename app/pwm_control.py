import usb.core
import usb.util
import time
import struct

# find our device
dev = usb.core.find(idVendor=0x726c, idProduct=0x3103)

# was it found?
if dev is None:
    raise ValueError('Device not found')

# set the active configuration. With no arguments, the first
# configuration will be the active one
dev.reset()

dev.set_configuration()

# get an endpoint instance
cfg = dev.get_active_configuration()
intf = cfg[(0,0)]
usb.util.claim_interface(dev, intf)


ep = usb.util.find_descriptor(
    intf,
    # match the first OUT endpoint
    custom_match = \
    lambda e: \
        usb.util.endpoint_direction(e.bEndpointAddress) == \
        usb.util.ENDPOINT_OUT)

assert ep is not None


epin = usb.util.find_descriptor(
    intf,
    # match the first OUT endpoint
    custom_match = \
    lambda e: \
        usb.util.endpoint_direction(e.bEndpointAddress) == \
        usb.util.ENDPOINT_IN)

assert epin is not None

import datetime

pwm_state = [0x80,0x800,0xf00,0x4ff]
pwm_period = 0xfff
pwm_direction = [1,1,1,1]
pwm_move = 50

# Write a set of zeros to turn everything off
out = struct.pack("<bLLLLL", 1, pwm_period, 0,0,0,0)
ep.write(out)
time.sleep(0.5)

while True:

    #print(datetime.datetime.now())
    out = struct.pack("<bLLLLL", 1, pwm_period, pwm_state[0], pwm_state[1], pwm_state[2], pwm_state[3])
    ep.write(out)
    inp = epin.read(64)
    print("Set ", end='')
    for x in range(0, len(pwm_state)):
        print("%d = %0.1f%% " % (x, float(pwm_state[x]) / pwm_period * 100), end='')
        next = (pwm_state[x] + (pwm_direction[x] * 50))
        if next > pwm_period:
            pwm_direction[x] = -1
        elif next < 0:
            pwm_direction[x] = 1
        else:
            pwm_state[x] = next
    print()
    time.sleep(0.01)


usb.util.release_interface(dev, intf)
