#!/usr/bin/env python3
from bluezero import adapter
from bluezero import device

dongles = adapter.list_adapters()
print('dongles available: ', dongles)
dongle = adapter.Adapter(dongles[0])
tmp = device.Device(dongle.address, '64:B7:08:82:BB:D6')
tmp.connect()
