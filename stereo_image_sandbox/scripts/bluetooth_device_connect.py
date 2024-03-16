#!/usr/bin/env python

from bluezero import adapter
from bluezero import device
from bluezero import dbus_tools


def known_devices(name_only=True):
    """Get a list of devices Bluez already knows about"""
    device_list = []
    mng_objs = dbus_tools.get_managed_objects()
    for path in mng_objs:
        address = mng_objs[path].get('org.bluez.Device1', {}).get('Address')
        if address:
            if name_only:
                device_list.append(str(address))
            else:
                device_list.append(mng_objs[path])
    return device_list


def connect_bluetooth_device(target_name):
    dongles = adapter.list_adapters()
    dongle = adapter.Adapter(dongles[0])
    for dev in known_devices(name_only=False):
        name = dev.get('org.bluez.Device1', {}).get('Alias')
        if name == target_name:
            mac_addr = dev.get('org.bluez.Device1', {}).get('Address')
            mac_addr = str(mac_addr)
            print('Connecting {} ({})'.format(target_name, mac_addr))
            tmp = device.Device(dongle.address, mac_addr)
            tmp.connect()
            print('Successfully connected {} ({})'.format(
                target_name, mac_addr))


if __name__ == '__main__':
    connect_bluetooth_device('M5_SPEAKER_T1')
