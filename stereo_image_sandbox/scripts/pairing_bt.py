#!/usr/bin/env python3

from bluezero import adapter
from bluezero import device
from bluezero import dbus_tools


def known_devices():
    """Get a list of devices Bluez already knows about"""
    device_list = []
    mng_objs = dbus_tools.get_managed_objects()
    for path in mng_objs:
        address = mng_objs[path].get('org.bluez.Device1', {}).get('Address')
        if address:
            device_list.append(str(address))
    return device_list


def is_known_device(mac_addr):
    """Is the given Bluetooth address already a known device"""
    found_devices = known_devices()
    return mac_addr in found_devices


found_device: device.Device = None


def on_device_found(device: device.Device, device_name='M5_SPEAKER_T1'):
    global found_device
    try:
        print(device.address)
        print(device.name)
        if device.name == device_name:
            found_device = device
    except Exception as e:
        print(str(e))


def main():
    dongles = adapter.list_adapters()
    print('dongles available: ', dongles)
    dongle = adapter.Adapter(dongles[0])

    if not dongle.powered:
        dongle.powered = True
        print('Now powered: ', dongle.powered)
    print('Start discovering')

    dongle.on_device_found = on_device_found
    dongle.nearby_discovery(timeout=20)

    if found_device is not None:
        print("pairing {}".format(found_device.address))
        found_device.pair()
        found_device.trusted = True
    else:
        print('failed')


if __name__ == '__main__':
    main()
