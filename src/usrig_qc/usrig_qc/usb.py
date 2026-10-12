"""USB link speed of the RealSense from sysfs (no librealsense needed)."""
from pathlib import Path

INTEL_VID = '8086'


def realsense_usb(serial=None):
    """List of {path, product, serial, speed_mbps} for Intel RealSense devices on the bus."""
    out = []
    for d in Path('/sys/bus/usb/devices').glob('*'):
        try:
            if (d / 'idVendor').read_text().strip() != INTEL_VID:
                continue
            product = (d / 'product').read_text().strip() if (d / 'product').exists() else ''
            if 'RealSense' not in product:
                continue
            ser = (d / 'serial').read_text().strip() if (d / 'serial').exists() else ''
            out.append({'path': d.name, 'product': product, 'serial': ser,
                        'speed_mbps': int(float((d / 'speed').read_text().strip()))})
        except OSError:
            continue
    # The USB serial string is not always the camera serial printed by rs-enumerate-devices
    # (NOT VERIFIED for the D415): filter only when it matches.
    if serial and any(x['serial'] == str(serial) for x in out):
        out = [x for x in out if x['serial'] == str(serial)]
    return out
