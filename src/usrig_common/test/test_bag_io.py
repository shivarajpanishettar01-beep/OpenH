import json

import pytest

from usrig_common.bag_io import tolerate_crlf_msgdefs

METADATA = 'realsense2_camera_msgs/msg/Metadata'
# as ros-humble-realsense2-camera installs it (and rosbag2 stores it in the bag): CRLF after the first line
METADATA_MSGDEF_CRLF = 'std_msgs/Header header\r\nstring json_data\n'


def _bag_with_crlf_metadata(path):
    """A bag whose stored Metadata definition has the CRLF line, with one message on the wrist metadata topic."""
    from rosbags.rosbag2 import StoragePlugin, Writer
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore
    ts = get_typestore(Stores.ROS2_HUMBLE)
    ts.register(get_types_from_msg('std_msgs/Header header\nstring json_data', METADATA))
    msgdef, _ = ts.generate_msgdef(METADATA, ros_version=2)
    Msg, Header, Time = ts.types[METADATA], ts.types['std_msgs/msg/Header'], ts.types['builtin_interfaces/msg/Time']
    msg = Msg(header=Header(stamp=Time(sec=1, nanosec=2), frame_id='d415'), json_data=json.dumps({'frame_number': 7}))
    with Writer(path, version=9, storage_plugin=StoragePlugin.MCAP) as w:
        conn = w.add_connection('/wrist/d415/color/metadata', METADATA,
                                msgdef=msgdef.replace('std_msgs/Header header\n', 'std_msgs/Header header\r\n', 1),
                                rihs01=ts.hash_rihs01(METADATA))
        w.write(conn, 1_000_000_002, ts.serialize_cdr(msg, METADATA))
    return path


def test_bag_with_crlf_realsense_metadata_definition_opens(tmp_path, monkeypatch):
    """The installed realsense Metadata.msg has a CRLF line: rosbags could not open any bag recording the wrist
    metadata topic (bag_to_lerobot stopped on it; QC and openh_export would too). The definition the test stores
    really has the CRLF, and the stock reader really fails on it."""
    from rosbags.highlevel import AnyReader, AnyReaderError, anyreader
    from rosbags.typesys import get_types_from_msg as stock
    # the stock parser, whatever another test already installed; monkeypatch restores the module afterwards
    monkeypatch.setattr(anyreader, 'get_types_from_msg', stock)
    bag = _bag_with_crlf_metadata(tmp_path / 'bag')
    with pytest.raises(AnyReaderError, match='Could not parse'):            # the bug
        with AnyReader([bag]):
            pass
    tolerate_crlf_msgdefs()
    patched = anyreader.get_types_from_msg
    tolerate_crlf_msgdefs()                                                  # idempotent: not wrapped twice
    assert anyreader.get_types_from_msg is patched and patched is not stock
    with AnyReader([bag]) as r:
        assert '\r' in r.connections[0].msgdef.data
        (c, t, raw), = list(r.messages())
        m = r.deserialize(raw, c.msgtype)
    assert c.topic == '/wrist/d415/color/metadata' and t == 1_000_000_002
    assert json.loads(m.json_data) == {'frame_number': 7} and m.header.frame_id == 'd415'
