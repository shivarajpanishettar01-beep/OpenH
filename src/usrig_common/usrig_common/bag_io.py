"""Reading rig bags with rosbags.

ros-humble-realsense2-camera ships msg/Metadata.msg with a CRLF line ending ("std_msgs/Header header\\r\\n").
rosbag2 stores that text in every bag that records /wrist/d415/color/metadata, and rosbags' AnyReader parses
every stored definition when it opens a bag (a default_typestore does not stop it), so the whole bag fails with
"Could not parse". Call tolerate_crlf_msgdefs() before opening a bag: it normalises line endings in stored
definitions. bag_to_lerobot (micrus, never modified here) is run through export/run_bag_to_lerobot.py for the same.
"""


def tolerate_crlf_msgdefs():
    """Make rosbags' AnyReader accept message definitions with CRLF / CR line endings. Idempotent."""
    from rosbags.highlevel import anyreader
    parse = anyreader.get_types_from_msg
    if getattr(parse, 'crlf_tolerant', False):
        return

    def get_types_from_msg(text, name):
        return parse(text.replace('\r\n', '\n').replace('\r', '\n'), name)

    get_types_from_msg.crlf_tolerant = True
    anyreader.get_types_from_msg = get_types_from_msg
