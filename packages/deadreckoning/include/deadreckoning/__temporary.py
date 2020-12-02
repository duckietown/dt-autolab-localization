def _is_fixed(frame: str) -> bool:
    # static tags are those within the range [300, 399]
    if frame.startswith('tag/'):
        frame_id = int(frame.split('/')[1])
        if 300 <= frame_id <= 399:
            return True
    return False


def _is_static(frame: str) -> bool:
    return _is_fixed(frame) or frame.startswith('watchtower')


def _is_wheel(msg) -> bool:
    return msg.transforms[0].header.frame_id.startswith('autobot')


def _is_excluded(msg) -> bool:
    tf = msg.transforms[0]
    return tf.child_frame_id.startswith('tag/136')


def _is_duckiebot_tag(msg) -> bool:
    tf = msg.transforms[0]
    return tf.header.frame_id.startswith('watchtower') and not _is_fixed(tf.child_frame_id)


def _is_ground_tag(msg) -> bool:
    tf = msg.transforms[0]
    return tf.header.frame_id.startswith('watchtower') and _is_fixed(tf.child_frame_id)


def _is_of_interest(msg) -> bool:
    origin = msg.transforms[0].header.frame_id
    target = msg.transforms[0].child_frame_id
    return 'watchtower01' in origin and '310' in target


def _type(frame: str) -> str:
    return frame.split('/')[0]