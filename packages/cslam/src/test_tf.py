#!/usr/bin/env python3

from dt_communication_utils import DTCommunicationGroup
from autolab_msgs.msg import \
    AutolabTransform


group = DTCommunicationGroup("/autolab/tf", AutolabTransform)


def cb(msg, _):
    print(msg)
    print('=' * 80)
    print()


print('Listening...')
group.Subscriber(cb)
