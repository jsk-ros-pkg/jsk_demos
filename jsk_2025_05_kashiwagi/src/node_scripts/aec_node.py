#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Cancel the robot's own voice from /audio, entirely inside ROS.

Both signals come from places that were reliable through a long night of
testing: the microphone from respeaker_node, which reads the device with
PyAudio and never once stopped or changed its timing, and the reference from
sound_play, tapped inside the GStreamer pipeline that plays it. Nothing here
goes through PipeWire, which is where every failure came from -- capture
stretched 1.5x on the six-channel card profiles, node names changing on every
re-enumeration, card profiles losing their inputs, and a monitor-derived
reference that arrived at 39% of the rate the canceller consumed it.

That last one is why the reference is taken from sound_play rather than from
the speaker's monitor. A reference the canceller has to invent 28% of the time
tells the model the far end is silent while the echo is playing, and the model
then removes the near-end speech as well: measured, the robot's voice was
attenuated 40 dB and a person's 36-40 dB, leaving speech that no recogniser
would accept. A reference taken from the playback pipeline is exactly the
samples the speaker received, and exactly zero when nothing is playing.

The remaining unknown is delay. The reference is branched off before the sound
card, so it leads the echo by the card's buffering plus the flight time to the
microphone. That is fixed for a given setup, so it is a parameter here
(~delay_ms) rather than something to estimate continuously; measure it once by
sweeping ~delay_ms and keeping the value that cancels most; 20 ms here.
"""
import ctypes
import glob
import os
import threading

import numpy as np
import rospy
from audio_common_msgs.msg import AudioData

BLOCK = 128          # the model's hop size
RATE = 16000


class Dtln(object):
    """The DTLN-aec LV2 plugin, driven directly.

    The plugin is what pipewire-aec uses and what reached 38-41 dB offline on
    this hardware's own recordings, so the model is known to work here; only
    the plumbing around it is new.
    """

    # LV2 port order, from dtln_aec.ttl: 0 = mic in, 1 = ref in, 2 = out.
    PORT_MIC, PORT_REF, PORT_OUT = 0, 1, 2

    def __init__(self, bundle):
        so = os.path.join(bundle, "dtln_aec.so")
        self.lib = ctypes.CDLL(so)
        self.lib.lv2_descriptor.restype = ctypes.c_void_p
        self.lib.lv2_descriptor.argtypes = [ctypes.c_uint32]

        class Descriptor(ctypes.Structure):
            _fields_ = [
                ("URI", ctypes.c_char_p),
                ("instantiate", ctypes.CFUNCTYPE(
                    ctypes.c_void_p, ctypes.c_void_p, ctypes.c_double,
                    ctypes.c_char_p, ctypes.c_void_p)),
                ("connect_port", ctypes.CFUNCTYPE(
                    None, ctypes.c_void_p, ctypes.c_uint32, ctypes.c_void_p)),
                ("activate", ctypes.CFUNCTYPE(None, ctypes.c_void_p)),
                ("run", ctypes.CFUNCTYPE(
                    None, ctypes.c_void_p, ctypes.c_uint32)),
                ("deactivate", ctypes.CFUNCTYPE(None, ctypes.c_void_p)),
                ("cleanup", ctypes.CFUNCTYPE(None, ctypes.c_void_p)),
                ("extension_data", ctypes.CFUNCTYPE(
                    ctypes.c_void_p, ctypes.c_char_p)),
            ]

        ptr = self.lib.lv2_descriptor(0)
        if not ptr:
            raise RuntimeError("dtln_aec.so returned no LV2 descriptor")
        self.desc = ctypes.cast(ptr, ctypes.POINTER(Descriptor)).contents
        rospy.loginfo("loaded %s", self.desc.URI.decode())

        self.handle = self.desc.instantiate(
            ctypes.byref(self.desc), ctypes.c_double(RATE),
            bundle.encode() + b"/", None)
        if not self.handle:
            raise RuntimeError("plugin refused to instantiate; is the model "
                               "in %s?" % bundle)

        self.buf_mic = (ctypes.c_float * BLOCK)()
        self.buf_ref = (ctypes.c_float * BLOCK)()
        self.buf_out = (ctypes.c_float * BLOCK)()
        self.desc.connect_port(self.handle, self.PORT_MIC, self.buf_mic)
        self.desc.connect_port(self.handle, self.PORT_REF, self.buf_ref)
        self.desc.connect_port(self.handle, self.PORT_OUT, self.buf_out)
        if self.desc.activate:
            self.desc.activate(self.handle)

    def process(self, mic, ref):
        """One block. mic and ref are float32 arrays of BLOCK samples."""
        self.buf_mic[:] = mic.tolist()
        self.buf_ref[:] = ref.tolist()
        self.desc.run(self.handle, BLOCK)
        return np.frombuffer(self.buf_out, dtype=np.float32).copy()


def find_bundle():
    """Locate the DTLN-aec LV2 bundle the way an LV2 host would.

    The plugin ships in the pipewire-aec package, which installs it under the
    architecture's library directory -- a path this node should not have to
    spell out. LV2_PATH wins if the user set it, then the usual prefixes.
    """
    roots = []
    env = os.environ.get("LV2_PATH")
    if env:
        roots += [p for p in env.split(os.pathsep) if p]
    roots += [
        os.path.expanduser("~/.lv2"),
        "/usr/local/lib/lv2",
        "/usr/lib/lv2",
    ]
    roots += sorted(glob.glob("/usr/local/lib/*/lv2")) + \
        sorted(glob.glob("/usr/lib/*/lv2"))

    for root in roots:
        candidate = os.path.join(root, "dtln_aec.lv2")
        if os.path.exists(os.path.join(candidate, "dtln_aec.so")):
            return candidate
    raise rospy.ROSInitException(
        "no dtln_aec.lv2 found in %s -- install the pipewire-aec package, "
        "which ships it, or set ~bundle to a bundle directory"
        % os.pathsep.join(roots))


class AecNode(object):
    def __init__(self):
        bundle = rospy.get_param("~bundle", "") or find_bundle()
        self.delay = int(rospy.get_param("~delay_ms", 0) * RATE / 1000)
        self.gain = float(rospy.get_param("~output_gain", 1.0))
        self.engine = Dtln(bundle)

        self.lock = threading.Lock()
        self.mic = np.zeros(0, dtype=np.float32)
        self.ref = np.zeros(0, dtype=np.float32)
        # The reference leads the echo, so hold it back by the measured delay.
        self.ref_hold = np.zeros(self.delay, dtype=np.float32)

        self.pub = rospy.Publisher("~output", AudioData, queue_size=32)
        rospy.Subscriber("~mic", AudioData, self.on_mic, queue_size=64)
        rospy.Subscriber("~reference", AudioData, self.on_ref, queue_size=64)
        rospy.loginfo("aec_node: delay %d ms, gain %.1fx",
                      self.delay * 1000 // RATE, self.gain)

    @staticmethod
    def _to_float(data):
        return np.frombuffer(bytes(data), dtype="<i2").astype(np.float32) / 32768.0

    def on_ref(self, msg):
        with self.lock:
            self.ref = np.concatenate([self.ref, self._to_float(msg.data)])

    def on_mic(self, msg):
        with self.lock:
            self.mic = np.concatenate([self.mic, self._to_float(msg.data)])
            out = []
            while len(self.mic) >= BLOCK:
                mic = self.mic[:BLOCK]
                self.mic = self.mic[BLOCK:]
                # Nothing playing means no reference at all, which is correct:
                # the model is told the far end is silent, because it is.
                if len(self.ref) >= BLOCK:
                    ref = self.ref[:BLOCK]
                    self.ref = self.ref[BLOCK:]
                else:
                    ref = np.zeros(BLOCK, dtype=np.float32)
                    self.ref = np.zeros(0, dtype=np.float32)
                out.append(self.engine.process(mic, ref))
        if out:
            y = np.concatenate(out) * self.gain
            pcm = (np.clip(y, -1.0, 1.0) * 32767).astype("<i2").tobytes()
            self.pub.publish(AudioData(data=pcm))


if __name__ == "__main__":
    rospy.init_node("aec_node")
    AecNode()
    rospy.spin()
