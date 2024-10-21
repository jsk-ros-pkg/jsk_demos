#!/usr/bin/env python3

import collections
import array
from threading import Lock

from audio_common_msgs.msg import AudioData
from jsk_hark_msgs.msg import HarkPower
import numpy as np
import rospy
from jsk_topic_tools import ConnectionBasedTransport


class AudioStream(object):

    def __init__(self, topic_name='/audio',
                 input_sample_rate=16000, output_sample_rate=None,
                 buffer_size=None,
                 depth=16,
                 n_channel=2,
                 get_latest_data=False,
                 discard_data=True,
                 is_speeching_topic_name=None):
        self.is_subscribing = True
        self.get_latest_data = get_latest_data
        self.discard_data = discard_data
        self.audio_buffer_len = buffer_size or input_sample_rate * 5
        self.lock = Lock()
        self.depth = depth
        self.n_channel = n_channel
        self.input_sample_rate = input_sample_rate
        self.output_sample_rate = output_sample_rate or self.input_sample_rate
        self.type_code = {}
        self.is_speeching_topic_name = is_speeching_topic_name
        for code in ['b', 'h', 'i', 'l']:
            self.type_code[array.array(code).itemsize] = code

        self.dtype = self.type_code[self.depth / 8]
        self.clear()

        self.topic_name = topic_name
        self.sub_audio = rospy.Subscriber(
            self.topic_name, AudioData, self.audio_cb)

    def clear(self):
        self.audio_buffer = np.zeros(shape=(0, 2), dtype=self.dtype)

    def close(self):
        try:
            self.sub_audio.unregister()
        except Exception:
            pass

    def _read(self, size, normalize=False):
        with self.lock:
            if self.get_latest_data:
                audio_buffer = self.audio_buffer[-size:]
            else:
                audio_buffer = self.audio_buffer[:size]
                if self.discard_data:
                    self.audio_buffer = self.audio_buffer[size:]
        if self.input_sample_rate != self.output_sample_rate:
            raise NotImplementedError
        if normalize is True:
            audio_buffer = audio_buffer / self.max_value
        return audio_buffer

    def sufficient_data(self, size):
        return len(self.audio_buffer) < size

    def read(self, size, normalize=False):
        if self.input_sample_rate and self.output_sample_rate:
            size = int(size * (self.input_sample_rate
                               / self.output_sample_rate))
        while not rospy.is_shutdown() and len(self.audio_buffer) < size:
            rospy.sleep(0.001)
        return self._read(size, normalize=normalize)

    def close(self):
        try:
            self.sub_audio.unregister()
        except Exception:
            pass
        self.audio_buffer = np.array([], dtype=self.dtype)

    def audio_cb(self, msg):
        audio_buffer = np.frombuffer(msg.data, dtype=self.dtype)
        audio_buffer = audio_buffer.reshape(-1, self.n_channel)
        with self.lock:
            self.audio_buffer = np.append(
                self.audio_buffer, audio_buffer)
            self.audio_buffer = self.audio_buffer[
                -self.audio_buffer_len:]

maxValue = float(2 ** 16)
SOUND_SPEED = 340.0
MIC_DISTANCE = 0.058
MAX_TDOA = MIC_DISTANCE / float(SOUND_SPEED)


class Element(object):
    def __init__(self):
        self.sinks = []

    def put(self, data):
        for sink in self.sinks:
            sink.put(data)

    def start(self):
        pass

    def stop(self):
        pass

    def link(self, sink):
        if hasattr(sink, 'put') and callable(sink.put):
            self.sinks.append(sink)
        else:
            raise ValueError('Not implement put() method')

    def unlink(self, sink):
        self.sinks.remove(sink)

    def pipeline(self, *args):
        source = self
        for sink in args:
            source.link(sink)
            source = sink

        return self

    def pipeline_start(self):
        def recursive_start_sink(s):
            if s:
                # start downstream first
                if hasattr(s, 'sinks'):
                    for sink in s.sinks:
                        recursive_start_sink(sink)
                s.start()

        recursive_start_sink(self)

    recursive_start = pipeline_start

    def pipeline_stop(self):
        def recursive_stop_sink(s):
            if s:
                # stop upstream first
                s.stop()
                if hasattr(s, 'sinks'):
                    for sink in s.sinks:
                        recursive_stop_sink(sink)

        recursive_stop_sink(self)

    recursive_stop = pipeline_stop



def gcc_phat(sig, refsig, fs=1, max_tau=None, interp=1):
    '''
    This function computes the offset between the signal sig and the reference signal refsig
    using the Generalized Cross Correlation - Phase Transform (GCC-PHAT)method.
    '''

    # make sure the length for the FFT is larger or equal than len(sig) + len(refsig)
    n = sig.shape[0] + refsig.shape[0]

    # Generalized Cross Correlation Phase Transform
    SIG = np.fft.rfft(sig, n=n)
    REFSIG = np.fft.rfft(refsig, n=n)
    R = SIG * np.conj(REFSIG)

    cc = np.fft.irfft(R / np.abs(R), n=(interp * n))

    max_shift = int(interp * n / 2)
    if max_tau:
        max_shift = np.minimum(int(interp * fs * max_tau), max_shift)

    cc = np.concatenate((cc[-max_shift:], cc[:max_shift + 1]))

    # find max cross correlation index
    shift = np.argmax(np.abs(cc)) - max_shift

    tau = shift / float(interp * fs)

    return tau, cc


class DOA(Element):
    def __init__(self, rate=16000):
        super(DOA, self).__init__()
        self.sample_rate = rate

    def get_direction(self, buf):
        tmp = buf.reshape(-1, 2)
        dataL = tmp[:, 0]
        dataR = tmp[:, 1]
        tau, _ = gcc_phat(dataL, dataR, fs=self.sample_rate, max_tau=MAX_TDOA, interp=4)
        theta = np.arcsin(tau / MAX_TDOA) * 180 / np.pi

        peakL = np.abs(1.0 * np.max(dataL) - 1.0 * np.min(dataL)) / maxValue
        peakR = np.abs(1.0 * np.max(dataR) - 1.0 * np.min(dataR)) / maxValue
        return theta, peakL, peakR


class DOANode(ConnectionBasedTransport):

    def __init__(self):
        super(DOANode, self).__init__()
        self.stream = None
        self.pub = self.advertise('/doa',
                                  HarkPower,
                                  queue_size=1)
        self.doa = DOA()

    def subscribe(self):
        input_sample_rate = 16000
        self.stream = AudioStream(buffer_size=input_sample_rate * 1)

    def unsubscribe(self):
        self.stream.sub_audio.unregister()
        del self.stream
        self.stream = None

    def run(self):
        rate = rospy.Rate(1)
        msg = HarkPower()
        while not rospy.is_shutdown():
            rate.sleep()
            if self.stream is None:
                continue

            if self.stream is not None and len(self.stream.audio_buffer) == 0:
                rospy.loginfo('waiting input audio topic')
                continue
            theta, peakL, peakR = self.doa.get_direction(self.stream.audio_buffer)
            if self.stream is not None:
                self.stream.clear()
            rospy.loginfo('Direction of Arrival: {} degree, peakL: {}, peakR: {}'.format(
                theta, peakL, peakR))
            msg.header.stamp = rospy.Time.now()
            msg.directions = int(theta)
            msg.powers = [peakL, peakR]
            self.pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('pi_hat_mic_doa')
    node = DOANode()
    node.run()
