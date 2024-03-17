#!/usr/bin/env python3

from fastapi import FastAPI
import os


import os
from pydub import AudioSegment
import tempfile
import hashlib
import os
import asyncio

import edge_tts as et
from pydantic import BaseModel

class SpeechRequest(BaseModel):
    text: str
    lang: str
    output_path: str


def et_save_to_file(text, voice="en-GB-SoniaNeural", file="test.mp3"):
    async def wrapper():
        c = et.Communicate(text, voice)
        await c.save(file)

    loop = asyncio.get_event_loop_policy().get_event_loop()
    try:
        loop.run_until_complete(wrapper())
    finally:
        loop.close()


def get_cache_dir():
    """Return cache dir.

    Returns
    -------
    cache_dir : str
        cache directory.
    """
    ros_home = os.getenv('ROS_HOME', os.path.expanduser('~/.ros'))
    pkg_ros_home = os.path.join(ros_home, 'edge')
    default_cache_dir = os.path.join(pkg_ros_home, 'cache')
    cache_dir = os.environ.get(
        'ROS_GOOGLE_TEXTTOSPEECH_CACHE_DIR',
        default_cache_dir)
    if not os.path.exists(cache_dir):
        os.makedirs(cache_dir)
    return cache_dir


def checksum_md5(filename, blocksize=8192):
    """Calculate md5sum.

    Parameters
    ----------
    filename : str or pathlib.Path
        input filename.
    blocksize : int
        MD5 has 128-byte digest blocks (default: 8192 is 128x64).
    Returns
    -------
    md5 : str
        calculated md5sum.
    """
    filename = str(filename)
    hash_factory = hashlib.md5()
    with open(filename, 'rb') as f:
        for chunk in iter(lambda: f.read(blocksize), b''):
            hash_factory.update(chunk)
    return hash_factory.hexdigest()


def convert_to_str(x):
    if isinstance(x, str):
        pass
    elif isinstance(x, bytes):
        x = x.decode('utf-8')
    else:
        raise ValueError(
            'Invalid input x type: {}'
            .format(type(x)))
    return x


async def request_synthesis(
        sentence, output_path, lang='en'):
    sentence = convert_to_str(sentence)
    # mp3_path = tempfile.mktemp('.mp3')
    mp3_path = '/tmp/hoge.mp3'
    if lang == 'en':
        voice = 'en-US-AnaNeural'
    else:
        voice = 'ja-JP-NanamiNeural'
    c = et.Communicate(sentence, voice)
    print('communicate')
    await c.save(mp3_path)
    print('save')
    AudioSegment.from_mp3(mp3_path).export(
        output_path, format='wav')
    print(output_path)


app = FastAPI()
@app.post("/text-to-speech/")
async def text_to_speech(request: SpeechRequest):
    print('hoge')
    request_synthesis(request.text, request.output_path, request.lang)


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8075)
