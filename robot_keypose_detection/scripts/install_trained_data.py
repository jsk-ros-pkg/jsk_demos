#!/usr/bin/env python

from __future__ import print_function

import argparse
import multiprocessing

import jsk_data


def download_data(*args, **kwargs):
    p = multiprocessing.Process(
            target=jsk_data.download_data,
            args=args,
            kwargs=kwargs)
    p.start()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-v', '--verbose', dest='quiet', action='store_false')
    args = parser.parse_args()
    quiet = args.quiet

    PKG = 'robot_keypose_detection'

    download_data(
        pkg_name=PKG,
        path='trained_data/pr2_keypose_model.chainermodel',
        url='https://drive.google.com/uc?id=1kpMcyJoZCn3vjxXsH-x-vHjgwzpjJ-vZ',
        md5='4e9c44b88697f5e1ee00e1e56fddcb8a',
        quiet=quiet,
    )


if __name__ == '__main__':
    main()
