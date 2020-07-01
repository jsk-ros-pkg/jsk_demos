#!/usr/bin/env python

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

    PKG = 'elevator_move_base_pr2'

    download_data(
        pkg_name=PKG,
        path='trained_model/fcn8s_at_once_door_button_model_20191025.npz',
        url='https://drive.google.com/uc?id=1CmEWO0kyTPHrQ-D4kb8bIy3yGsWV3RBd',
        md5='f255461e5b04a0b05e04b5aedefa5618',
        quiet=quiet,
    )


if __name__ == '__main__':
    main()
