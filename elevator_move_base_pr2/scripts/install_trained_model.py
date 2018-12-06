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
        path='trained_model/fcn8s_atonce_door_button_model.npz',
        url='https://drive.google.com/uc?id=1kDKStQw886u-zYPbTBcPDH3W80de3Nt_',
        md5='49c5ad7569990ce3a42a5fa499b98deb',
        quiet=quiet,
    )


if __name__ == '__main__':
    main()
