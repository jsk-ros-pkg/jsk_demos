#!/usr/bin/env python

import multiprocessing

try:
    import chainer  # NOQA
    _chainer_available = True
except:  # NOQA
    print('### Failed to import chainer')
    _chainer_available = False

import jsk_data


def download_data(*args, **kwargs):
    p = multiprocessing.Process(
            target=jsk_data.download_data,
            args=args,
            kwargs=kwargs)
    p.start()


def main():
    PKG = 'jsk_2018_10_22_stamp'

    # node_scripts/people_pose_estimation_2d.py
    path = 'trained_data/stamp_mask_rcnn.npz'
    if _chainer_available:
        download_data(
            pkg_name=PKG,
            path=path,
            url='https://drive.google.com/'
            'uc?id=1NCMz45lOCVzgR144copsFX_E3VlqlpYx',
            md5='c3cef6ab55bf3a0f30694bb5291de9db',
        )


if __name__ == '__main__':
    main()
