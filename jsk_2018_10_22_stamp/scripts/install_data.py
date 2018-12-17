#!/usr/bin/env python

import multiprocessing

import jsk_data


def download_data(*args, **kwargs):
    p = multiprocessing.Process(
            target=jsk_data.download_data,
            args=args,
            kwargs=kwargs)
    p.start()


def main():
    PKG = 'jsk_2018_10_22_stamp'

    path = 'data/eng2_73B2.pcd'
    download_data(
        pkg_name=PKG,
        path=path,
        url='https://drive.google.com/'
        'uc?id=19Hp1nWtduw6eO3fXmhSQKKGeGu_a_Jq9',
        md5='ecc7c6984e671c03b7cf328a9f5de80b')


if __name__ == '__main__':
    main()
