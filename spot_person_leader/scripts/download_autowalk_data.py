#!/usr/bin/env python

from jsk_data import download_data


def main():
    PKG = 'spot_person_leader'

    download_data(
        pkg_name=PKG,
        path='autowalk/eng2_73b2_to_81c1_night.walk.tar.gz',
        url='https://drive.google.com/uc?id=1PXgMsmN3uKPG3YAap9vdfrBduEiLgAl8',
        md5='a79dd142013b12c0babb4400be583739',
        extract=True
    )

    download_data(
        pkg_name=PKG,
        path='autowalk/eng2_73b2_to_7FElevator.walk.tar.gz',
        url='https://drive.google.com/uc?id=1O8o6voq2v8WenfaUYcmpSU-IwJSsXW5_',
        md5='160e2052d51b9a2f56cf9cf1788b18c2',
        extract=True
    )


if __name__ == '__main__':
    main()
