#!/usr/bin/env python

from jsk_data import download_data


def main():
    PKG = 'spot_person_leader'

    download_data(
        pkg_name=PKG,
        path='autowalk/eng2_73B2_to_81C1.walk.tar.gz',
        url='https://drive.google.com/uc?id=1IDTCP7n4LCowizW3mFQvTOQtE4qOH9tx',
        md5='65f09629c0ac2aa21df6f179c9875bd0',
        extract=True
    )

    download_data(
        pkg_name=PKG,
        path='autowalk/eng2_73B2_to_7FElevator.walk.tar.gz',
        url='https://drive.google.com/uc?id=1O8o6voq2v8WenfaUYcmpSU-IwJSsXW5_',
        md5='ecd4d8dc043995f7675a59fce950676b',
        extract=True
    )

    download_data(
        pkg_name=PKG,
        path='autowalk/eng2_7FElevator_to_2FElevator.walk.tar.gz',
        url='https://drive.google.com/uc?id=12MOg5okckmQlYiM6flkdeMYxqjn9-C9l',
        md5='67ae3210cbfb55791fff6494f84abb3b',
        extract=True
    )


if __name__ == '__main__':
    main()
