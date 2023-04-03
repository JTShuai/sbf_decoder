# -*- coding: UTF-8 -*-
'''
@Author  ：Jiangtao Shuai
@Date    ：03/04/2023 10:57 AM 
'''


from setuptools import setup, find_packages


setup(
    name='sbf_decoder',
    version='0.1',
    packages=find_packages("src"),
    package_dir={"": "src"},
    install_requires=['gps-time', 'numpy'],
)
