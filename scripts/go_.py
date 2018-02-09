#! /usr/bin/env python
import rospy 
import sys
import argparse
from turtle_follower_ import ARTagFollow

'''
rosrun practice_xb turtle_follower_go.py --dir=/home/xahid/datasets/diver_robot_test/diver/oliv09/
'''

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--program', required=False, dest='program', type=str, default=None, help='Which program to run')
    parser.add_argument('--dir', required=True, dest='dir', type=str, default=None, help='Which program to run')

    args = parser.parse_args()
    go = ARTagFollow(args.dir)
