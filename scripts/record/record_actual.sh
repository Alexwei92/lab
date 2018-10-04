#!/bin/bash

set -xe

rosbag record \
	/vrpn_client_node/drone5/pose
