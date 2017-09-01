#!/bin/bash

set -xe

rosbag record \
	/vrpn_client_node/drone3/pose
