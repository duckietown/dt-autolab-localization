#!/bin/bash

DUCKIEBOT_NAME=bwstod

for i in {01..07}; do
  dts devel run -C ../ -H watchtower$i --pull --detach
done
dts devel run -C ../ -H tticlargeloop --pull --detach
dts devel run -C ../ -H $DUCKIEBOT_NAME --pull --detach