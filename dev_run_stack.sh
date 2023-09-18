#!/bin/bash

DUCKIEBOT_NAME=bwstod

for i in {01..07}; do
  dts devel run -H watchtower$i --pull --detach
done
dts devel run -H tticlargeloop --pull --detach
dts devel run -H $DUCKIEBOT_NAME --pull --detach