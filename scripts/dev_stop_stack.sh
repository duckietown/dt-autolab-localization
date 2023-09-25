#!/bin/bash

DUCKIEBOT_NAME=bwstod
CONTAINER_NAME=dts-run-dt-autolab-localization

for i in {01..07}; do
    docker -H watchtower$i.local stop $CONTAINER_NAME
done

docker -H tticlargeloop.local stop $CONTAINER_NAME
docker -H $DUCKIEBOT_NAME.local stop $CONTAINER_NAME