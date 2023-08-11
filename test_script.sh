#!/bin/bash

for i in {01..07}; do
  dts devel run -H watchtower$i --pull
done
