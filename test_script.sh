#!/bin/bash

for i in {02..07}; do
  dts devel run -H watchtower$i --pull --detach
done
