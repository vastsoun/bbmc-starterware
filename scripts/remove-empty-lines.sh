#!/bin/bash

cat $1 | tr -d $'\r' | grep -v '^$' >> killog
