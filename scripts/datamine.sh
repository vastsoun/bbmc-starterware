#!/bin/bash

mkdir data;

cat $1 | tr -d $'\r' | grep -v '^$' | awk '/^DATALOG_END/ { inBlock=0 } inBlock { print > outfile } /^DATALOG_START:/ { inBlock=1; outfile = "data/outfile" ++count ".csv"}'
