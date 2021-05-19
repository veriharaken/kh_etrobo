#!/usr/bin/bash
# 1st arg is number pf column
xxd -c $1 log.dat > log.dat.txt
