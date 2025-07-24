#!/bin/bash

plinkPath="/mnt/c/plink/plink.exe"
"$plinkPath" -serial COM3 -sercfg 9600,8,n,1,N
