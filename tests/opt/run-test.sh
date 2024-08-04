#!/usr/bin/env bash
cd "$(dirname "$0")"
set -eu
source ../gen-tests-makefile.sh

if [ $# -ge 1 ]
then
    glob=$1
else
	glob='*.ys'
fi

run_tests --yosys-scripts --yosys-glob "$glob"
