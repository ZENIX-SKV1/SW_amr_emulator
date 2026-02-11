#!/bin/bash

export LD_LIBRARY_PATH="$(pwd)/lib:$LD_LIBRARY_PATH"
cd ./bin
./amr_emulator


