#!/bin/bash
if [ ! -d "./build" ];then
mkdir -p ./build/Debug/obj
mkdir -p ./build/Release/obj
fi

if [ "$1" = "PC" ];
then
    make TG=PC
else
    make TG=PI
fi
