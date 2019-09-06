#!/bin/bash
if [ ! -d "./rtk_build" ];then
mkdir -p ./rtk_build/Debug/obj
mkdir -p ./rtk_build/Release/obj
fi

if [ "$1" = "PC" ];
then
    make TG=PC
else
    make TG=PI
fi
