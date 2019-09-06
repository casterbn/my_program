#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := rtk_esp_bt_app


#COMPONENT_ADD_INCLUDEDIRS += ./inc/bsp ./inc/common

#COMPONENT_SRCDIRS += src/bsp 


EXTRA_COMPONENT_DIRS += aceinna_app
include $(IDF_PATH)/make/project.mk 

COMPONENT_LDFLAGS+= -D ABS_PATH=$(abspath $(dir $(firstword $(MAKEFILE_LIST))))	