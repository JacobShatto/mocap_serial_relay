#  This compiles the qualisys_driver only
#  Compile the receiver code from the optitrack folder
SRC_PATH 	:= $(shell pwd)/../.
BIN_PATH	:= $(SRC_PATH)/bin/.
COMMON_LIB_PATH := $(SRC_PATH)/qualisys/common/lib/.

LDFLAGS_STD	:= -lm -lpthread
LDFLAGS_COMMON	:= -L -lcommon $(COMMON_LIB_PATH)/libcommon.a

LD = g++
LDFLAGS = -std=c++0x -g -fpermissive -I $(SRC_PATH) \
	$(LDFLAGS_STD) \
	$(LDFLAGS_COMMON)

QUALI_SRCS = RTProtocol.cpp Network.cpp RTPacket.cpp NBC_Markup.cpp
QUALI_SRCS := $(addprefix $(SRC_PATH)/qualisys/qualisys/, $(QUALI_SRCS))
QUALISYS_DRIVER_SRCS := $(SRC_PATH)/qualisys/qualisys_driver.cpp \
	$(QUALI_SRCS) 

all: qualisys_driver

qualisys_driver:
	$(LD) $(QUALISYS_DRIVER_SRCS) $(LDFLAGS) -o $(BIN_PATH)/qualisys_driver

clean:
	-rm -f *.o $(BIN_PATH)/qualisys_driver

