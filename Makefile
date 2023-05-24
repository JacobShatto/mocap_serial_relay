# Directory and file variables
SRCDIR		:= src
BINDIR		:= bin
BUILDDIR	:= build
INCLUDEDIR	:= include
INCLUDESUBD	:= $(shell find $(INCLUDEDIR)/* -type d)

AUX_DIR 	:= auxiliary
COMM_DIR	:= common
OPTI_DIR 	:= optitrack
QUALI_DIR	:= qualisys
MAIN_DIR 	:= main_files

GEN_SRC_DIRS	:= $(SRCDIR)/$(AUX_DIR) $(SRCDIR)/$(COMM_DIR)

# Target executables
TARGET_OPTI  	:= $(BINDIR)/opti_serial_relay
TARGET_QUALI	:= $(BINDIR)/quali_serial_relay

# File definitions for rules
#	Sources - *.c files including main.c
#	Objects	- c objects created from the main source files
#	Includes - all include files, dependency of all objects
GEN_SOURCE_C	:= $(shell find $(GEN_SRC_DIRS) -type f -name *.c)
GEN_SOURCE_CPP	:= $(shell find $(GEN_SRC_DIRS) -type f -name *.cpp)
MAIN_SOURCE 	:= $(shell find $(SRCDIR)/$(MAIN_DIR) -type f -name *.cpp)
OPTI_SOURCE 	:= $(shell find $(SRCDIR)/$(OPTI_DIR) -type f -name *.cpp)
QUALI_SOURCE 	:= $(shell find $(SRCDIR)/$(QUALI_DIR) -type f -name *.cpp)

GEN_OBJS		:= $(GEN_SOURCE_C:$(SRCDIR)/%.c=$(BUILDDIR)/%.o) \
					$(GEN_SOURCE_CPP:$(SRCDIR)/%.cpp=$(BUILDDIR)/%.o)

MAIN_OBJS		:= $(MAIN_SOURCE:$(SRCDIR)/%.cpp=$(BUILDDIR)/%.o)

OPTI_OBJS		:= $(OPTI_SOURCE:$(SRCDIR)/%.cpp=$(BUILDDIR)/%.o)
QUALI_OBJS		:= $(QUALI_SOURCE:$(SRCDIR)/%.cpp=$(BUILDDIR)/%.o)


INCLUDES		:= $(shell find $(INCLUDEDIR) -name '*.h*')

# Compilers, linkers and options
CC			:= gcc
CXX			:= g++
CLINKER		:= gcc
CXXLINKER	:= g++
WFLAGS		:= -Wall -Wextra 
INCLUDEFLAG	:= $(INCLUDESUBD:%=-I%)
# INCLUDEFLAG	:= $(addprefix -I, $(INCLUDEDIR), $(INCLUDESUBD))
CFLAGS		:= $(INCLUDEFLAG)
CXXFLAGS	:= $(INCLUDEFLAG)
OPT_FLAGS	:= -O3
LDFLAGS		:= -lm -lrt -pthread -lrobotcontrol -ljson-c
MAIN_FLAGS	:= $(CXXFLAGS) $(OPT_FLAGS) $(WFLAGS)

#Rule for all BINARIES (executables)	
all: opti quali r_serial s_serial recv_serial_opti quali_5q r_serial_5q opti_5q
	@echo "All binaries made successfully"

#Rule for BINARIES osr_5q_900mhz 
opti_5q: $(OPTI_OBJS) $(BUILDDIR)/$(MAIN_DIR)/osr_5q_900mhz.o
	@mkdir -p $(BINDIR)

	@echo "Making BINARY osr_5q_900mhz"
	@$(CXX) $(BUILDDIR)/$(MAIN_DIR)/osr_5q_900mhz.o 	$(GEN_OBJS) $(OPTI_OBJS) $(MAIN_FLAGS) -o $(BINDIR)/osr_5q_900mhz

#Rule for BINARIES opti_serial_relay 
opti: $(OPTI_OBJS) $(BUILDDIR)/$(MAIN_DIR)/opti_serial_relay.o
	@mkdir -p $(BINDIR)

	@echo "Making BINARY opti_serial_relay"
	@$(CXX) $(BUILDDIR)/$(MAIN_DIR)/opti_serial_relay.o 	$(GEN_OBJS) $(OPTI_OBJS) $(MAIN_FLAGS) -o $(BINDIR)/opti_serial_relay

#Rule for BINARY qsr_5q_900mhz
quali_5q: $(QUALI_OBJS) $(BUILDDIR)/$(MAIN_DIR)/qsr_5q_900mhz.o
	@mkdir -p $(BINDIR)

	@echo "Making BINARY qsr_5q_900mhz"
	@$(CXX) $(BUILDDIR)/$(MAIN_DIR)/qsr_5q_900mhz.o 	$(GEN_OBJS) $(QUALI_OBJS) $(MAIN_FLAGS) -o $(BINDIR)/qsr_5q_900mhz 

#Rule for BINARY quali_serial_relay
quali: $(QUALI_OBJS) $(BUILDDIR)/$(MAIN_DIR)/quali_serial_relay.o
	@mkdir -p $(BINDIR)

	@echo "Making BINARY quali_serial_relay"
	@$(CXX) $(BUILDDIR)/$(MAIN_DIR)/quali_serial_relay.o 	$(GEN_OBJS) $(QUALI_OBJS) $(MAIN_FLAGS) -o $(BINDIR)/quali_serial_relay 

#Rule for BINARY receive_serial
r_serial_5q: $(BUILDDIR)/$(MAIN_DIR)/receive_serial_5q.o
	@mkdir -p $(BINDIR)

	@echo "Making BINARY receive_serial_5q"
	@$(CXX) $(BUILDDIR)/$(MAIN_DIR)/receive_serial_5q.o 		$(GEN_OBJS) $(MAIN_FLAGS) -o $(BINDIR)/receive_serial_5q 

#Rule for BINARY receive_serial
r_serial: $(BUILDDIR)/$(MAIN_DIR)/receive_serial.o
	@mkdir -p $(BINDIR)

	@echo "Making BINARY receive_serial"
	@$(CXX) $(BUILDDIR)/$(MAIN_DIR)/receive_serial.o 		$(GEN_OBJS) $(MAIN_FLAGS) -o $(BINDIR)/receive_serial 

#Rule for BINARY send_serial
s_serial: $(BUILDDIR)/$(MAIN_DIR)/send_serial.o
	@mkdir -p $(BINDIR)

	@echo "Making BINARY send_serial"
	@$(CXX) $(BUILDDIR)/$(MAIN_DIR)/send_serial.o 		$(GEN_OBJS) $(MAIN_FLAGS) -o $(BINDIR)/send_serial 

#Rule for BINARY receive_serial
recv_serial_opti: $(GEN_OBJS) $(BUILDDIR)/$(MAIN_DIR)/receive_serial_opti.o
	@mkdir -p $(BINDIR)

	@echo "Making BINARY receive_serial_opti"
	@$(CXX) $(BUILDDIR)/$(MAIN_DIR)/receive_serial_opti.o 	$(GEN_OBJS) $(MAIN_FLAGS) -o $(BINDIR)/receive_serial_opti 

#Rule for all AUXILIARY objects
$(BUILDDIR)/$(AUX_DIR)/%.o : $(SRCDIR)/$(AUX_DIR)/%.c 
	@echo Making AUXILIARY object $@
	@mkdir -p $(BUILDDIR)/$(AUX_DIR)
	@$(CC) -c $(CFLAGS) $(OPT_FLAGS) $(WFLAGS) $< -o $(@) 

#Rule for all COMMON objects
$(BUILDDIR)/$(COMM_DIR)/%.o : $(SRCDIR)/$(COMM_DIR)/%.cpp 
	@echo Making COMMON object $@
	@mkdir -p $(BUILDDIR)/$(COMM_DIR)
	@$(CXX) -c $(CXXFLAGS) $(OPT_FLAGS) $(WFLAGS) $< -o $(@)

$(BUILDDIR)/$(COMM_DIR)/%.o : $(SRCDIR)/$(COMM_DIR)/%.c
	@echo Making COMMON object $@
	@mkdir -p $(BUILDDIR)/$(COMM_DIR)
	@$(CC) -c $(CFLAGS) $(OPT_FLAGS) $(WFLAGS) $< -o $(@)

#Rule for all OPTITRACK objects
$(BUILDDIR)/$(OPTI_DIR)/%.o : $(SRCDIR)/$(OPTI_DIR)/%.cpp 
	@echo Making OPTITRACK object $@
	@mkdir -p $(BUILDDIR)/$(OPTI_DIR)
	@$(CXX) -c $(CXXFLAGS) $(OPT_FLAGS) $(WFLAGS) $< -o $(@)

#Rule for all QUALISYS objects
$(BUILDDIR)/$(QUALI_DIR)/%.o : $(SRCDIR)/$(QUALI_DIR)/%.cpp 
	@echo Making QUALISYS object $@
	@mkdir -p $(BUILDDIR)/$(QUALI_DIR)
	@$(CXX) -c $(CXXFLAGS) $(OPT_FLAGS) $(WFLAGS) $< -o $(@)

#Rule for all MAIN objects
$(BUILDDIR)/$(MAIN_DIR)/%.o: $(SRCDIR)/$(MAIN_DIR)/%.cpp $(GEN_OBJS)
	@echo Making MAIN object $@
	@mkdir -p $(BUILDDIR)/$(MAIN_DIR)
	@$(CXX) -c $(CXXFLAGS) $(OPT_FLAGS) $(WFLAGS) $< -o $(@)

clean:
	@rm -rvf $(BINDIR)
	@rm -rvf $(BUILDDIR)
	@touch * $(SRCDIR)/* $(INCLUDEDIR)/*
	@echo "Library Clean Complete"