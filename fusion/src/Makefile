# Usage:
# make			# compile all binary
# make clean	# remove ALL binaries and objects

.PHONY = all clean

CC = g++

LINKERFLAG = -std=c++11

SRCS := $(wildcard *.cpp)
# BINS := $(SRCS:%.cpp=%)
OUT := Fusion

all: compile #${BINS}

# %: %.o
# 		@echo "Checking.."
# 		@echo ${SRCS}
# 		${CC} ${LINKERFLAG} $< -o $@

# %.o: %.cpp
# 		@echo "Creating object.."
# 		${CC} ${LINKERFLAG} -c $<

compile: ${SRCS}
		@echo "Compiling cpp files..."
		${CC} ${LINKERFLAG} ${SRCS} -o ${OUT}

clean:
		@echo "Cleaning up..."
		rm -rvf *.o ${BINS}
