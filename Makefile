# Makefile for libOnStep

CXX    = g++
RM     = rm -f
STRIP  = strip

UNAME_S := $(shell uname -s)

ifeq ($(UNAME_S),Darwin)
  TARGET_LIB = libOnStep.dylib
  OS_FLAG    = -DSB_MACOSX_BUILD
  # Universal binary: x86_64 for Rosetta/older installs, arm64 for native Apple Silicon
  ARCH_FLAGS = -arch x86_64 -arch arm64
  LDFLAGS    = -dynamiclib -lstdc++ $(ARCH_FLAGS)
  # Support both Intel (/usr/local) and Apple Silicon (/opt/homebrew) brew prefixes
  UIC        ?= $(firstword $(wildcard \
                  /usr/local/opt/qt@5/bin/uic \
                  /opt/homebrew/opt/qt@5/bin/uic))
else
  TARGET_LIB = libOnStep.so
  OS_FLAG    = -DSB_LINUX_BUILD
  LDFLAGS    = -shared -lstdc++
  UIC        ?= $(shell command -v uic 2>/dev/null)
endif

# The SDK headers use relative includes of the form "../../licensedinterfaces/foo.h".
# Adding -I../X2-Examples/licensedinterfaces makes that path resolve as:
#   ../X2-Examples/licensedinterfaces/../../licensedinterfaces/ = appinstall/licensedinterfaces/
SDK_LI   = ../X2-Examples/licensedinterfaces

CPPFLAGS = -fPIC -Wall -Wextra -O2 -g $(OS_FLAG) -std=gnu++11 -I. -I$(SDK_LI) $(ARCH_FLAGS)

SRCS = main.cpp OnStep.cpp ZWOMount.cpp x2mount.cpp
OBJS = $(SRCS:.cpp=.o)

.PHONY: all clean validate_ui install

all: validate_ui $(TARGET_LIB)

validate_ui:
	@if [ -n "$(UIC)" ] && [ -x "$(UIC)" ]; then \
		echo "Validating OnStep.ui..."; \
		"$(UIC)" OnStep.ui > /dev/null || (echo "UI validation failed!" && exit 1); \
		echo "UI OK."; \
	else \
		echo "uic not found, skipping UI validation."; \
	fi

$(TARGET_LIB): $(OBJS)
	$(CXX) $(LDFLAGS) -o $@ $^
	$(STRIP) $@ >/dev/null 2>&1 || true

%.o: %.cpp
	$(CXX) $(CPPFLAGS) -c $< -o $@

install: $(TARGET_LIB)
	./installer/install.sh

clean:
	$(RM) libOnStep.so libOnStep.dylib $(OBJS)
