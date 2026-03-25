# Makefile for libOnStep (Linux only)
# macOS: use Xcode — xcodebuild clean && xcodebuild

CXX    = g++
RM     = rm -f
STRIP  = strip

TARGET_LIB = libOnStep.so
OS_FLAG    = -DSB_LINUX_BUILD
LDFLAGS    = -shared -lstdc++

# The SDK headers use relative includes of the form "../../licensedinterfaces/foo.h".
# Adding -I../X2-Examples/licensedinterfaces makes that path resolve as:
#   ../X2-Examples/licensedinterfaces/../../licensedinterfaces/ = appinstall/licensedinterfaces/
SDK_LI   = ../X2-Examples/licensedinterfaces

CPPFLAGS = -fPIC -Wall -Wextra -O2 -g $(OS_FLAG) -std=gnu++11 -I. -I$(SDK_LI)

SRCS = main.cpp OnStep.cpp ZWOMount.cpp x2mount.cpp
OBJS = $(SRCS:.cpp=.o)

.PHONY: all clean validate_ui

all: validate_ui $(TARGET_LIB)

validate_ui:
	@if command -v uic >/dev/null 2>&1; then \
		echo "Validating OnStep.ui..."; \
		uic OnStep.ui > /dev/null || (echo "UI validation failed!" && exit 1); \
		echo "UI OK."; \
	else \
		echo "uic not found, skipping UI validation."; \
	fi

$(TARGET_LIB): $(OBJS)
	$(CXX) $(LDFLAGS) -o $@ $^
	$(STRIP) $@ >/dev/null 2>&1 || true

%.o: %.cpp
	$(CXX) $(CPPFLAGS) -c $< -o $@

clean:
	$(RM) $(TARGET_LIB) $(OBJS)
