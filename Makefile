# Makefile for libOnStep

CC = gcc
CFLAGS = -fPIC -Wall -Wextra -O2 -g -DSB_LINUX_BUILD -I. -I./../../
CPPFLAGS = -fPIC -Wall -Wextra -O2 -g -DSB_LINUX_BUILD -std=gnu++11 -I. -I./../../
LDFLAGS = -shared -lstdc++
RM = rm -f
STRIP = strip
TARGET_LIB = libOnStep.so

SRCS = main.cpp OnStep.cpp ZWOMount.cpp x2mount.cpp
OBJS = $(SRCS:.cpp=.o)

.PHONY: all validate_ui
all: validate_ui ${TARGET_LIB}

validate_ui:
	@if command -v uic >/dev/null 2>&1; then \
		echo "Validating OnStep.ui with uic..."; \
		uic OnStep.ui > /dev/null || (echo "UI Validation failed!" && exit 1); \
	else \
		echo "uic not found, skipping UI validation."; \
	fi

$(TARGET_LIB): $(OBJS)
	$(CC) ${LDFLAGS} -o $@ $^
	$(STRIP) $@ >/dev/null 2>&1  || true

$(SRCS:.cpp=.d):%.d:%.cpp
	$(CC) $(CFLAGS) $(CPPFLAGS) -MM $< >$@

.PHONY: clean
clean:
	${RM} ${TARGET_LIB} ${OBJS}
