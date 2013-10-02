# Call with Q='' to see full commands
Q ?= @

OPTFLAGS = -O2
WARNFLAGS ?= -Wall -Wextra -std=c99 -pedantic-errors \
	-Wshadow -Wswitch-default -Wswitch-enum -Wundef \
	-Wuninitialized -Wpointer-arith -Wstrict-prototypes \
	-Wmissing-prototypes -Wcast-align -Wformat=2 \
	-Wimplicit-function-declaration -Wredundant-decls \
	-Wformat-security -Wcast-qual -Wconversion \

LDSECFLAGS ?= -pie -Wl,-z,relro -Wl,-z,now
LDFLAGS = $(LINKS) $(LDSECFLAGS) $(LDDBGFLAGS)

SECFLAGS ?= -fPIE -fPIC -D_FORTIFY_SOURCE=2 -fstack-protector
CFLAGS = $(WARNFLAGS) $(DBGFLAGS) $(OPTFLAGS) $(SECFLAGS) $(INCLUDES) $(DEFINES)
CC ?= gcc

CCSTR   = "\e[01;37mCC    \e[00m"
CCLDSTR = "\e[01;37mCCLD  \e[00m"

LINKS = -lm

C_SRC := quaternion.c linalg.c comp_filter.c zimu.c
C_OBJ := $(C_SRC:%.c=%.o)

zimu: $(C_OBJ)
	@echo -e $(CCLDSTR) $@
	$(Q)$(CC) $(CFLAGS) $^ $(LDFLAGS) -o $@

%.o: %.c
	@echo -e $(CCSTR) $<
	$(Q)$(CC) $(CFLAGS) -c $< -o $@
