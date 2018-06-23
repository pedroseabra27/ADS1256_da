PROJ_NAME=ads1256_da

LINK=$(CXX)
#LINK=$(CC)

uname_m := $(shell uname -m)

SRCS = ads1256_da.cpp

ifeq ($(uname_m),armv7l)
  LIBS= -lbcm2835
else
  SRCS += bcm_fake.c
endif
#

###################################################

CFLAGS_ADD :=

DEPSDIR=.deps
OBJDIR=.objs


ALLFLAGS := -g2 -O2 -Wall -grecord-gcc-switches
ALLFLAGS += -pthread
ALLFLAGS += -march=native

ALLFLAGS += $(CFLAGS_ADD)

ALLFLAGS += -I.

SRCPATHS =  .

vpath %.c   $(SRCPATHS)
vpath %.cpp $(SRCPATHS)
vpath %.s $(STMSRC)
vpath %.o $(OBJDIR)
vpath %.d $(DEPSDIR)


OBJS0a = $(SRCS:.cpp=.o)
OBJS0 = $(OBJS0a:.c=.o)
OBJS  = $(OBJS0:.s=.o)
OBJS1 = $(addprefix $(OBJDIR)/,$(OBJS))

CFLAGS   = $(ALLFLAGS)  -std=c11
CXXFLAGS = $(ALLFLAGS)  -std=c++17 -fgnu-keywords

###################################################

.PHONY: proj

all: proj dirs

dirs:
	mkdir -p $(DEPSDIR) $(OBJDIR)

proj:  dirs $(PROJ_NAME)

$(OBJDIR)/*.o:  Makefile


$(OBJDIR)/%.o: %.c
	$(CC) $(CFLAGS) -c -MMD -o $@ $<
	mv $(OBJDIR)/$*.d $(DEPSDIR)

$(OBJDIR)/%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -MMD -o $@ $<
	mv $(OBJDIR)/$*.d $(DEPSDIR)

$(OBJDIR)/%.o: %.s
	$(CC) $(CFLAGS) -c -o $@ $<

$(PROJ_NAME): $(OBJS1)
	$(LINK) $(CFLAGS) $(LDFLAGS) $^ $(LIBS)  -o $@



clean:
	rm -f *.o *.d $(OBJDIR)/*.o $(DEPSDIR)/*.d

include $(wildcard $(DEPSDIR)/*.d)
#

