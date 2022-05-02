PROJNAME := AC_Test
CC := g++
SRCDIR := src
INCDIR := src
BINDIR := bin
SRC_C := $(wildcard $(SRCDIR)/*.c)
NOD_C := $(notdir $(SRC_C)) #SRC w/o dir.
OBJ_C := $(patsubst %.c,$(BINDIR)/%_C.o,$(NOD_C))
SRC_CPP := $(wildcard $(SRCDIR)/*.cpp)
NOD_CPP := $(notdir $(SRC_CPP)) #SRC w/o dir.
OBJ_CPP := $(patsubst %.cpp,$(BINDIR)/%_CPP.o,$(NOD_CPP))
INC := -I $(INCDIR)
CFLAGS := -g -Wa,--warn -fdata-sections -ffunction-sections -Os
PTRPRO := -D __NONE__
LINKFLAGS := -Wl,-gc-sections
MATHFLAGS := -lm

prt:
	@echo $(OBJ_C) $(OBJ_CPP)

all: $(OBJ_C) $(OBJ_CPP)
	$(CC) $(PREPRO) -o $(BINDIR)/$(PROJNAME) $(OBJ_C) $(OBJ_CPP) $(LINKFLAGS) $(MATHFLAGS)

$(BINDIR)/%_C.o: $(SRCDIR)/%.c
	$(CC) $(PREPRO) $(CFLAGS) $(INC) -c $< -o $@ $(MATHFLAGS)

$(BINDIR)/%_CPP.o: $(SRCDIR)/%.cpp
	$(CC) $(PREPRO) $(CFLAGS) $(INC) -c $< -o $@ $(MATHFLAGS)

clean:
	rm $(BINDIR)/*
