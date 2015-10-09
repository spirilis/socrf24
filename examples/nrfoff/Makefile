# Makefile for socrf24 examples

GXX=g++
CFLAGS=-I../.. -I. -DDEBUG=1 `pkg-config libsoc --cflags`
LDFLAGS=`pkg-config libsoc --libs`

PROG=nrfoff
SRC=$(PROG).cpp ../../socrf24_interrupt.cpp

all: $(PROG)

$(PROG): $(SRC)
	$(GXX) $(CFLAGS) -o $(PROG) $(SRC) $(LDFLAGS)

clean:
	rm -f $(PROG)
