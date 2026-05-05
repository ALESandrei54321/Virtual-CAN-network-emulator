# bus_broker/transport/Makefile

CC      = gcc
CFLAGS  = -O2 -Wall -Wextra -fPIC -std=c11
TARGET  = libringbuffer.so

all: $(TARGET)

$(TARGET): ring_buffer.c ring_buffer.h
	$(CC) $(CFLAGS) -shared -o $(TARGET) ring_buffer.c -lrt

clean:
	rm -f $(TARGET) *.o

.PHONY: all clean
