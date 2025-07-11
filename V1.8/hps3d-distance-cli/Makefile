CC = gcc
CFLAGS = -Wall -Wextra -I./src -I./lib
LDFLAGS = -L./lib -lHPS3D
SRC = src/main.c
OBJ = $(SRC:.c=.o)
TARGET = hps3d-distance-cli

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CC) -o $@ $^

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ) $(TARGET)