CC = gcc
CFLAGS = -I./src
LDFLAGS = -L./lib -L./lib/lHPS3DSDK_aarch64_1-8-6
SRC = src/main.c
OUT = hps3d-distance-cli

all:
	$(CC) $(CFLAGS) $(SRC) -o $(OUT) $(LDFLAGS)