CC = gcc
CFLAGS = -I./src
LDFLAGS = -L./lib -lHPS3D
SRC = src/main.c
OUT = hps3d-distance-cli

all: $(OUT)

$(OUT): $(SRC)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

clean:
	rm -f $(OUT) *.o src/*.o