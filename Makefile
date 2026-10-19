CC      = gcc
CFLAGS  = -O2 -Wall -Wextra -std=gnu11
LDLIBS  = -lSDL2 -lGL -lGLU -lm

OBJ = main.o physics.o visual.o

skater: $(OBJ)
	$(CC) $(CFLAGS) -o $@ $(OBJ) $(LDLIBS)

main.o: main.c physics.h visual.h
physics.o: physics.c physics.h
visual.o: visual.c visual.h physics.h

run: skater
	./skater 4 20

single: figure_skater.c
	$(CC) $(CFLAGS) -o figure_skater figure_skater.c $(LDLIBS)

clean:
	rm -f $(OBJ) skater figure_skater

.PHONY: run test single clean
