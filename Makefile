CC=g++
CFLAGS=-pedantic-errors -Wall -Weffc++ -Wextra -Wsign-conversion
DEPS=config.o rrt.o collision.o graphics.o

target: $(DEPS)
	$(CC) $(DEPS) -lsfml-graphics -lsfml-window -lsfml-system

%.o: %.cpp %.h
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm *.o a.out

