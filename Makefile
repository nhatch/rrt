CC=g++
CFLAGS=-pedantic-errors -Wall -Weffc++ -Wextra -Wsign-conversion
DEPS=config.o rrt.o collision.o graphics.o control.o
ROS_INCLUDE=-I /opt/ros/melodic/include
ROS_LINK=-lboost_system
SFML_LINK=-lsfml-graphics -lsfml-window -lsfml-system

target: $(DEPS)
	$(CC) $(DEPS) $(ROS_LINK) $(SFML_LINK)

rrt.o: rrt.cpp rrt.h graphics.h
	$(CC) $(CFLAGS) -c -o $@ $<

control.o: control.cpp control.h
	$(CC) $(CFLAGS) $(ROS_INCLUDE) -c -o $@ $<

%.o: %.cpp %.h
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm *.o a.out

