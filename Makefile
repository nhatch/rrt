CC=g++
ROS_INCLUDE=-I /opt/ros/melodic/include
CFLAGS=-pedantic-errors $(ROS_INCLUDE) #-Wall -Wextra #-Wsign-conversion #-Weffc++
DEPS=config.o rrt.o collision.o graphics.o control.o mppi/kinematic_mppi.o mppi/stick_mppi.o arrayio.o graph.o
ROS_LINK=-L /opt/ros/melodic/lib -lboost_system
SFML_LINK=-lsfml-graphics -lsfml-window -lsfml-system

target: $(DEPS)
	$(CC) $(DEPS) $(ROS_LINK) $(SFML_LINK)

rrt.o: rrt.cpp *.h mppi/*.h
	$(CC) $(CFLAGS) -c -o $@ $<

control.o: control.cpp *.h mppi/*.h
	$(CC) $(CFLAGS) -c -o $@ $<

%.o: %.cpp %.h *.h mppi/*.h
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm mppi/*.o *.o a.out

