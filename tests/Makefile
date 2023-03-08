CXX = g++
OBJ = obj
FLAGS = -I ../include
TARGET = tests

.PHONY: all clean

all: $(TARGET)

clean:
	rm -f $(TARGET)
	rm -rf $(OBJ)


$(TARGET): $(OBJ)/catch_amalgamated.o $(OBJ)/main.o
	$(CXX) $(OBJ)/catch_amalgamated.o $(OBJ)/main.o -o $(TARGET)

# compile commands
$(OBJ)/catch_amalgamated.o: catch2/catch_amalgamated.cpp ../include/catch2/catch_amalgamated.hpp
	mkdir -p $(OBJ)
	$(CXX) $(FLAGS) catch2/catch_amalgamated.cpp -c -o $(OBJ)/catch_amalgamated.o

$(OBJ)/main.o: main.cpp ../include/catch2/catch_amalgamated.hpp
	mkdir -p $(OBJ)
	$(CXX) $(FLAGS) main.cpp -c -o $(OBJ)/main.o
