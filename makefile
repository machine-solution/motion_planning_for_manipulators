FLAGS = -O2 -I include # -mavx -pthread -Wl,-rpath,'$$ORIGIN'
LIBS = -lmujoco -lglfw
CXX = g++
OBJ = obj
SRC = src
NAME = manipulator

.PHONY: all clean

all: $(NAME)

clean:
	rm -rf $(OBJ)
	rm -f $(NAME)

$(NAME): $(OBJ)/planner.o $(OBJ)/main.o
	$(CXX) $(OBJ)/planner.o $(OBJ)/main.o $(LIBS) -o $(NAME)

# compile commands
$(OBJ)/planner.o: $(SRC)/planner.cpp include/planner.h
	mkdir -p $(OBJ)
	$(CXX) $(FLAGS) $(SRC)/planner.cpp $(LIBS) -c -o $(OBJ)/planner.o

$(OBJ)/main.o: $(SRC)/main.cpp include/planner.h
	mkdir -p $(OBJ)
	$(CXX) $(FLAGS) $(SRC)/main.cpp $(LIBS) -c -o $(OBJ)/main.o
