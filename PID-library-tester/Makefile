PROJECT=D:/WorkSpace/PID-lib/PID/PID-library-tester/pid-tester
PID_LIB=D:/WorkSpace/PID-lib/PID/PID-Library/PID_V2
INC=$(PID_LIB)
SRC=$(PID_LIB)/PID_V2.c
all: 
	gcc -g -Wall $(PROJECT).c $(SRC) -o $(PROJECT).exe -I $(INC)

clean:
	rm $(PROJECT).exe

debug:
	gdb $(PROJECT).exe