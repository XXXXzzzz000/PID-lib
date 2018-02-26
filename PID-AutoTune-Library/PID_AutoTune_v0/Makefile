
PROJECT=D:\WorkSpace\PID-lib\PID\PID-AutoTune-Library\PID_AutoTune_v0\AutoTune_Example

PID_LIB=D:/WorkSpace/PID-lib/PID/PID-Library/PID_V2
AUTO_TUNE_LIB=D:\WorkSpace\PID-lib\PID\PID-AutoTune-Library\PID_AutoTune_v0

INC=$(PID_LIB) $(AUTO_TUNE_LIB)
SRC=$(PID_LIB)/PID_V2.c $(AUTO_TUNE_LIB)/PID_AutoTune_v0.c
all: 
	gcc -g -Wall $(PROJECT).c $(SRC) -o $(PROJECT).exe -I $(INC)

clean:
	rm $(PROJECT).exe

debug:
	gdb $(PROJECT).exe