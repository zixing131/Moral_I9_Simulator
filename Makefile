CC := gcc

OBJS := 

UNICORN = Lib/unicorn-2.1.4/unicorn-import.lib

SDL2 = Lib/sdl2-2.0.10

# -Wl,-subsystem,windows gets rid of the console window
# gcc  -o main.exe main.c -lmingw32 -Wl,-subsystem,windows -L./lib -lSDL2main -lSDL2
# -mwindows 关闭控制台窗口
# -lwinhttp http通信库
build:
	$(CC) src/main.c -o bin/main.exe -g -lpthread -lm -lmingw32 -lkernel32 -lwinmm -Wall -lws2_32 -DNETWORK_SUPPORT $(UNICORN) -L$(SDL2)/lib/ -lSDL2main -lSDL2
