Target = ./rtk_build/Debug/dch_test
INC += -I./wiringPi
TOPDIR=..
APPDIR=rtk_app

LIBS = -lpthread -L./wiringPi_lib/  -lwiringPi   -L./wiringPi_lib/ -lwiringPiDev   


ifndef TG
	TG=PI
	#TG=PC
endif

ifeq "$(TG)" "PC"
CC = arm-linux-gnueabihf-gcc -std=c99
CPP = arm-linux-gnueabihf-gcc -std=c99
else
CC = gcc -std=c99
CPP = arm-linux-gnueabihf-gcc -std=c99
endif


ifndef CFG
	CFG=Debug
	#CFG=Release
endif

ifndef TEST
	TEST=ENABLE
	#CFG=Release
endif

ifeq "$(CFG)" "Debug"
#CFLAGS += -w -o2
CFLAGS +=  -MMD  -w
else
CFLAGS += -W
CFLAGS += -o1
endif

OUTDIR = rtk_build/$(CFG)/obj
COMPILE = @$(CC)  $(LIBS)  $(INC)  $(CFLAGS) -c $<   -o  $@#"$(OUTDIR)/$(*F).o"

COMMON_OBJ=$(OUTDIR)/main.o \
	$(OUTDIR)/RingBuffer.o  \
	$(OUTDIR)/rtk.o \
#$(Target) : $(SRCS)
#	gcc  -I $(INC) -o2 $(Target)   $(SRCS)  $(LIBS) -g

OBJ=$(COMMON_OBJ)

CDEF = $(OBJ:.o=.d)

all: $(Target)

$(OBJ):
# Pattern rules



$(OUTDIR)/%.o : $(APPDIR)/%.c
	@echo ""
	@echo "->>>>>>>> compile " $@
	$(COMPILE)
        

LINK =	@$(CC)  $(OBJ) $(LIBS) $(INC) -o $(Target)



# Build rules
$(Target): $(OUTDIR)  $(OBJ)
	$(LINK)
	cp $(Target) ./
	@echo "->>>>>>>> gen " $(Target)
ifeq "$(CFG)" "Debug"
	@echo "Debug" 
else
	@echo "Release" 
endif
	@echo "+++++++++++++++++++++++++"
	@echo "+++++++++++++++++++++++++"
clean:            
	rm $(Target) $(COMMON_OBJ) $(CDEF)

