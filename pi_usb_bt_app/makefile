Target = ./build/Debug/pi_bt_app
INC += -I./rtk_app 
INC += -I./usr/include/libusb-1.0 
TOPDIR = ..
APPDIR = ./rtk_app

LIBS = -lpthread -lbluetooth -lusb-1.0 
EXCHAR	= -fexec-charset=GB18030    

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
#CFLAGS += -w 
else
CFLAGS += -W
CFLAGS += -o1
endif

OUTDIR = build/$(CFG)/obj
COMPILE = @$(CC) $(EXCHAR) $(LIBS)  $(INC)  $(CFLAGS)  -c $<  $(EXCHAR) -o  $@#"$(OUTDIR)/$(*F).o"

COMMON_OBJ=$(OUTDIR)/main.o \
	$(OUTDIR)/bt.o  \
	$(OUTDIR)/com.o
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
        

LINK =	@$(CC) $(OBJ) $(LIBS) $(INC)  -o $(Target)



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

