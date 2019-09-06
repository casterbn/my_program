Target = ./libhello.a
INC += -I./
APPDIR=./
AR = arm-none-eabi-ar
ARFLAGS = rcs
LIBS = -lpthread -lm  -lc -ld	#需要添加math库


ifndef TG
	TG=ARM
	#TG=PC
endif

ifeq "$(TG)" "PC"
CC = arm-none-eabi-gcc -std=c99
CPP = arm-none-eabi-gcc -std=c99
else
CC = arm-none-eabi-gcc -std=c99
CPP = arm-none-eabi-gcc  -std=c99
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

CFLAGS += -mcpu=cortex-m4
#CFLAGS += -mcpu=cortex-m4 -mthumb -Wall 
#CFLAGS += -mfloat-abi=softfp -mfpu=fpv4-sp-d16
#CFLAGS += -Os
#CFLAGS += -ffunction-sections -fdata-sections

else
CFLAGS += -W
CFLAGS += -o1
endif

OUTDIR = ./build
COMPILE = @$(CC)  $(LIBS)  $(INC)  $(CFLAGS) -c $<   -o  $@ #"$(OUTDIR)/$(*F).o"

COMMON_OBJ = $(OUTDIR)/hello.o  
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
        

LINK =	@$(AR) $(ARFLAGS) -o $(Target) $(OBJ) 



# Build rules
$(Target): $(OUTDIR)  $(OBJ)
	$(LINK)
#	cp $(Target) ./
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

