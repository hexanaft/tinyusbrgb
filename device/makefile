#=======================================

# Название файла проекта (без расширения)
TARGET = t45_led_rgb_soft_pwm_usb

# Тип используемого микроконтроллера
MCU  = attiny85

# Частота кварцевого резонатора (Hz) # в Герцах
F_CPU   = 16500000

# Используемый программатор (для AVRDUDE)
MY_AVRDUDE_PROGRAMMER = usbasp
#MY_AVRDUDE_PROGRAMMER = usbtiny

# Порт, к которому подключен программатор
MY_PROGRAMMER_PORT = usb

#=======================================

USB_DRV_DIR = usbdrv

CFLAGSMY  = -I$(USB_DRV_DIR) -I. -DDEBUG_LEVEL=0

OBJECTS = $(USB_DRV_DIR)/usbdrv.o $(USB_DRV_DIR)/usbdrvasm.o $(USB_DRV_DIR)/oddebug.o $(TARGET).o


#============================================================================


# Optimization level, can be [0, 1, 2, 3, s].
#     0 = turn off optimization. s = optimize for size.
#     (Note: 3 is not always the best optimization level. See avr-libc FAQ.)
OPT = s


# Debugging format.
#     Native formats for AVR-GCC's -g are dwarf-2 [default] or stabs.
#     AVR Studio 4.10 requires dwarf-2.
#     AVR [Extended] COFF format requires stabs, plus an avr-objcopy run.
#DEBUG = dwarf-2
DEBUG = stabs


# Place -D or -U options here for C sources
CDEFS = -DF_CPU=$(F_CPU)UL


# Compiler flag to set the C Standard level.
#     c89   = "ANSI" C
#     gnu89 = c89 plus GCC extensions
#     c99   = ISO C99 standard (not yet fully implemented)
#     gnu99 = c99 plus GCC extensions
CSTANDARD = -std=gnu99


#---------------- Compiler Options C ----------------
#  -g*:          generate debugging information
#  -O*:          optimization level
#  -f...:        tuning, see GCC manual and avr-libc documentation
#  -Wall...:     warning level
#  -Wa,...:      tell GCC to pass this to the assembler.
#    -adhlns...: create assembler listing
CFLAGS = -g$(DEBUG)
CFLAGS += $(CDEFS)
CFLAGS += -O$(OPT)
#CFLAGS += -mint8
#CFLAGS += -mshort-calls
#CFLAGS += -funsigned-char
#CFLAGS += -funsigned-bitfields
#CFLAGS += -fpack-struct
#CFLAGS += -fshort-enums
#CFLAGS += -fno-unit-at-a-time
CFLAGS += -Wall
#CFLAGS += -Wstrict-prototypes
#CFLAGS += -Wundef
#CFLAGS += -Wunreachable-code
#CFLAGS += -Wsign-compare
#CFLAGS += -Wa,-adhlns=$(<:%.c=$(OBJDIR)/%.lst)
#CFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS))
CFLAGS += $(CSTANDARD)

CFLAGS  += -I$(USB_DRV_DIR) -I. -DDEBUG_LEVEL=0


#============================================================================


# Define programs and commands.
SHELL = sh
CC = avr-gcc
OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump
SIZE = avr-size
NM = avr-nm
AVRDUDE = avrdude
REMOVE = rm -f
REMOVEDIR = rm -rf
COPY = cp
WINSHELL = cmd


#COMPILE = avr-gcc -Wall -Os -DF_CPU=$(F_CPU) $(CFLAGS) -mmcu=$(MCU)
#COMPILE = avr-gcc -Wall -O0 -gdwarf-2 -DF_CPU=$(F_CPU) $(CFLAGS) -mmcu=$(MCU)
COMPILE = $(CC)		\
	$(CFLAGS)		\
	-mmcu=$(MCU)
#COMPILE = avr-gcc -Wall -O$(OPT) -g$(DEBUG) -DF_CPU=$(F_CPU) $(CFLAGSMY) -mmcu=$(MCU)







#============================================================================
##---------------- Programming Options (avrdude) ----------------

# Programming hardware: alf avr910 avrisp bascom bsd
# dt006 pavr picoweb pony-stk200 sp12 stk200 stk500
#
# Type: avrdude -c ?
# to get a full listing.
#
AVRDUDE_PROGRAMMER = $(MY_AVRDUDE_PROGRAMMER)

# com1 = serial port. Use lpt1 to connect to parallel port.
AVRDUDE_PORT = $(MY_PROGRAMMER_PORT)

AVRDUDE_WRITE_FLASH = -U flash:w:$(TARGET).hex
#AVRDUDE_WRITE_EEPROM = -U eeprom:w:$(TARGET).eep


# Uncomment the following if you want avrdude's erase cycle counter.
# Note that this counter needs to be initialized first using -Yn,
# see avrdude manual.
#AVRDUDE_ERASE_COUNTER = -y

# Uncomment the following if you do /not/ wish a verification to be
# performed after programming the MCU.
#AVRDUDE_NO_VERIFY = -V

# Increase verbosity level.  Please use this when submitting bug
# reports about avrdude. See <http://savannah.nongnu.org/projects/avrdude>
# to submit bug reports.
AVRDUDE_VERBOSE = -v -v

AVRDUDE_FLAGS = -p $(MCU)					\
				-P $(AVRDUDE_PORT)			\
				-c $(AVRDUDE_PROGRAMMER)	\
				$(AVRDUDE_NO_VERIFY)		\
				$(AVRDUDE_VERBOSE)			\
				$(AVRDUDE_ERASE_COUNTER)
#AVRDUDE_FLAGS += $(AVRDUDE_NO_VERIFY)
#AVRDUDE_FLAGS += $(AVRDUDE_VERBOSE)
#AVRDUDE_FLAGS += $(AVRDUDE_ERASE_COUNTER)
##---------------- Programming Options (avrdude) ----------------
#============================================================================






#============================================================================
# символические цели (symbolic targets):
help:
	@echo "This Makefile has no default rule. Use one of the following:"
	@echo "make all ....... to build $(TARGET).hex"
	@echo "make program ... to flash the firmware"
	@echo "make clean ..... to delete objects and hex file"




#------------------------------------------------------------------
# ===== Сообщения в выводе

# Define Messages
# Русский
MSG_ERRORS_NONE = Ошибок: нет
MSG_BEGIN 	= -------- Начало компиляции --------
MSG_END 	= -------- Конец компиляции ---------
MSG_SIZE_BEFORE 	= --- Размер до:
MSG_SIZE_AFTER 		= --- Размер после:
MSG_COFF 			= --- Конвертирование в AVR COFF:
MSG_EXTENDED_COFF 	= --- Конвертирование в AVR Extended COFF:
MSG_FLASH 			= --- Создание загрузочного файла для Flash:
MSG_EEPROM 			= --- Создание загрузочного файла для EEPROM:
MSG_EXTENDED_LISTING = --- Создание разширенного листинга:
MSG_SYMBOL_TABLE 	= --- Создание таблицы обозначений:
MSG_LINKING 		= --- Линковка:
MSG_COMPILING 		= --- Компиляция C:
MSG_COMPILING_CPP 	= --- Компиляция C++:
MSG_ASSEMBLING 		= --- Ассемблирование:
MSG_CLEANING 		= --- Отчистка проекта:
MSG_CREATING_LIBRARY = --- Создание библиотеки:
# ===== Сообщения в выводе
#------------------------------------------------------------------



#------------------
# построение проекта

# Eye candy.
# AVR Studio 3.x does not check make's exit code but relies on
# the following magic strings to be generated by the compile job.
begin:
	@echo
	@echo $(MSG_BEGIN)

end:
	@echo $(MSG_END)
	@echo


# Display size of file.
HEXSIZE = $(SIZE) --target=$(FORMAT) $(TARGET).hex
ELFSIZE = $(SIZE) -A $(TARGET).elf  -C --mcu=$(MCU)
AVRMEM = avr-mem.sh $(TARGET).elf $(MCU)

sizebefore:
	@if test -f $(TARGET).elf; then echo; echo $(MSG_SIZE_BEFORE); $(ELFSIZE); \
	$(AVRMEM) 2>/dev/null; echo; fi

sizeafter:
	@if test -f $(TARGET).elf; then echo; echo $(MSG_SIZE_AFTER); $(ELFSIZE); \
	$(AVRMEM) 2>/dev/null; echo; fi



# Display compiler version information.
gccversion :
	@$(CC) --version

#==>
all: begin gccversion sizebefore build sizeafter end

build: $(TARGET).hex
#------------------



#------------------
# Прошивка памяти микроконтроллера
#Program the device.
program: $(TARGET).hex
	$(AVRDUDE) \
	$(AVRDUDE_FLAGS) \
	$(AVRDUDE_WRITE_FLASH) \
	$(AVRDUDE_WRITE_EEPROM)

#program: $(TARGET).hex
#	$(AVRDUDE_WRITE_FLASH) -U flash:w:$(TARGET).hex:i
#------------------



#------------------
# правило для удаления файлов зависимостей (которые могут быть построены утилитой Make):
clean_msg:
	@echo
	@echo $(MSG_CLEANING)

clean_all:
	rm -f $(TARGET).hex	\
	$(TARGET).lst		\
	$(TARGET).obj		\
	$(TARGET).cof		\
	$(TARGET).list		\
	$(TARGET).map		\
	$(TARGET).eep.hex	\
	$(TARGET).elf		\
	*.o usbdrv/*.o

clean: clean_msg clean_all
#------------------




# Обычное (generic) правило для компилирования файлов на языке C:
.c.o:
	$(COMPILE) -c $< -o $@

# Обычное (generic) правило для компиляции файлов на языке ассемблера:
.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@
# "-x assembler-with-cpp" не должен быть необходим, поскольку тип файлов
#  с расширением .S (здесь заглавная S) задан по умолчанию. Однако, символы
#  в верхнем регистре не всегда сохраняются в именах на Windows. Чтобы обеспечить
#  совместимость с WinAVR задайте тип файла вручную.

# Generic правило для компилирования файлов языка C в ассемблер, успользуется только для отладки.
.c.s:
	$(COMPILE) -S $< -o $@

# файловые цели (file targets):
# Поскольку мы не хотим поставлять драйвер много раз с каждым пректом, мы копируем драйвер
#  в этот проект:
#usbdrv:
#	cp -r ../../../usbdrv .

$(TARGET).elf: $(USB_DRV_DIR) $(OBJECTS)	# зависимость (dependency) usbdrv нужна только для копирования
	$(COMPILE) -o $(TARGET).elf $(OBJECTS)

$(TARGET).hex: $(TARGET).elf
	rm -f $(TARGET).hex $(TARGET).eep.hex
	avr-objcopy -j .text -j .data -O ihex $(TARGET).elf $(TARGET).hex
#	avr-objcopy --debugging -O coff-ext-avr $< $(TARGET).cof
	avr-size $(TARGET).hex
	avr-size $(TARGET).elf

# цели отладки:
disasm:	$(TARGET).elf
	avr-objdump -d $(TARGET).elf

cpp:
	$(COMPILE) -E $(TARGET).c
