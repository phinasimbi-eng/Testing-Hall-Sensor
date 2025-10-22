# Makefile for N32G430 Motor Control Project using ARM GCC

TARGET = firmware

CC = arm-none-eabi-gcc
AS = arm-none-eabi-as
LD = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SIZE = arm-none-eabi-size

# Compiler flags
CFLAGS = -mcpu=cortex-m4 -mthumb -O2 -Wall -Wextra -std=c99 \
         -ffunction-sections -fdata-sections \
         -DMATH_TYPE=1 \
         -Wno-unused-parameter \
         -Wno-unused-variable \
         -Wno-unused-but-set-variable \
         -Wno-parentheses \
         -Wno-strict-aliasing \
         -Wno-absolute-value \
         -IApp/inc \
         -IDriver/CMSIS/core \
         -IDriver/CMSIS/device \
         -IDriver/n32g430_std_periph_driver/inc \
         -IFoc/inc \
         -IFoc/iqmath \
         -IFoc/user \
         -I. \
         -Icontrollers

# Linker flags
LDFLAGS = -mcpu=cortex-m4 -mthumb -T linker_script.ld \
          -Wl,--gc-sections -Wl,-Map=$(TARGET).map

# Source files
C_SOURCES = \
    App/src/Adc.c \
    App/src/main.c \
    App/src/Pwm.c \
    App/src/Spi.c \
    App/src/System_Init.c \
    App/src/SystemClock.c \
    App/src/Uart.c \
    Driver/CMSIS/device/system_n32g430.c \
    Driver/n32g430_std_periph_driver/src/misc.c \
    Driver/n32g430_std_periph_driver/src/n32g430_adc.c \
    Driver/n32g430_std_periph_driver/src/n32g430_beeper.c \
    Driver/n32g430_std_periph_driver/src/n32g430_can.c \
    Driver/n32g430_std_periph_driver/src/n32g430_comp.c \
    Driver/n32g430_std_periph_driver/src/n32g430_crc.c \
    Driver/n32g430_std_periph_driver/src/n32g430_dbg.c \
    Driver/n32g430_std_periph_driver/src/n32g430_dma.c \
    Driver/n32g430_std_periph_driver/src/n32g430_exti.c \
    Driver/n32g430_std_periph_driver/src/n32g430_flash.c \
    Driver/n32g430_std_periph_driver/src/n32g430_gpio.c \
    Driver/n32g430_std_periph_driver/src/n32g430_i2c.c \
    Driver/n32g430_std_periph_driver/src/n32g430_iwdg.c \
    Driver/n32g430_std_periph_driver/src/n32g430_lptim.c \
    Driver/n32g430_std_periph_driver/src/n32g430_pwr.c \
    Driver/n32g430_std_periph_driver/src/n32g430_rcc.c \
    Driver/n32g430_std_periph_driver/src/n32g430_rtc.c \
    Driver/n32g430_std_periph_driver/src/n32g430_spi.c \
    Driver/n32g430_std_periph_driver/src/n32g430_tim.c \
    Driver/n32g430_std_periph_driver/src/n32g430_usart.c \
    Driver/n32g430_std_periph_driver/src/n32g430_wwdg.c \
    Foc/src/BrushlessPiCtrl.c \
    Foc/src/CurrentLoop.c \
    Foc/src/Data_Uart.c \
    Foc/src/DataDeal.c \
    Foc/src/EncObserver.c \
    Foc/src/ErrDeal.c \
    Foc/src/Hall.c \
    Foc/src/IdIqFedBak.c \
    Foc/src/MotorDrive.c \
    Foc/src/PublicDefine.c \
    Foc/src/SpeedCtrl.c \
    Foc/src/StartStopCtrl.c \
    Foc/src/Svpwm.c \
    Foc/src/FunctionWrappers.c \
    Foc/user/bsp_delay.c \
    Foc/user/i2c_drv.c \
    Foc/user/myiic.c \
    Foc/user/oled.c
    # TODO: Integrate modular controllers once struct mismatches are resolved
    # main_controller_v3.c \
    # controllers/motor_controller.c \
    # controllers/hardware_controller.c \
    # controllers/safety_controller.c \
    # controllers/communication_controller.c
    # Issues to fix:
    #  - CurLoop_Obj has no IqTarget field (line 234 in motor_controller.c)
    #  - MotorFeedback_t has no motor_id field (line 580)
    #  - Need to add HardwareController_GetTimestampMs() implementation

ASM_SOURCES = \
    startup_gcc.s

# Modular architecture files (not currently in build, but keep clean)
MODULAR_SOURCES = \
    main_controller.c \
    main_controller_v3.c \
    communication_manager.c \
    system_diagnostics.c \
    controllers/motor_controller.c \
    controllers/hardware_controller.c \
    controllers/safety_controller.c \
    controllers/communication_controller.c

# Object files
C_OBJECTS = $(C_SOURCES:.c=.o)
ASM_OBJECTS = $(ASM_SOURCES:.s=.o)
OBJECTS = $(C_OBJECTS) $(ASM_OBJECTS)

# Modular object and dependency files (for cleanup)
MODULAR_OBJECTS = $(MODULAR_SOURCES:.c=.o)
MODULAR_DEPS = $(MODULAR_SOURCES:.c=.d)

# Dependency files
DEPS = $(C_OBJECTS:.o=.d)

# Build rules
all: $(TARGET).elf $(TARGET).bin $(TARGET).hex

$(TARGET).elf: $(OBJECTS)
	$(LD) $(LDFLAGS) -o $@ $^
	$(SIZE) $@

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

%.o: %.c
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

%.o: %.s
	$(AS) -mcpu=cortex-m4 -mthumb -c $< -o $@

clean:
	rm -f $(OBJECTS) $(DEPS) $(MODULAR_OBJECTS) $(MODULAR_DEPS) $(TARGET).elf $(TARGET).bin $(TARGET).hex $(TARGET).map

rebuild: clean all

.PHONY: all clean rebuild

# Include dependency files
-include $(DEPS)