
MAKEFLAGS = -j8

RTSLIB = -iC:/IAR/arm/lib/ -llibc.a

XDCLIB = -l "C:/ti/xdctools_3_32_00_06_core/packages/iar/targets/arm/rts/lib/release/boot.arm3" 

XOPTS = -I"C:/ti/xdctools_3_32_00_06_core/packages/" -I"C:/ti/xdctools_3_32_00_06_core/packages/" -Dxdc_target_types__=C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/iar/targets/arm/std.h -Dxdc_target_name__=M3

vpath % C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/
vpath %.c C:/ti/xdctools_3_32_00_06_core/packages/

CCOPTS    = --aeabi --cpu=Cortex-M3 --diag_suppress=Pa050,Go005 --endian=little -e --thumb --silent  --mfc -Ohz  -Dti_sysbios_knl_Task_minimizeLatency__D=FALSE -Dti_sysbios_family_arm_cc26xx_Boot_driverlibVersion=2 -Dti_sysbios_knl_Clock_stopCheckNext__D=TRUE -Dti_sysbios_family_arm_m3_Hwi_enableException__D=TRUE -Dti_sysbios_family_arm_m3_Hwi_disablePriority__D=32U -Dti_sysbios_family_arm_m3_Hwi_numSparseInterrupts__D=0U  --dlib_config C:/IAR/arm/inc/c/DLib_Config_Normal.h

XDC_ROOT  = C:/ti/xdctools_3_32_00_06_core/packages/

BIOS_ROOT = C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/

BIOS_DEFS =  -Dti_sysbios_BIOS_swiEnabled__D=TRUE -Dti_sysbios_BIOS_taskEnabled__D=TRUE -Dti_sysbios_BIOS_clockEnabled__D=TRUE -Dti_sysbios_BIOS_runtimeCreatesEnabled__D=TRUE -Dti_sysbios_knl_Task_moduleStateCheckFlag__D=FALSE -Dti_sysbios_knl_Task_objectCheckFlag__D=FALSE -Dti_sysbios_hal_Hwi_DISABLE_ALL_HOOKS -Dti_sysbios_knl_Swi_DISABLE_ALL_HOOKS -Dti_sysbios_BIOS_smpEnabled__D=FALSE -Dti_sysbios_Build_useHwiMacros -Dti_sysbios_knl_Swi_numPriorities__D=6 -Dti_sysbios_knl_Task_deleteTerminatedTasks__D=FALSE -Dti_sysbios_knl_Task_numPriorities__D=6 -Dti_sysbios_knl_Task_checkStackFlag__D=FALSE -Dti_sysbios_knl_Task_initStackFlag__D=TRUE -Dti_sysbios_knl_Task_DISABLE_ALL_HOOKS -Dti_sysbios_knl_Clock_TICK_SOURCE=ti_sysbios_knl_Clock_TickSource_TIMER -Dti_sysbios_knl_Clock_TICK_MODE=ti_sysbios_knl_Clock_TickMode_DYNAMIC -Dti_sysbios_hal_Core_delegate_getId=ti_sysbios_hal_CoreNull_getId__E -Dti_sysbios_hal_Core_delegate_interruptCore=ti_sysbios_hal_CoreNull_interruptCore__E -Dti_sysbios_hal_Core_delegate_lock=ti_sysbios_hal_CoreNull_lock__E -Dti_sysbios_hal_Core_delegate_unlock=ti_sysbios_hal_CoreNull_unlock__E -Dti_sysbios_hal_Core_numCores__D=1 -Dti_sysbios_hal_CoreNull_numCores__D=1 -Dti_sysbios_utils_Load_taskEnabled__D=TRUE -Dti_sysbios_utils_Load_swiEnabled__D=FALSE -Dti_sysbios_utils_Load_hwiEnabled__D=FALSE -Dti_sysbios_family_arm_m3_Hwi_dispatcherSwiSupport__D=TRUE -Dti_sysbios_family_arm_m3_Hwi_dispatcherTaskSupport__D=TRUE -Dti_sysbios_family_arm_m3_Hwi_dispatcherAutoNestingSupport__D=TRUE -Dti_sysbios_family_arm_m3_Hwi_dispatcherIrpTrackingSupport__D=TRUE -Dti_sysbios_knl_Semaphore_supportsEvents__D=FALSE -Dti_sysbios_knl_Semaphore_supportsPriority__D=FALSE -Dxdc_runtime_Assert_DISABLE_ALL -Dxdc_runtime_Log_DISABLE_ALL

INTERN_FUNCS = ""

BIOS_INC  = -I"C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/"

TARGET_INC = -I"C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/"

INCS = $(BIOS_INC) $(TARGET_INC)   -IE:/BeelinkerCode/CC2640F128/Project/cc2650em/simple_observer/iar/app/../../../../../src/controller/cc26xx/inc/   -IE:/BeelinkerCode/CC2640F128/Project/cc2650em/simple_observer/iar/app/../../../../../src/inc/   -IE:/BeelinkerCode/CC2640F128/Project/cc2650em/simple_observer/iar/app/../../../../../src/common/cc26xx/   -IE:/BeelinkerCode/CC2640F128/Project/cc2650em/simple_observer/iar/app/../../../../../src/icall/inc/   -IE:/BeelinkerCode/CC2640F128/Project/cc2650em/simple_observer/iar/app/../../../../../src/inc/   -IE:/BeelinkerCode/CC2640F128/Project/cc2650em/simple_observer/iar/app/../../../../../src/profiles/roles/   -IE:/BeelinkerCode/CC2640F128/Project/cc2650em/simple_observer/iar/app/../../../../../src/profiles/roles/cc26xx/   -IE:/BeelinkerCode/CC2640F128/Project/cc2650em/simple_observer/iar/app/../../../../../src/target/   -IE:/BeelinkerCode/CC2640F128/Project/cc2650em/simple_observer/iar/app/../../../../../src/components/hal/src/inc/   -IE:/BeelinkerCode/CC2640F128/Project/cc2650em/simple_observer/iar/app/../../../../../src/components/hal/src/target/_common/   -IE:/BeelinkerCode/CC2640F128/Project/cc2650em/simple_observer/iar/app/../../../../../src/components/hal/src/target/_common/cc26xx/   -IE:/BeelinkerCode/CC2640F128/Project/cc2650em/simple_observer/iar/app/../../../../../src/components/hal/src/target/cc2650/rom/   -IE:/BeelinkerCode/CC2640F128/Project/cc2650em/simple_observer/iar/app/../../../../../src/components/heapmgr/   -IE:/BeelinkerCode/CC2640F128/Project/cc2650em/simple_observer/iar/app/../../../../../src/components/icall/src/inc/   -IE:/BeelinkerCode/CC2640F128/Project/cc2650em/simple_observer/iar/app/../../../../../src/components/osal/src/inc/   -IE:/BeelinkerCode/CC2640F128/Project/cc2650em/simple_observer/iar/app/../../../../../src/components/services/src/saddr/   -IE:/BeelinkerCode/CC2640F128/Project/cc2650em/simple_observer/iar/app/../../../../../src/components/services/src/sdata/   -IC:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/cc26xxware_2_24_03_17272/   -IC:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/tidrivers_cc13xx_cc26xx_2_21_01_01/packages/   -IE:/BeelinkerCode/CC2640F128/Project/cc2650em/simple_observer/iar/app/../../../../../src/common/cc26xx/rcosc/   -IC:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ 

CC = LC_ALL=C  C:/IAR/arm/bin/iccarm $(CCOPTS)
ASM = LC_ALL=C  C:/IAR/arm/bin/iasmarm --cpu Cortex-M3 --endian little -S
AR = LC_ALL=C  C:/IAR/arm/bin/iarchive 

CC_LIB = C:/IAR/arm/bin/iccarm --aeabi --cpu=Cortex-M3 --diag_suppress=Pa050,Go005 --endian=little -e --thumb --silent  --mfc -Ohz  -Dti_sysbios_knl_Task_minimizeLatency__D=FALSE -Dti_sysbios_family_arm_cc26xx_Boot_driverlibVersion=2 -Dti_sysbios_knl_Clock_stopCheckNext__D=TRUE -Dti_sysbios_family_arm_m3_Hwi_enableException__D=TRUE -Dti_sysbios_family_arm_m3_Hwi_disablePriority__D=32U -Dti_sysbios_family_arm_m3_Hwi_numSparseInterrupts__D=0U -I C:/IAR/arm/include

CC_PP = C:/IAR/arm/bin/iccarm --aeabi --cpu=Cortex-M3 --diag_suppress=Pa050,Go005 --endian=little -e --thumb --silent  --mfc   -Dti_sysbios_knl_Task_minimizeLatency__D=FALSE -Dti_sysbios_family_arm_cc26xx_Boot_driverlibVersion=2 -Dti_sysbios_knl_Clock_stopCheckNext__D=TRUE -Dti_sysbios_family_arm_m3_Hwi_enableException__D=TRUE  -Dti_sysbios_family_arm_m3_Hwi_numSparseInterrupts__D=0U -I C:/IAR/arm/include  --dlib_config C:/IAR/arm/inc/c/DLib_Config_Normal.h

DEL = C:/ti/xdctools_3_32_00_06_core/packages/../bin/rm -f
CP = C:/ti/xdctools_3_32_00_06_core/packages/../bin/cp -f

define RM
	$(if $(wildcard $1),$(DEL) $1,:)
endef

define ASSEMBLE
	@echo asmrm3 $< ...
	@$(ASM) $(BIOS_DEFS) $(XOPTS) $(INCS) $<
endef

all: rom_sysbios.arm3

include C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/golden/CC26xx/symbolMapFile.mak

rom_arm_m3_Hwi_asm_iar_lib.obj: family/arm/m3/Hwi_asm_iar.sv7M makefile
	@$(XDC_ROOT)/../xs -c C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleAsmFile.xs $(BIOS_ROOT)family/arm/m3/Hwi_asm_iar.sv7M rom_arm_m3_Hwi_asm_iar_lib.asm romInternFuncs.txt
	@$(ASM) -o rom_arm_m3_Hwi_asm_iar_lib.obj -I$(BIOS_ROOT)family/arm/m3 $(BIOS_DEFS) $(XOPTS) $(INCS) rom_arm_m3_Hwi_asm_iar_lib.asm

rom_arm_m3_Hwi_asm_switch_iar_lib.obj: family/arm/m3/Hwi_asm_switch_iar.sv7M makefile
	@$(XDC_ROOT)/../xs -c C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleAsmFile.xs $(BIOS_ROOT)family/arm/m3/Hwi_asm_switch_iar.sv7M rom_arm_m3_Hwi_asm_switch_iar_lib.asm romInternFuncs.txt
	@$(ASM) -o rom_arm_m3_Hwi_asm_switch_iar_lib.obj -I$(BIOS_ROOT)family/arm/m3 $(BIOS_DEFS) $(XOPTS) $(INCS) rom_arm_m3_Hwi_asm_switch_iar_lib.asm

rom_arm_m3_IntrinsicsSupport_asm_iar_lib.obj: family/arm/m3/IntrinsicsSupport_asm_iar.sv7M makefile
	@$(XDC_ROOT)/../xs -c C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleAsmFile.xs $(BIOS_ROOT)family/arm/m3/IntrinsicsSupport_asm_iar.sv7M rom_arm_m3_IntrinsicsSupport_asm_iar_lib.asm romInternFuncs.txt
	@$(ASM) -o rom_arm_m3_IntrinsicsSupport_asm_iar_lib.obj -I$(BIOS_ROOT)family/arm/m3 $(BIOS_DEFS) $(XOPTS) $(INCS) rom_arm_m3_IntrinsicsSupport_asm_iar_lib.asm

rom_arm_m3_TaskSupport_asm_iar_lib.obj: family/arm/m3/TaskSupport_asm_iar.sv7M makefile
	@$(XDC_ROOT)/../xs -c C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleAsmFile.xs $(BIOS_ROOT)family/arm/m3/TaskSupport_asm_iar.sv7M rom_arm_m3_TaskSupport_asm_iar_lib.asm romInternFuncs.txt
	@$(ASM) -o rom_arm_m3_TaskSupport_asm_iar_lib.obj -I$(BIOS_ROOT)family/arm/m3 $(BIOS_DEFS) $(XOPTS) $(INCS) rom_arm_m3_TaskSupport_asm_iar_lib.asm


rom__BIOS_lib.pp: BIOS.c makefile
	@echo Preprocessing library source files ...
	@$(CC_PP) --preprocess=nl _BIOS_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/BIOS.c
	@$(XDC_ROOT)/../xs -c C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs _BIOS_lib.pp rom__BIOS_lib.pp romInternFuncs.txt  "$(_BIOS_lib.pp_SYMS)"

rom_knl_Clock_lib.pp: knl/Clock.c makefile
	@$(CC_PP) --preprocess=nl knl_Clock_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/knl/Clock.c
	@$(XDC_ROOT)/../xs -c C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs knl_Clock_lib.pp rom_knl_Clock_lib.pp romInternFuncs.txt  "$(knl_Clock_lib.pp_SYMS)"

rom_knl_Idle_lib.pp: knl/Idle.c makefile
	@$(CC_PP) --preprocess=nl knl_Idle_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/knl/Idle.c
	@$(XDC_ROOT)/../xs -c C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs knl_Idle_lib.pp rom_knl_Idle_lib.pp romInternFuncs.txt  "$(knl_Idle_lib.pp_SYMS)"

rom_knl_Intrinsics_lib.pp: knl/Intrinsics.c makefile
	@$(CC_PP) --preprocess=nl knl_Intrinsics_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/knl/Intrinsics.c
	@$(XDC_ROOT)/../xs -c C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs knl_Intrinsics_lib.pp rom_knl_Intrinsics_lib.pp romInternFuncs.txt  "$(knl_Intrinsics_lib.pp_SYMS)"

rom_knl_Event_lib.pp: knl/Event.c makefile
	@$(CC_PP) --preprocess=nl knl_Event_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/knl/Event.c
	@$(XDC_ROOT)/../xs -c C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs knl_Event_lib.pp rom_knl_Event_lib.pp romInternFuncs.txt  "$(knl_Event_lib.pp_SYMS)"

rom_knl_Mailbox_lib.pp: knl/Mailbox.c makefile
	@$(CC_PP) --preprocess=nl knl_Mailbox_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/knl/Mailbox.c
	@$(XDC_ROOT)/../xs -c C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs knl_Mailbox_lib.pp rom_knl_Mailbox_lib.pp romInternFuncs.txt  "$(knl_Mailbox_lib.pp_SYMS)"

rom_knl_Queue_lib.pp: knl/Queue.c makefile
	@$(CC_PP) --preprocess=nl knl_Queue_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/knl/Queue.c
	@$(XDC_ROOT)/../xs -c C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs knl_Queue_lib.pp rom_knl_Queue_lib.pp romInternFuncs.txt  "$(knl_Queue_lib.pp_SYMS)"

rom_knl_Semaphore_lib.pp: knl/Semaphore.c makefile
	@$(CC_PP) --preprocess=nl knl_Semaphore_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/knl/Semaphore.c
	@$(XDC_ROOT)/../xs -c C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs knl_Semaphore_lib.pp rom_knl_Semaphore_lib.pp romInternFuncs.txt  "$(knl_Semaphore_lib.pp_SYMS)"

rom_knl_Swi_lib.pp: knl/Swi.c makefile
	@$(CC_PP) --preprocess=nl knl_Swi_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/knl/Swi.c
	@$(XDC_ROOT)/../xs -c C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs knl_Swi_lib.pp rom_knl_Swi_lib.pp romInternFuncs.txt  "$(knl_Swi_lib.pp_SYMS)"

rom_knl_Swi_andn_lib.pp: knl/Swi_andn.c makefile
	@$(CC_PP) --preprocess=nl knl_Swi_andn_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/knl/Swi_andn.c
	@$(XDC_ROOT)/../xs -c C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs knl_Swi_andn_lib.pp rom_knl_Swi_andn_lib.pp romInternFuncs.txt  "$(knl_Swi_andn_lib.pp_SYMS)"

rom_knl_Task_lib.pp: knl/Task.c makefile
	@$(CC_PP) --preprocess=nl knl_Task_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/knl/Task.c
	@$(XDC_ROOT)/../xs -c C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs knl_Task_lib.pp rom_knl_Task_lib.pp romInternFuncs.txt  "$(knl_Task_lib.pp_SYMS)"

rom_arm_m3_Hwi_lib.pp: family/arm/m3/Hwi.c makefile
	@$(CC_PP) --preprocess=nl arm_m3_Hwi_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/family/arm/m3/Hwi.c
	@$(XDC_ROOT)/../xs -c C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs arm_m3_Hwi_lib.pp rom_arm_m3_Hwi_lib.pp romInternFuncs.txt  "$(arm_m3_Hwi_lib.pp_SYMS)"

rom_arm_m3_TaskSupport_lib.pp: family/arm/m3/TaskSupport.c makefile
	@$(CC_PP) --preprocess=nl arm_m3_TaskSupport_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/family/arm/m3/TaskSupport.c
	@$(XDC_ROOT)/../xs -c C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs arm_m3_TaskSupport_lib.pp rom_arm_m3_TaskSupport_lib.pp romInternFuncs.txt  "$(arm_m3_TaskSupport_lib.pp_SYMS)"

rom_arm_cc26xx_Boot_lib.pp: family/arm/cc26xx/Boot.c makefile
	@$(CC_PP) --preprocess=nl arm_cc26xx_Boot_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/family/arm/cc26xx/Boot.c
	@$(XDC_ROOT)/../xs -c C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs arm_cc26xx_Boot_lib.pp rom_arm_cc26xx_Boot_lib.pp romInternFuncs.txt  "$(arm_cc26xx_Boot_lib.pp_SYMS)"

rom_arm_cc26xx_Timer_lib.pp: family/arm/cc26xx/Timer.c makefile
	@$(CC_PP) --preprocess=nl arm_cc26xx_Timer_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/family/arm/cc26xx/Timer.c
	@$(XDC_ROOT)/../xs -c C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs arm_cc26xx_Timer_lib.pp rom_arm_cc26xx_Timer_lib.pp romInternFuncs.txt  "$(arm_cc26xx_Timer_lib.pp_SYMS)"

rom_arm_cc26xx_TimestampProvider_lib.pp: family/arm/cc26xx/TimestampProvider.c makefile
	@$(CC_PP) --preprocess=nl arm_cc26xx_TimestampProvider_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/family/arm/cc26xx/TimestampProvider.c
	@$(XDC_ROOT)/../xs -c C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs arm_cc26xx_TimestampProvider_lib.pp rom_arm_cc26xx_TimestampProvider_lib.pp romInternFuncs.txt  "$(arm_cc26xx_TimestampProvider_lib.pp_SYMS)"

rom_hal_Hwi_lib.pp: hal/Hwi.c makefile
	@$(CC_PP) --preprocess=nl hal_Hwi_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/hal/Hwi.c
	@$(XDC_ROOT)/../xs -c C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs hal_Hwi_lib.pp rom_hal_Hwi_lib.pp romInternFuncs.txt  "$(hal_Hwi_lib.pp_SYMS)"

rom_hal_Hwi_stack_lib.pp: hal/Hwi_stack.c makefile
	@$(CC_PP) --preprocess=nl hal_Hwi_stack_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/hal/Hwi_stack.c
	@$(XDC_ROOT)/../xs -c C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs hal_Hwi_stack_lib.pp rom_hal_Hwi_stack_lib.pp romInternFuncs.txt  "$(hal_Hwi_stack_lib.pp_SYMS)"

rom_hal_Hwi_startup_lib.pp: hal/Hwi_startup.c makefile
	@$(CC_PP) --preprocess=nl hal_Hwi_startup_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/hal/Hwi_startup.c
	@$(XDC_ROOT)/../xs -c C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs hal_Hwi_startup_lib.pp rom_hal_Hwi_startup_lib.pp romInternFuncs.txt  "$(hal_Hwi_startup_lib.pp_SYMS)"

rom_gates_GateHwi_lib.pp: gates/GateHwi.c makefile
	@$(CC_PP) --preprocess=nl gates_GateHwi_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/gates/GateHwi.c
	@$(XDC_ROOT)/../xs -c C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs gates_GateHwi_lib.pp rom_gates_GateHwi_lib.pp romInternFuncs.txt  "$(gates_GateHwi_lib.pp_SYMS)"

rom_gates_GateMutex_lib.pp: gates/GateMutex.c makefile
	@$(CC_PP) --preprocess=nl gates_GateMutex_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/gates/GateMutex.c
	@$(XDC_ROOT)/../xs -c C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs gates_GateMutex_lib.pp rom_gates_GateMutex_lib.pp romInternFuncs.txt  "$(gates_GateMutex_lib.pp_SYMS)"

rom_heaps_HeapMem_lib.pp: heaps/HeapMem.c makefile
	@$(CC_PP) --preprocess=nl heaps_HeapMem_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/heaps/HeapMem.c
	@$(XDC_ROOT)/../xs -c C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs heaps_HeapMem_lib.pp rom_heaps_HeapMem_lib.pp romInternFuncs.txt  "$(heaps_HeapMem_lib.pp_SYMS)"

rom_xdc_runtime_xdc_noinit_lib.pp: xdc/runtime/xdc_noinit.c makefile
	@$(CC_PP) --preprocess=nl xdc_runtime_xdc_noinit_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/xdc_noinit.c
	@$(XDC_ROOT)/../xs -c C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs xdc_runtime_xdc_noinit_lib.pp rom_xdc_runtime_xdc_noinit_lib.pp romInternFuncs.txt  "$(xdc_runtime_xdc_noinit_lib.pp_SYMS)"

rom_xdc_runtime_Assert_lib.pp: xdc/runtime/Assert.c makefile
	@$(CC_PP) --preprocess=nl xdc_runtime_Assert_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Assert.c
	@$(XDC_ROOT)/../xs -c C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs xdc_runtime_Assert_lib.pp rom_xdc_runtime_Assert_lib.pp romInternFuncs.txt  "$(xdc_runtime_Assert_lib.pp_SYMS)"

rom_xdc_runtime_Core-mem_lib.pp: xdc/runtime/Core-mem.c makefile
	@$(CC_PP) --preprocess=nl xdc_runtime_Core-mem_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Core-mem.c
	@$(XDC_ROOT)/../xs -c C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs xdc_runtime_Core-mem_lib.pp rom_xdc_runtime_Core-mem_lib.pp romInternFuncs.txt  "$(xdc_runtime_Core-mem_lib.pp_SYMS)"

rom_xdc_runtime_Core-smem_lib.pp: xdc/runtime/Core-smem.c makefile
	@$(CC_PP) --preprocess=nl xdc_runtime_Core-smem_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Core-smem.c
	@$(XDC_ROOT)/../xs -c C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs xdc_runtime_Core-smem_lib.pp rom_xdc_runtime_Core-smem_lib.pp romInternFuncs.txt  "$(xdc_runtime_Core-smem_lib.pp_SYMS)"

rom_xdc_runtime_Core-label_lib.pp: xdc/runtime/Core-label.c makefile
	@$(CC_PP) --preprocess=nl xdc_runtime_Core-label_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Core-label.c
	@$(XDC_ROOT)/../xs -c C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs xdc_runtime_Core-label_lib.pp rom_xdc_runtime_Core-label_lib.pp romInternFuncs.txt  "$(xdc_runtime_Core-label_lib.pp_SYMS)"

rom_xdc_runtime_Core-params_lib.pp: xdc/runtime/Core-params.c makefile
	@$(CC_PP) --preprocess=nl xdc_runtime_Core-params_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Core-params.c
	@$(XDC_ROOT)/../xs -c C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs xdc_runtime_Core-params_lib.pp rom_xdc_runtime_Core-params_lib.pp romInternFuncs.txt  "$(xdc_runtime_Core-params_lib.pp_SYMS)"

rom_xdc_runtime_Diags_lib.pp: xdc/runtime/Diags.c makefile
	@$(CC_PP) --preprocess=nl xdc_runtime_Diags_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Diags.c
	@$(XDC_ROOT)/../xs -c C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs xdc_runtime_Diags_lib.pp rom_xdc_runtime_Diags_lib.pp romInternFuncs.txt  "$(xdc_runtime_Diags_lib.pp_SYMS)"

rom_xdc_runtime_Error_lib.pp: xdc/runtime/Error.c makefile
	@$(CC_PP) --preprocess=nl xdc_runtime_Error_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Error.c
	@$(XDC_ROOT)/../xs -c C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs xdc_runtime_Error_lib.pp rom_xdc_runtime_Error_lib.pp romInternFuncs.txt  "$(xdc_runtime_Error_lib.pp_SYMS)"

rom_xdc_runtime_Gate_lib.pp: xdc/runtime/Gate.c makefile
	@$(CC_PP) --preprocess=nl xdc_runtime_Gate_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Gate.c
	@$(XDC_ROOT)/../xs -c C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs xdc_runtime_Gate_lib.pp rom_xdc_runtime_Gate_lib.pp romInternFuncs.txt  "$(xdc_runtime_Gate_lib.pp_SYMS)"

rom_xdc_runtime_Log_lib.pp: xdc/runtime/Log.c makefile
	@$(CC_PP) --preprocess=nl xdc_runtime_Log_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Log.c
	@$(XDC_ROOT)/../xs -c C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs xdc_runtime_Log_lib.pp rom_xdc_runtime_Log_lib.pp romInternFuncs.txt  "$(xdc_runtime_Log_lib.pp_SYMS)"

rom_xdc_runtime_Memory_lib.pp: xdc/runtime/Memory.c makefile
	@$(CC_PP) --preprocess=nl xdc_runtime_Memory_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Memory.c
	@$(XDC_ROOT)/../xs -c C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs xdc_runtime_Memory_lib.pp rom_xdc_runtime_Memory_lib.pp romInternFuncs.txt  "$(xdc_runtime_Memory_lib.pp_SYMS)"

rom_xdc_runtime_Registry_lib.pp: xdc/runtime/Registry.c makefile
	@$(CC_PP) --preprocess=nl xdc_runtime_Registry_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Registry.c
	@$(XDC_ROOT)/../xs -c C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs xdc_runtime_Registry_lib.pp rom_xdc_runtime_Registry_lib.pp romInternFuncs.txt  "$(xdc_runtime_Registry_lib.pp_SYMS)"

rom_xdc_runtime_Startup_lib.pp: xdc/runtime/Startup.c makefile
	@$(CC_PP) --preprocess=nl xdc_runtime_Startup_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Startup.c
	@$(XDC_ROOT)/../xs -c C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs xdc_runtime_Startup_lib.pp rom_xdc_runtime_Startup_lib.pp romInternFuncs.txt  "$(xdc_runtime_Startup_lib.pp_SYMS)"

rom_xdc_runtime_System_lib.pp: xdc/runtime/System.c makefile
	@$(CC_PP) --preprocess=nl xdc_runtime_System_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/System.c
	@$(XDC_ROOT)/../xs -c C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs xdc_runtime_System_lib.pp rom_xdc_runtime_System_lib.pp romInternFuncs.txt  "$(xdc_runtime_System_lib.pp_SYMS)"

rom_xdc_runtime_SysCallback_lib.pp: xdc/runtime/SysCallback.c makefile
	@$(CC_PP) --preprocess=nl xdc_runtime_SysCallback_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/SysCallback.c
	@$(XDC_ROOT)/../xs -c C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs xdc_runtime_SysCallback_lib.pp rom_xdc_runtime_SysCallback_lib.pp romInternFuncs.txt  "$(xdc_runtime_SysCallback_lib.pp_SYMS)"

rom_xdc_runtime_Text_lib.pp: xdc/runtime/Text.c makefile
	@$(CC_PP) --preprocess=nl xdc_runtime_Text_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Text.c
	@$(XDC_ROOT)/../xs -c C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs xdc_runtime_Text_lib.pp rom_xdc_runtime_Text_lib.pp romInternFuncs.txt  "$(xdc_runtime_Text_lib.pp_SYMS)"

rom_xdc_runtime_Timestamp_lib.pp: xdc/runtime/Timestamp.c makefile
	@$(CC_PP) --preprocess=nl xdc_runtime_Timestamp_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Timestamp.c
	@$(XDC_ROOT)/../xs -c C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs xdc_runtime_Timestamp_lib.pp rom_xdc_runtime_Timestamp_lib.pp romInternFuncs.txt  "$(xdc_runtime_Timestamp_lib.pp_SYMS)"


rom_sysbios.obj: rom__BIOS_lib.pp rom_knl_Clock_lib.pp rom_knl_Idle_lib.pp rom_knl_Intrinsics_lib.pp rom_knl_Event_lib.pp rom_knl_Mailbox_lib.pp rom_knl_Queue_lib.pp rom_knl_Semaphore_lib.pp rom_knl_Swi_lib.pp rom_knl_Swi_andn_lib.pp rom_knl_Task_lib.pp rom_arm_m3_Hwi_lib.pp rom_arm_m3_TaskSupport_lib.pp rom_arm_cc26xx_Boot_lib.pp rom_arm_cc26xx_Timer_lib.pp rom_arm_cc26xx_TimestampProvider_lib.pp rom_hal_Hwi_lib.pp rom_hal_Hwi_stack_lib.pp rom_hal_Hwi_startup_lib.pp rom_gates_GateHwi_lib.pp rom_gates_GateMutex_lib.pp rom_heaps_HeapMem_lib.pp rom_xdc_runtime_xdc_noinit_lib.pp rom_xdc_runtime_Assert_lib.pp rom_xdc_runtime_Core-mem_lib.pp rom_xdc_runtime_Core-smem_lib.pp rom_xdc_runtime_Core-label_lib.pp rom_xdc_runtime_Core-params_lib.pp rom_xdc_runtime_Diags_lib.pp rom_xdc_runtime_Error_lib.pp rom_xdc_runtime_Gate_lib.pp rom_xdc_runtime_Log_lib.pp rom_xdc_runtime_Memory_lib.pp rom_xdc_runtime_Registry_lib.pp rom_xdc_runtime_Startup_lib.pp rom_xdc_runtime_System_lib.pp rom_xdc_runtime_SysCallback_lib.pp rom_xdc_runtime_Text_lib.pp rom_xdc_runtime_Timestamp_lib.pp makefile
	@echo Creating the SYS/BIOS library that contains the APIs not included in the ROM ...
	@$(CC) -o rom_sysbios.obj $(BIOS_DEFS) $(XOPTS) $(INCS) \
	rom__BIOS_lib.pp \
	rom_knl_Clock_lib.pp \
	rom_knl_Idle_lib.pp \
	rom_knl_Intrinsics_lib.pp \
	rom_knl_Event_lib.pp \
	rom_knl_Mailbox_lib.pp \
	rom_knl_Queue_lib.pp \
	rom_knl_Semaphore_lib.pp \
	rom_knl_Swi_lib.pp \
	rom_knl_Swi_andn_lib.pp \
	rom_knl_Task_lib.pp \
	rom_arm_m3_Hwi_lib.pp \
	rom_arm_m3_TaskSupport_lib.pp \
	rom_arm_cc26xx_Boot_lib.pp \
	rom_arm_cc26xx_Timer_lib.pp \
	rom_arm_cc26xx_TimestampProvider_lib.pp \
	rom_hal_Hwi_lib.pp \
	rom_hal_Hwi_stack_lib.pp \
	rom_hal_Hwi_startup_lib.pp \
	rom_gates_GateHwi_lib.pp \
	rom_gates_GateMutex_lib.pp \
	rom_heaps_HeapMem_lib.pp \
	rom_xdc_runtime_xdc_noinit_lib.pp \
	rom_xdc_runtime_Assert_lib.pp \
	rom_xdc_runtime_Core-mem_lib.pp \
	rom_xdc_runtime_Core-smem_lib.pp \
	rom_xdc_runtime_Core-label_lib.pp \
	rom_xdc_runtime_Core-params_lib.pp \
	rom_xdc_runtime_Diags_lib.pp \
	rom_xdc_runtime_Error_lib.pp \
	rom_xdc_runtime_Gate_lib.pp \
	rom_xdc_runtime_Log_lib.pp \
	rom_xdc_runtime_Memory_lib.pp \
	rom_xdc_runtime_Registry_lib.pp \
	rom_xdc_runtime_Startup_lib.pp \
	rom_xdc_runtime_System_lib.pp \
	rom_xdc_runtime_SysCallback_lib.pp \
	rom_xdc_runtime_Text_lib.pp \
	rom_xdc_runtime_Timestamp_lib.pp \

rom_sysbios.arm3: rom_sysbios.obj rom_arm_m3_Hwi_asm_iar_lib.obj rom_arm_m3_Hwi_asm_switch_iar_lib.obj rom_arm_m3_IntrinsicsSupport_asm_iar_lib.obj rom_arm_m3_TaskSupport_asm_iar_lib.obj
	@-$(call RM, $@)
	@$(AR) $@ $^ C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/ti/sysbios/rom/cortexm/cc26xx/golden/CC26xx/rom_sysbios_config.obj
	@echo Library build complete.

clean:
	@-$(call RM, *)
