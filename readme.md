# LPC11U24 Common files
This repository contains CMSIS, peripheral drivers, sample Makefile and linker script for compiling firmware for NXP LPC11U24 MCUs with the [Yagarto toolchain](http://yagarto.de) on Mac OS X (probably Linux too).

## New projects
To use these files in a new project, do the following:

1. Create a new folder for the firmware.
2. Include this repository as a submodule (or export and copy the files into a folder).
3. Create a `main.c` file (possibly in a `src` subfolder if you wish).
4. Copy the Makefile out from the subfolder to the main folder.
5. Edit the Makefile and set project name, path to the common dir (containing this repository) and specify paths to source and header file.

So a sample project would look like this:

	/.../myfirmware_project
	   |-common_LPC11U24/
	   |---LPC11U24.ld
	   |---makefile
	   |---cmsis/
	   |-----core/
	   |-------core_cm0.c
	   |-------core_cm0.h
	   |-------core_cmFunc.h
	   |-------core_cmInstr.h
	   |-----device/
	   |-------LPC11Uxx.h
	   |-------power_api.h
	   |-------startup_LPC11U24.c
	   |-------system_LPC11Uxx.c
	   |-------system_LPC11Uxx.h
	   |---drivers/
	   |-----soft_uart/
	   |-------[...]
	   |-----uart/
	   |-------[...]
	   |-----usb/
	   |-------[...]
	   |-----usb_cdc/
	   |-------[...]
	   |-----[...]/
	   |-main.c
	   |-makefile
