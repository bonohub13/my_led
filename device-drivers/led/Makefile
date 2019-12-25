obj-m:= myled.o #verify the name of the object file (filename.o)

myled.ko: myled.c
	make -C /usr/src/linux-headers-$(shell uname -r) M=`pwd` V=1 modules # runs at "make"
clean:
	make -C /usr/src/linux-headers-$(shell uname -r) M=`pwd` V=1 clean # runs "make clean"
