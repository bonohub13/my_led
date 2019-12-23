obj-m:= myled_17.o #verify the name of the object file (filename.o)

myled.ko: myled_17.c
	make -C /usr/src/linux-headers-`uname -r` M=`pwd` V=1 modules # runs at "make"
clean:
	make -C /usr/src/linux-headers-`uname -r` M=`pwd` V=1 clean # runs "make clean"
