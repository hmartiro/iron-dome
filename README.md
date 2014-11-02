iron-dome
=========

A robotic projectile defense system with style. A project for Experimental Robotics (CS225A) at Stanford.

Authors:

 * Hayk Martirosyan
 * Daniel O'Shea
 * Wanxi Liu

How to set up OpenNI2 and Libfreenect:
---------------------------------------

1.Download and unpack the latest version of [libusb](http://sourceforge.net/projects/libusb/files/libusb-1.0/libusb-1.0.19/libusb-1.0.19.tar.bz2/download). Go to the top libusb-1.0.19 folder and compile

```
	./Configure
	make
	sudo make install
	sudo ld config
```

2.Use Git clone to get [libfreenect](https://github.com/OpenKinect/libfreenect)
3.Make sure you have all dependencies installed on your computer (you can run step 4 to see if there's error)
4.Go to the top libfreenect directory and build it with the OpenNI2 driver.

```
	mkdir build
	cd build
	cmake .. --DBUILD_OPENNI2_DRIVER=ON
	make
	export LD_LIBRARY_PATH=($YourPathtoLibfreenectFolder)/build/lib/
```
5.Download and unpack OpenNI 2.2.0.33 Beta(X64/X86) from (http://structure.io/openni)
6.Go to the top OpenNI2 folder (usually OpenNI-Linux-X64-2.2) and run install

```
	sudo sh install.sh
```
7.Now copy the OpenNI2-FreenectDriver (libFreenectDriver.so.0.5.0, libFreenectDriver.so.0.5, libFreenectDriver.so) from *($YourPathtoLibfreenectFolder)/build/lib/OpenNI2-FreenectDriver* to *($YourPathtoOpenNI2Folder)/Samples/Bin/OpenNI2/Drivers/* and *($YourPathtoOpenNI2Folder)/Redist/OpenNI2/Drivers/*  
8.Plug in Kinect and in the terminal go to ($YourPathtoOpenNI2Folder)/Samples/Bin, run the SimpleViewer.

```
	sudo ./SimpleViewer
```

