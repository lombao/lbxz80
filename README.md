## LBXZ80
Z80 Emulator 

# Why ?
Why not. Next question

# HOW TO COMPILE
``` 
./autogen.sh
./configure
make
sudo make install

The resulting objects are ( relative to the chosen prefix , use ./configure --prefix=.... to choose your own prefix ):
./
./lib
./lib/pkgconfig
./lib/pkgconfig/lbxz80.pc
./lib/liblbxz80.so.2.0.1
./lib/liblbxz80.so.2
./lib/liblbxz80.so
./lib/liblbxz80.la
./include
./include/lbxz80.h
```

The build requirements are:
- Obviously, autoconf and automake and libtool
- any non-prehistoric glibc 

# BSD and others
LBXZ80 has been developed on Linux platforms, but ideally it should run 
on BSD systems with minor or no changes at all, but I do not have any of 
those systems to test it. I will accept patches to improve or fix 
compatibility with UX systems. Non UX-OSes are out of scope.

# Bugs, suggestions and comments
Please , send them to me: cesar dot lombao at gmail dot com .

# How to use INT in MODE 0
In the initialization you passed a pointer to a struct z80signals. You 
need to modify that struct in the following way
* Of course, Set the interruption mode of the Z80 to Mode 0 with the instruciton IM0
* Copy the instructions to the inst array of the z80signals struct( i.e: CD00AB )
* Set the ptrinst to 0 in the z80signals struct
* Modify the field INT of the struct z80signals you passed to the init to 1 

# How to use INT in MODE 2
Using the struct z80signals you passed at the init stage
* Of course, set the IM2 at some point
* Copy the vector address in the array inst of the struct ( just in the 0 poistion of the array ). Please write a 8 bits byte, the Z80 will set the lowest bit to 0 as it is only expected a 7 bit data, in any 
case you should write a 8 bit
* Set the ptrinst to 0 in the z80signals struct
* Modify the field INT of the z80signals struct to set 1 ( this signals interruption )
