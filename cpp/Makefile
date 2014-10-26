# Makefile : builds libbreezyslam.so C++ library
#
# Copyright (C) Simon D. Levy 2014
#
# This code is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as 
# published by the Free Software Foundation, either version 3 of the 
# License, or (at your option) any later version.
#
# This code is distributed in the hope that it will be useful,     
# but WITHOUT ANY WARRANTY without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License 
#  along with this code.  If not, see <http:#www.gnu.org/licenses/>.
#  You should also have received a copy of the Parrot Parrot AR.Drone 
#  Development License and Parrot AR.Drone copyright notice and disclaimer 
#  and If not, see 
#   <https:#projects.ardrone.org/attachments/277/ParrotLicense.txt> 
# and
#   <https:#projects.ardrone.org/attachments/278/ParrotCopyrightAndDisclaimer.txt>.

# Where you want to put the library
LIBDIR = /usr/local/lib

# Set library extension based on OS
ifeq ("$(shell uname)","Darwin")
  LIBEXT = dylib
else ifeq ("$(shell uname)","Linux")
  CFLAGS = -fPIC
  LIBEXT = so
else
  LIBEXT = dll
endif

ARCH = $(shell uname -m)

# Set SIMD compile params based on architecture
ifeq ("$(ARCH)","armv7l")
  SIMD_FLAGS = -mfpu=neon
else ifeq ("$(ARCH)","i686")
  SIMD_FLAGS = -msse3
else
  ARCH = sisd
endif

all: libbreezyslam.$(LIBEXT)

test: breezytest
	./breezytest

libbreezyslam.$(LIBEXT): algorithms.o  Scan.o Map.o WheeledRobot.o \
                         coreslam.o coreslam_$(ARCH).o random.o ziggurat.o
	g++ -O3 -shared algorithms.o Scan.o Map.o WheeledRobot.o \
                        coreslam.o coreslam_$(ARCH).o random.o ziggurat.o \
          -o libbreezyslam.$(LIBEXT) -lm

algorithms.o: algorithms.cpp algorithms.hpp Laser.hpp Position.hpp Map.hpp Scan.hpp Velocities.hpp \
               WheeledRobot.hpp ../c/coreslam.h 
	g++ -O3 -I../c -c -Wall $(CFLAGS) algorithms.cpp

Scan.o: Scan.cpp Scan.hpp Velocities.hpp Laser.hpp ../c/coreslam.h
	g++ -O3 -I../c -c -Wall $(CFLAGS) Scan.cpp

Map.o: Map.cpp Map.hpp Position.hpp Scan.hpp ../c/coreslam.h
	g++ -O3 -I../c -c -Wall $(CFLAGS) Map.cpp

WheeledRobot.o: WheeledRobot.cpp WheeledRobot.hpp 
	g++ -O3 -I../c -c -Wall $(CFLAGS) WheeledRobot.cpp

coreslam.o: ../c/coreslam.c ../c/coreslam.h
	gcc -O3 -c -Wall $(CFLAGS) ../c/coreslam.c

coreslam_$(ARCH).o: ../c/coreslam_$(ARCH).c ../c/coreslam.h
	gcc -O3 -c -Wall $(CFLAGS) $(SIMD_FLAGS) ../c/coreslam_$(ARCH).c

random.o: ../c/random.c
	gcc -O3 -c -Wall $(CFLAGS) ../c/random.c
	
ziggurat.o: ../c/ziggurat.c
	gcc -O3 -c -Wall $(CFLAGS) ../c/ziggurat.c
	
install: libbreezyslam.$(LIBEXT)
	cp libbreezyslam.$(LIBEXT) $(LIBDIR)
	
doc:
	doxygen

clean:
	rm -rf  libbreezyslam.$(LIBEXT) *.o Documentation \#* *~
