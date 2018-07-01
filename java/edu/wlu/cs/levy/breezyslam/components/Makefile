# Makefile for BreezySLAM components in Java
#
# Copyright (C) 2014 Simon D. Levy
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

# You should have received a copy of the GNU Lesser General Public License 
# along with this code.  If not, see <http://www.gnu.org/licenses/>.

BASEDIR = ../../../../../../..
JAVADIR = $(BASEDIR)/java
CDIR    = $(BASEDIR)/c
JFLAGS = -Xlint
JDKINC = -I /usr/lib/jvm/java-7-openjdk-amd64/include
#JDKINC = -I /opt/jdk1.7.0_67/include -I /opt/jdk1.7.0_67/include/linux

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

ALL = libjnibreezyslam_components.$(LIBEXT) Laser.class Position.class PoseChange.class URG04LX.class

all: $(ALL)

libjnibreezyslam_components.$(LIBEXT): jnibreezyslam_components.o coreslam.o coreslam_$(ARCH).o
	gcc -shared -Wl,-soname,libjnibreezyslam_components.so -o libjnibreezyslam_components.so jnibreezyslam_components.o \
	            coreslam.o coreslam_$(ARCH).o

jnibreezyslam_components.o: jnibreezyslam_components.c Map.h Scan.h ../jni_utils.h
	gcc $(JDKINC) -fPIC -c jnibreezyslam_components.c

coreslam.o: $(CDIR)/coreslam.c $(CDIR)/coreslam.h
	gcc -O3 -c -Wall $(CFLAGS) $(CDIR)/coreslam.c

coreslam_$(ARCH).o: $(CDIR)/coreslam_$(ARCH).c $(CDIR)/coreslam.h
	gcc -O3 -c -Wall $(CFLAGS) $(SIMD_FLAGS) $(CDIR)/coreslam_$(ARCH).c

Map.h: Map.class
	javah -o Map.h -classpath $(JAVADIR) -jni edu.wlu.cs.levy.breezyslam.components.Map

Map.class: Map.java Scan.class Position.class
	javac $(JFLAGS) -classpath $(JAVADIR) Map.java

Scan.h: Scan.class
	javah -o Scan.h -classpath $(JAVADIR) -jni edu.wlu.cs.levy.breezyslam.components.Scan

Scan.class: Scan.java Laser.class
	javac $(JFLAGS) -classpath $(JAVADIR) Scan.java

Laser.class: Laser.java
	javac $(JFLAGS) Laser.java

URG04LX.class: URG04LX.java Laser.class
	javac $(JFLAGS) -classpath $(JAVADIR) URG04LX.java

PoseChange.class: PoseChange.java
	javac $(JFLAGS) PoseChange.java


Position.class: Position.java
	javac $(JFLAGS) Position.java

clean:
		rm -f *.so *.class *.o *.h *~
