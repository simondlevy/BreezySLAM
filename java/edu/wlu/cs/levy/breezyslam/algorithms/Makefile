# Makefile for BreezySLAM algorithms in Java
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

# Set SIMD compile params based on architecture
ifeq ("$(ARCH)","armv7l")
  SIMD_FLAGS = -mfpu=neon
else ifeq ("$(ARCH)","i686")
  SIMD_FLAGS = -msse3
else
  ARCH = sisd
endif


ALL = libjnibreezyslam_algorithms.$(LIBEXT) CoreSLAM.class SinglePositionSLAM.class DeterministicSLAM.class RMHCSLAM.class

all: $(ALL)

libjnibreezyslam_algorithms.$(LIBEXT): jnibreezyslam_algorithms.o coreslam.o random.o ziggurat.o coreslam_$(ARCH).o
	gcc -shared -Wl,-soname,libjnibreezyslam_algorithms.so -o libjnibreezyslam_algorithms.so jnibreezyslam_algorithms.o \
	            coreslam.o coreslam_$(ARCH).o random.o ziggurat.o

jnibreezyslam_algorithms.o: jnibreezyslam_algorithms.c RMHCSLAM.h ../jni_utils.h
	gcc $(JDKINC) -fPIC -c jnibreezyslam_algorithms.c


CoreSLAM.class: CoreSLAM.java
	javac -classpath $(JAVADIR):. CoreSLAM.java


SinglePositionSLAM.class: SinglePositionSLAM.java
	javac -classpath $(JAVADIR):. SinglePositionSLAM.java

DeterministicSLAM.class: DeterministicSLAM.java
	javac -classpath $(JAVADIR):. DeterministicSLAM.java

RMHCSLAM.class: RMHCSLAM.java
	javac -classpath $(JAVADIR):. RMHCSLAM.java

RMHCSLAM.h: RMHCSLAM.class
	javah -o RMHCSLAM.h -classpath $(JAVADIR) -jni edu.wlu.cs.levy.breezyslam.algorithms.RMHCSLAM

coreslam.o: $(CDIR)/coreslam.c $(CDIR)/coreslam.h
	gcc -O3 -c -Wall $(CFLAGS) $(CDIR)/coreslam.c

coreslam_$(ARCH).o: $(CDIR)/coreslam_$(ARCH).c $(CDIR)/coreslam.h
	gcc -O3 -c -Wall $(CFLAGS) $(SIMD_FLAGS) $(CDIR)/coreslam_$(ARCH).c

random.o: $(CDIR)/random.c
	gcc -O3 -c -Wall $(CFLAGS) $(CDIR)/random.c
	
ziggurat.o: $(CDIR)/ziggurat.c
	gcc -O3 -c -Wall $(CFLAGS) $(CDIR)/ziggurat.c
	
clean:
		rm -f *.class *.o *.h *.$(LIBEXT) *~

backup:
		cp *.java bak
		cp Makefile bak
