# Makefile for BreezySLAM demos
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

# Where you put the libbreezyslam library
LIBDIR = /usr/local/lib

# Use EOG or your favorite image-display program
VIEWER = eog

# Set these for different experiments
DATASET = exp2
USE_ODOMETRY = 1
RANDOM_SEED  = 9999

all: log2pgm

pytest: 
	./log2pgm.py $(DATASET) $(USE_ODOMETRY) $(RANDOM_SEED)
	$(VIEWER) $(DATASET).pgm ~/Desktop/$(DATASET).pgm

cpptest: log2pgm 
	./log2pgm $(DATASET) $(USE_ODOMETRY) $(RANDOM_SEED)
	$(VIEWER) $(DATASET).pgm

log2pgm: log2pgm.o 
	g++ -O3 -o log2pgm log2pgm.o -L$(LIBDIR) -lbreezyslam

log2pgm.o: log2pgm.cpp 
	g++ -O3 -c -I ../cpp log2pgm.cpp

$(DATASET).pgm:
	./log2pgm.py $(DATASET) $(USE_ODOMETRY) $(RANDOM_SEED)

backup:
	cp -r .. ~/Documents/slam/bak-breezyslam

clean:
	rm -f log2pgm *.pyc *.pgm *.o *~
