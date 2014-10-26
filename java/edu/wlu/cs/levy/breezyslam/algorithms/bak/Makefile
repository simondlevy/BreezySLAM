JAVADIR = ../../../../../..

ALL = CoreSLAM.class

all: $(ALL)

CoreSLAM.class: CoreSLAM.java
	javac -classpath $(JAVADIR):. CoreSLAM.java

clean:
		rm -f *.class *~

backup:
		cp *.java bak
		cp Makefile bak
