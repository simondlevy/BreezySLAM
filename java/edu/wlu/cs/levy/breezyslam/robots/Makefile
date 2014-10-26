BASEDIR = ../../../../../../..
JAVADIR = $(BASEDIR)/java
JFLAGS = -Xlint

ALL = WheeledRobot.class

all: $(ALL)

WheeledRobot.class: WheeledRobot.java
	javac $(JFLAGS) -classpath $(JAVADIR) WheeledRobot.java

clean:
		rm -f *.class *~
