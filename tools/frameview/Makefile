.SUFFIXES: .C $(SUFFIXES)

CXX		= g++
CXXINCLUDE	= -I/usr/X11R6/include
#		-I ../components 

#CXXINCLUDE	= -I/sw/share/X11R6.4/include
CXXLIBS		= -lm -L/usr/X11R6/lib -lX11
#		-L$(HOME)/lib  -lcomponents

# O1 ... 03 optimization levels for g++
#CXXFLAGS	= -Wall -O1

#-pg for profiling information (must also be set for linking)
#CXXFLAGS	= -pg -Wall
CXXFLAGS       = -Wall


#SRCS		= $(wildcard *.C)
SRCS		= 2dview.C area2d.C conv.C display_x11.C frame2d.C rgbcolor.C structure.C visobject.C\
		  rgb_db.C smonitor_dev.C ascii_dev.C joystick_dev.C builder_direct.C ascii_processor.C \
		  menu_x11.C tools.C conv_area2d.C popup_x11.C \
		angle.C vector2d.C valueparser.C str2val.C udpsocket.C tcpsocket.C 

OBJS		=  $(SRCS:.C=.o) 
#OBJS		= 2dview.o  angle.o area2d.o builder.o coder.o conv.o  drawdisplay.o  frame2d.o rgbcolor.o structure.o  vector2d.o visobject.o rgb_db.o pendulum_dev.o smonitor_dev.o

THIS_DIR		:= $(PWD)

TARGET		= frameview
$(TARGET): $(OBJS) 
#	$(CXX) -pg -o $(TARGET) $(OBJS) $(CXXLIBS) 
	$(CXX) -o $(TARGET) $(OBJS) $(CXXLIBS)
	cd ../../bin; ln -sf $(THIS_DIR)/$(TARGET) .

structure: structure.o angle.o vector2d.o frame2d.o
	$(CXX) -o $@ $^ $(CXXLIBS)

coder: coder.o angle.o vector2d.o frame2d.o
	$(CXX) -o $@ $^ $(CXXLIBS)

Rgbcolor: rgbcolor.o 
	$(CXX) -o $@ $^ $(CXXLIBS)

client2dview: client2dview.o coder.o  
	$(CXX) -o $@ $^ $(CXXLIBS)

pipe2dview: pipe2dview.o  
	$(CXX) -o $@ $^ 

Smonitor_dev: smonitor_dev.C
	$(CXX) -c $^  $(CXXFLAGS) $(CXXINCLUDE)

Draw_ps: draw_ps.o rgbcolor.o angle.o
	$(CXX) -o $@ $^ $(CXXLIBS)

conv_area2d: conv_area2d.o area2d.o
	$(CXX) -o $@ $^ $(CXXLIBS)

distribution : $(wildcard *.C) $(wildcard *.h) Doxyfile Makefile ../components/angle.C ../components/angle.h ../components/vector2d.C ../components/vector2d.h ../components/valueparser.C ../components/valueparser.h ../components/str2val.C ../components/str2val.h ../components/udpsocket.C ../components/udpsocket.h ../components/tcpsocket.C ../components/tcpsocket.h ../components/rwbin.h COPYING.TXT FAQ.TXT INSTALL.TXT README.TXT CHANGES-AFTER-1.0 Frameview Frameview2  colors.fv robocup_middlesize.fv haartree.fv
	mkdir ../frameview-1.5
	cp  $^ ../frameview-1.5


#erzeugt Abhaengigkeiten und schreibt die in Dateien mit Endung *.d
%.d: %.C
	@$(SHELL) -ec '$(CXX) -MM $(CXXFLAGS) $(CXXINCLUDE) $< \
                 | sed '\''s/\($*\)\.o[ :]*/\1.o $@: /g'\'' > $@'
	@(echo "   $@ generated")

#beachte alle Abhaengigkeitsdateien
-include $(SRCS:.C=.d)

%.o: %.C	
	$(CXX) -c $(CXXFLAGS) $(CXXINCLUDE) $<

clean:
	rm -f $(OBJS) $(SRCS:.C=.d) $(TARGET) *~ 


