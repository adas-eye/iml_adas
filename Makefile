# -------------------------------------------------
# Variables setting
# -------------------------------------------------
SYSTEM = $(shell uname)
VERSION = $(shell  uname -a | awk '{print $$(NF-3)}')
OPT = release
#OPT = debug
CXX = g++
TARGET = iml_adas

INCDIR=	\
		-I./inc \
		-I/usr/include \
		-I/usr/include/opencv \
		-I/usr/include/opencv2 \
		-I/usr/local/include \
		-I/usr/local/include/opencv \
		-I/usr/local/include/opencv2 \
		-I../install/opencv_lib/include \
		-I../install/opencv_lib/include/opencv \
		-I../install/opencv_lib/include/opencv2

SRCDIR=	\
		src

LIBDIR=	\
		-L/usr/local/cuda/lib64 \
		-L/usr/local/jpegdev/lib \
		-L../install/opencv_lib/lib \
		-lopencv_core -lopencv_highgui -lopencv_calib3d -lopencv_imgproc -lopencv_objdetect \
		-lpthread -lm -lstdc++

OBJDIR = obj.$(SYSTEM).$(OPT).$(CXX)
SRC = $(wildcard $(SRCDIR)/*.cpp)
OBJ = $(patsubst $(SRCDIR)%.cpp,$(OBJDIR)%.o,$(SRC))
$(shell if [ ! -d ${OBJDIR} ]; then mkdir ${OBJDIR}; fi)

ifeq ($(VERSION), armv7l)
	GCC = arm-linux-gnueabihf-g++-4.6
$(shell cp lib/libmodel_core.a.arm lib/libmodel_core.a;\
	cp src/loadCascade.cpp.win src/loadCascade.cpp)
else ifeq ($(VERSION), PDT)
	GCC = /usr/bin/g++
$(shell cp lib/libmodel_core.a.mac lib/libmodel_core.a;\
	cp src/loadCascade.cpp.ctos src/loadCascade.cpp)
else
	GCC = /usr/bin/g++
$(shell cp lib/libmodel_core.a.ctos lib/libmodel_core.a;\
	cp src/loadCascade.cpp.ctos src/loadCascade.cpp)
endif

ifeq ($(OPT), release)
	CPPFLAGS = -c -fpic -O3 -DNDEBUG  -DPPLM_FLOAT -DPYTHON_HEAD_FILE_INCLUDE
endif
ifeq ($(OPT), debug)
	CPPFLAGS = -c -fpic -g -DNDEBUG  -DPPLM_FLOAT -DPYTHON_HEAD_FILE_INCLUDE
endif

# -------------------------------------------------
# Code generation
# -------------------------------------------------
bin : $(TARGET)

$(TARGET) : ${OBJ}
	ar -x lib/libmodel_core.a
	$(GCC) -g -o $@ *.o $^ $(LIBDIR) $(INCDIR)
	rm *.o __.SYMDEF*

${OBJDIR}/%.o : $(SRCDIR)/%.cpp
	$(GCC) $(CPPFLAGS) -c $< -o $@ $(INCDIR)

clean:
	rm -rf ${OBJDIR} *.o $(TARGET) __.SYMDEF*
