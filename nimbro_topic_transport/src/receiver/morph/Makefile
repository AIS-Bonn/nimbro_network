
ROS_INCLUDES := -I/opt/ros/melodic/include
CFLAGS := -fPIC -std=c++17 $(ROS_INCLUDES)

DEPFLAGS = -MT $@ -MMD -MP -MF $*.d

COMPILE.c = $(CXX) $(DEPFLAGS) $(CFLAGS) $(CPPFLAGS) -c
SRCS = rewrite_header.cpp

%.o: %.cpp
%.o: %.cpp %.d
	$(COMPILE.c) $(OUTPUT_OPTION) $<

librewrite_header.so: rewrite_header.o
	g++ -shared $< -o $@

all: librewrite_header.so

DEPFILES := $(SRCS:%.cpp=%.d)
$(DEPFILES):
include $(wildcard $(DEPFILES))
