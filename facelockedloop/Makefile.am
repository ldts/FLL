bin_PROGRAMS = fll

fll_SOURCES = \
	main.c	\
	pipeline.c \
	pipeline.h \
	capture.c \
	capture.h \
	detect.c \
	detect.h \
	track.c	\
	track.h \
	store.h

fll_CPPFLAGS =		\
	@FLL_CFLAGS@ @FLL_EXTRA_CFLAGS@	\
	-I$(top_srcdir)/include		\
	-I$(top_builddir)/servolib/lib


fll_LDFLAGS = @FLL_LDFLAGS@

fll_LDADD =		\
	../servolib/libservolib.la \
	-lpthread -lrt

fll_CPPFLAGS +=  @opencvinc@ -DHAVE_OPENCV2
fll_LDFLAGS +=  @opencvlib@
fll_LDFLAGS += -lm
fll_LDADD += @OPENCV_ADD_LDFLAG@
fll_LDADD += -lm


