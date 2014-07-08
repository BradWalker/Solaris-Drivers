#
#	file: Makefile for serial/parallel cards
#	sccs info: @(#)Makefile	1.13 20 May 1993 1
#

CC =
DRVFLGS = -DKERNEL
CFLAGS =
CWD = $$PWD

INFO_SOURCES	= pioinfo.c sioinfo.c
INFO_OBJECTS	= $(INFOSOURCES:.c=)

DRV_SOURCES	= xsio.c xpio.c
DRV_OBJECTS	= $(DRV_SOURCES:.c=)

XXTRACE_SRC	= event.h xxtrace.h xxtracelib.o

default:
	echo " You must type in S or SJ"

S:	xsio xpio pkgS
	echo " Done and built"

SJ:	sio pio pkgS
	echo " Done and built"

VERSION: VERSION
	@ echo "********* VERSION file is missing!!! *********"
	@ echo 999 >VERSION

mset:	mset.c
	cc -o mset $(CFLAGS) mset.c

source:	sourceme

allinfo:	$(INFOOBJECTS)

event.h:	Devent event.in
		@ chmod 755 Devent
		@ rm -f event.h
		Devent event.in event.h

xxtrace.h:
		@ echo "Building $@"
		@ if [ ! -d xxtrace_svr4 ] ; then /vol/aurora/bin/linksccs xxtrace_svr4; fi
		@ (cd xxtrace_svr4; make) > /dev/null
		@ ln -s xxtrace_svr4/xxtrace.h

xxtracelib.o:
		@ echo "Building $@"
		@ if [ ! -d xxtrace_svr4 ] ; then /vol/aurora/bin/linksccs xxtrace_svr4; fi
		@ (cd xxtrace_svr4; make) > /dev/null
		@ ln -s xxtrace_svr4/xxtracelib.o

pioinfo: pioinfo.c
	cc $(TARGET_ARCH) -o pioinfo pioinfo.c

sioinfo: sioinfo.c
	cc $(TARGET_ARCH) -o sioinfo sioinfo.c

xpio.ed.c: pio.c
	rm -f xpio.ed.c
	D xpio xpio.ed.c pio.c

xpio:	xpio.ed.c event.h pio.h xxtrace.h xxtracelib.o
	cc -O -DBOARDS -D_KERNEL -DSUNDDI -I. -I/usr/share/src/uts/sun4c -c -o xpio.o xpio.ed.c
	ld -r -o xpio xpio.o

xpioxx:	xpio.ed.c event.h pio.h xxtrace.h xxtracelib.o
	cc -O -DBOARDS -D_KERNEL -DSUNDDI -DXXTRACE -I. -I/usr/share/src/uts/sun4c -c -o xpioxx.o xpio.ed.c
	ld -r -o xpioxx xpioxx.o xxtracelib.o

xsio.ed.c: sio.c
	rm -f xsio.ed.c
	D xsio xsio.ed.c sio.c

xsio:	xsio.ed.c event.h sio.h xxtrace.h xxtracelib.o
	cc -O -DBOARDS -D_KERNEL -DSUNDDI -I. -I/usr/share/src/uts/sun4c -c -o xsio.o xsio.ed.c
	ld -r -o xsio xsio.o

xsioxx:	xsio.ed.c event.h sio.h xxtrace.h xxtracelib.o
	cc -O -DBOARDS -D_KERNEL -DSUNDDI -DXXTRACE -I. -I/usr/share/src/uts/sun4c -c -o xsioxx.o xsio.ed.c
	ld -r -o xsio xsio.o xxtracelib.o

pio.ed.c: pio.c
	rm -f pio.ed.c
	D pio pio.ed.c pio.c

pio:	pio.ed.c event.h pio.h xxtrace.h xxtracelib.o
	cc -O -DBOARDSJ -D_KERNEL -DSUNDDI -I. -I/usr/share/src/uts/sun4c -c -o pio.o pio.ed.c
	ld -r -o pio pio.o

pioxx:	pio.ed.c event.h pio.h xxtrace.h xxtracelib.o
	cc -O -DBOARDSJ -D_KERNEL -DSUNDDI -DXXTRACE -I. -I/usr/share/src/uts/sun4c -c -o pioxx.o pio.ed.c
	ld -r -o pioxx pioxx.o xxtracelib.o

sio.ed.c: sio.c
	rm -f sio.ed.c
	D sio sio.ed.c sio.c
	
sio:	sio.ed.c event.h sio.h xxtrace.h xxtracelib.o
	cc -O -DBOARDSJ -D_KERNEL -DSUNDDI -I. -I/usr/share/src/uts/sun4c -c -o sio.o sio.ed.c
	ld -r -o sio sio.o

sioxx:	sio.ed.c event.h sio.h xxtrace.h xxtracelib.o
	cc -O -DBOARDSJ -D_KERNEL -DSUNDDI -DXXTRACE -I. -I/usr/share/src/uts/sun4c -c -o sioxx.o sio.ed.c
	ld -r -o sioxx sioxx.o xxtracelib.o

420sjprom: 420sj.fth
	tokenize 420sj.fth
	dd < 420sj.fcode > 420sj.stripped bs=32 skip=1
	mv 420sj.stripped 420sj.fco

210sjprom: 210sj.fth
	tokenize 210sj.fth
	dd < 210sj.fcode > 210sj.stripped bs=32 skip=1
	mv 210sj.stripped 210sj.fco

210prom: 210s.fth
	tokenize 210s.fth
	dd < 210s.fcode > 210s.stripped bs=32 skip=1
	mv 210s.stripped 210s.fco

sj.tar: $(%=REL/AURAsp%)
	rm -f sj.tar
	(cd REL; tar cvf ../sj.tar AURAsp)

s.tar: $(%=REL/AURAxsp%)
	rm -f s.tar
	(cd REL; tar cvf ../s.tar AURAxsp)

10S:	D.10S
	rm -f D
	cp D.10S D
	chmod +x D
	rm -f MODEL
	echo 10S > MODEL

210S:	D.210S
	rm -f D
	cp D.210S D
	chmod +x D
	rm -f MODEL
	echo 210S > MODEL

10SJ:	D.10SJ
	rm -f D
	cp D.10SJ D
	chmod +x D
	rm -f MODEL
	echo 10SJ > MODEL

210SJ:	D.210SJ
	rm -f D
	cp D.210SJ D
	chmod +x D
	rm -f MODEL
	echo 210SJ > MODEL

420SJ:	D.420SJ
	rm -f D
	cp D.420SJ D
	chmod +x D
	rm -f MODEL
	echo 420SJ > MODEL

REL/%: %
	rm -f REL/$*
	cp $* REL
	rsh -l root auratek chown -fR root.daemon $(CWD)/REL/$*

lint:
	lint $(CFLAGS) $(DRVFLGS) xsio.ed.c > lint.out
	lint $(CFLAGS) $(DRVFLGS) xpio.ed.c >> lint.out

pkginfo:	pkginfo_proto
		rm -f pkginfo
		D Makefile pkginfo pkginfo_proto

devlink.tab:	devlink.tab_proto
		rm -f devlink.tab
		D Makefile devlink.tab devlink.tab_proto

iu.ap:		iu.ap_proto
		rm -f iu.ap
		D Makefile iu.ap iu.ap_proto

path_to_inst:	path_to_inst_proto
		rm -f path_to_inst
		D Makefile path_to_inst path_to_inst_proto

postinstall:	postinstall_proto
		rm -f postinstall
		D Makefile postinstall postinstall_proto

preremove:	preremove_proto
		rm -f preremove
		D Makefile preremove preremove_proto

prototype:	prototype_proto
		rm -f prototype
		D Makefile prototype prototype_proto

system:		system_proto
		rm -f system
		D Makefile system system_proto

pkgS:	system iu.ap postinstall path_to_inst devlink.tab xsio xpio copyright pkginfo preremove Readme xpioxx xsioxx
	@ rm -rf REL etc kernel AURAxsp
	@ mkdir -p AURAxsp etc kernel/drv REL
	@ cp xsioxx xpioxx pio.h sio.h Readme AURAxsp
	@ cp xsio xpio kernel/drv
	@ cp path_to_inst devlink.tab iu.ap system etc
	pkgmk -o -r `pwd` -d `pwd`/REL
	pkgtrans -s `pwd`/REL /dev/diskette AURAxsp
	dd bs=18k if=/dev/rfd0c of=REL/AURAxsp.disk.image

pkgSJ:	system iu.ap postinstall path_to_inst devlink.tab xsio xpio copyright pkginfo preremove Readme sioxx pioxx
	@ rm -rf REL etc kernel AURAsp
	@ mkdir -p AURAsp etc kernel/drv REL
	@ cp sioxx pioxx mset pio.h sio.h Readme AURAsp
	@ cp sio pio kernel/drv
	@ cp path_to_inst devlink.tab iu.ap system etc
	pkgmk -o -r `pwd` -d `pwd`/REL
	pkgtrans -s `pwd`/REL /dev/diskette AURAsp
	dd bs=18k if=/dev/rfd0c of=REL/AURAsp.disk.image

