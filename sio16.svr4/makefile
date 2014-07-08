# @(#)$Header: /vol/sources.cvs/dev/sio16.svr4/makefile,v 1.1 1996/09/20 21:27:47 bkd Exp $
default: $(OS)/.depend
	cd $(OS); make

$(OS)/.depend: m.$(OS)
	test -d $(OS) || mkdir $(OS)
	cp m.$(OS) $(OS)/makefile
	cd $(OS); make depends
	touch $(OS)/.depend

release: $(OS)/.depend
	cd $(OS); make release

transfer: $(OS)/.depend
	cd $(OS); make transfer

image: $(OS)/.depend
	cd $(OS); make image

id: $(OS)/.depend
	cd $(OS); make id

clean:
	rm -rf $(OS)
