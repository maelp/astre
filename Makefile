CC=gcc
LIBS=-lm -largtable2 -lcbase
CFLAGS=-I include/ --std=gnu99 -O3 -funroll-loops -ffunction-sections -fdata-sections -fexpensive-optimizations

IDIR=include/vision/
_VISION_INCLUDES=core.h mini_megawave.h formats/descfile.h math/base.h math/combinatorics.h trajs/pointsdesc.h trajs/trajs.h utils/argparser.h utils/datastructures.h utils/dllist.h utils/string.h
VISION_INCLUDES=$(patsubst %,$(IDIR)/%,$(_VISION_INCLUDES))

_VISION_OBJS=core.o mini_megawave.o formats/descfile.o math/combinatorics.o trajs/pointsdesc.o trajs/trajs.o utils/argparser.o utils/datastructures.o utils/string.o
VISION_OBJS=$(patsubst %,src/vision/%,$(_VISION_OBJS))

BINS=astre_naive.py astre-noholes astre-holes tpsmg tcripple tstats tview.py

all: $(patsubst %,bin/%,$(BINS))

$(ODIR)/%.o: src/vision/%.c $(VISION_INCLUDES)
	$(CC) -c -o $@ $< $(CFLAGS)

bin/astre_naive.py: src/astre/astre_naive.py
	ln -s ../src/astre/astre_naive.py $@
bin/tview.py: utils/tview.py
	ln -s ../utils/tview.py $@
bin/astre-noholes: src/astre/astre-common-code.h src/astre/astre-common-defs.h src/astre/astre.c $(VISION_OBJS)
	$(CC) -o $@ -D ASTRE_HAS_NO_HOLES $(CFLAGS) src/astre/astre.c $(VISION_OBJS) $(LIBS) 
bin/astre-holes: src/astre/astre-common-code.h src/astre/astre-common-defs.h src/astre/astre.c $(VISION_OBJS)
	$(CC) -o $@ -D ASTRE_HAS_HOLES $(CFLAGS) src/astre/astre.c $(VISION_OBJS) $(LIBS) 
bin/%: src/astre/%.c $(VISION_OBJS)
	$(CC) -o $@ $(CFLAGS) $(VISION_OBJS) $(LIBS) $<

clean:
	rm -f $(VISION_OBJS)
	rm -f include/vision/**/.*~
	rm -f src/vision/**/.*~
	rm -f src/astre/**/.*~

