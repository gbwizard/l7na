CFLAGS       := $(CFLAGS) -static -ggdb3 -std=gnu99 -O0 -fno-inline
ETHERCAT_DIR = ../ethercat/
LIBS         = ethercat
ALL_TARGETS  = servotech servotech3 klm

all: $(ALL_TARGETS)

servotech: main.c
	gcc $(CFLAGS) $^ -I $(ETHERCAT_DIR)/include/ -L $(ETHERCAT_DIR)/lib/.libs/ -l $(LIBS) -o $@

servotech3: main3.c
	gcc $(CFLAGS) $^ -I $(ETHERCAT_DIR)/include/ -L $(ETHERCAT_DIR)/lib/.libs/ -l $(LIBS) -o $@

klm: klm.c
	gcc $(CFLAGS) $^ -I $(ETHERCAT_DIR)/include/ -L $(ETHERCAT_DIR)/lib/.libs/ -l $(LIBS) -o $@

.PHONY: clean
clean:
	rm -f $(ALL_TARGETS)
