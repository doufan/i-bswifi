LDFLAGS += -L$(ROOTDIR)/lib/libnvram -lnvram
LDFLAGS += -L$(ROOTDIR)/user/rosCommon -lroscommon 
LDFLAGS += -L$(ROOTDIR)/user/rosPlatformCommon -lrosplatform -lpthread
CFLAGS += -I$(ROOTDIR)/lib/libnvram
CFLAGS += -I$(ROOTDIR)/user/rosCommon
CFLAGS += -I$(ROOTDIR)/user/rosPlatformCommon
all: uartBT

uartBT: uartBT.o
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lpthread

romfs:
	$(ROMFSINST) /bin/uartBT

clean:
	-rm -f uartBT *.o

