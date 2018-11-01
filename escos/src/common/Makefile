ifeq ($(SRCDIR),)
include $(MAKEFILE_DIR)/target.mk
else

LIB_NAME := $(C2X_SDK_GEN_LIB_DIR)/libWaveApp.so
VPATH := $(SRCDIR)

SRCS := asnHelper.cpp \
        base64.cpp \
        cmdLineParser.cpp \
        facilityWaveSae.cpp \
        fileUtil.cpp \
        timeUtil.cpp \
        udpReceiver.cpp

LD_LIBS += -lPocoFoundation

include $(MAKEFILE_DIR)/rules.mk
include $(MAKEFILE_DIR)/facilities.mk

prep:
	cd $(SRCDIR) && git diff --relative --diff-filter=AMX --name-only HEAD "*.[ch]pp" | xargs -r -d '\n' clang-format -i

build: $(LIB_NAME)

$(LIB_NAME): $(OBJS) $(SRCDIR)/Makefile $(SRCDIR)/unittest.cpp
	@echo "  LD    $(notdir $@)"
ifeq ($(C2X_UNIT_TESTS),y)
	$(Q)$(CXX) $(LD_FLAGS) -shared -pie -o $@ -Wl,--start-group $(SRCDIR)/unittest.cpp $(OBJS) -Wl,--end-group $(LD_LIBS) -Wl,-E
else
	$(Q)$(CXX) $(LD_FLAGS) -shared -o $@ -Wl,--start-group $(OBJS) -Wl,--end-group $(LD_LIBS)
endif

install::
	$(Q)install -m 0755 $(LIB_NAME) $(INSTALL_LIB_DIR)

endif
