ifeq ($(SRCDIR),)
include $(MAKEFILE_DIR)/target.mk
else

LIB_NAME := $(C2X_SDK_GEN_LIB_DIR)/libMmitss.so
VPATH := $(SRCDIR)

INC_DIRS += $(TOPDIR)/mmitss/lib \
            $(TOPDIR)/mmitss/include \
            $(TOPDIR)/mmitss/libSieMmitss

SRCS := GeoCoord.cpp \
        Signal.cpp \
        Config.cpp \
        Array.cpp \
        Mib.cpp \
        ReqEntry.cpp \
        mmitssUtils.cpp \
        EVLS.cpp \
        ReqEntryListHandle.cpp

LD_LIBS += -lPocoFoundation -lPocoNet -lnetsnmp -lSieMmitss

include $(MAKEFILE_DIR)/rules.mk
include $(MAKEFILE_DIR)/facilities.mk

prep:
	cd $(SRCDIR) && git diff --relative --diff-filter=AMX --name-only HEAD "*.[ch]pp" | xargs -r -d '\n' clang-format -i

build: $(LIB_NAME)

$(LIB_NAME): $(OBJS) $(SRCDIR)/Makefile
	@echo "  LD    $(notdir $@)"
	$(Q)$(CXX) $(LD_FLAGS) -shared -o $@ -Wl,--start-group $(OBJS) -Wl,--end-group $(LD_LIBS)

install::
	$(Q)install -m 0755 $(LIB_NAME) $(INSTALL_LIB_DIR)

endif
