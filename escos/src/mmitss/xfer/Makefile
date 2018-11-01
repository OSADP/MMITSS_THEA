ifeq ($(SRCDIR),)
include $(MAKEFILE_DIR)/target.mk
else

LIB_NAME := $(C2X_SDK_GEN_LIB_DIR)/libXferMmitss.so
VPATH := $(SRCDIR)

SRCS := mmitssCommand.cpp

include $(MAKEFILE_DIR)/rules.mk
include $(MAKEFILE_DIR)/sigcxx.mk

LD_LIBS += -lPocoJSON -lxfer

prep:
	cd $(SRCDIR) && git diff --relative --diff-filter=AMX --name-only HEAD "*.[ch]pp" | xargs -r -d '\n' clang-format -i

build: $(LIB_NAME)

$(LIB_NAME): $(OBJS) $(SRCDIR)/Makefile
	@echo "  LD    $(notdir $@)"
	$(Q)$(CXX) $(LD_FLAGS) -shared -o $@ -Wl,--start-group $(OBJS) -Wl,--end-group $(LD_LIBS)

install::
	$(Q)install -m 0755 $(LIB_NAME) $(INSTALL_LIB_DIR)

endif
