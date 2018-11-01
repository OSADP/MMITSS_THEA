ifeq ($(SRCDIR),)
include $(MAKEFILE_DIR)/target.mk
else

APP_NAME:=$(C2X_SDK_GEN_TARGET_DIR)/mmitss_perf_observer

INC_DIRS += $(TOPDIR)/mmitss/lib \
            $(TOPDIR)/mmitss/include \
            $(TOPDIR)/mmitss/libSieMmitss

SRCS := mmitssPerformanceObserverApp.cpp \
        mmitssPerformanceObserver.cpp

LD_FLAGS += -L$(TOPDIR)/mmitss/PerformanceObserver
LD_LIBS +=  -lWaveApp -lPocoFoundation -lPocoNet -lPocoJSON -lSieMmitss -lMmitss -lnetsnmp

include $(MAKEFILE_DIR)/rules.mk
include $(MAKEFILE_DIR)/facilities.mk

prep:
	cd $(SRCDIR) && git diff --relative --diff-filter=AMX --name-only HEAD "*.[ch]pp" | xargs -r -d '\n' clang-format -i

build: $(APP_NAME)

$(APP_NAME): $(OBJS) Makefile
	@echo "  LD    $(notdir $@)"
	$(Q)$(CXX) $(LD_FLAGS) -o $@ -Wl,--start-group $(OBJS) $(LD_LIBS) -Wl,--end-group $(LD_LIBS_CORE)

install::
	$(Q)install -m 0755 $(APP_NAME) $(INSTALL_BIN_DIR)
	$(Q)install -m 0644 $(SRCDIR)/mmitss_perf_observer.service $(INSTALL_SYSTEMD_DIR)/

endif
