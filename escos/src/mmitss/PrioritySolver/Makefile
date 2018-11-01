ifeq ($(SRCDIR),)
include $(MAKEFILE_DIR)/target.mk
else

APP_NAME:=$(C2X_SDK_GEN_TARGET_DIR)/mmitss_prio_solver

INC_DIRS += $(TOPDIR)/../../thirdparty/glpk-4.55/esc/include \
            $(TOPDIR)/mmitss/lib \
            $(TOPDIR)/mmitss/include \
            $(TOPDIR)/mmitss/libSieMmitss \
            $(TOPDIR)/common

SRCS := mmitssPrioritySolverApp.cpp \
        mmitssPrioritySolver.cpp \
        PriorityRequest.cpp

LD_FLAGS += -L$(TOPDIR)/mmitss/PrioritySolver -L$(TOPDIR)/common
LD_LIBS +=  -lPocoFoundation -lPocoNet -lPocoJSON -lSieMmitss -lMmitss -lWaveApp -lglpk -lnetsnmp

ifeq ($(DEVICE),esc)
LD_FLAGS += -L$(TOPDIR)/../../thirdparty/glpk-4.55/esc/lib
GLPK_LIB = $(TOPDIR)/../../thirdparty/glpk-4.55/esc/lib/libglpk.so.36
endif

ifeq ($(DEVICE),kvm)
LD_FLAGS += -L$(TOPDIR)/../../thirdparty/glpk-4.55/kvm/lib
GLPK_LIB = $(TOPDIR)/../../thirdparty/glpk-4.55/kvm/lib/libglpk.so.36
endif

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
	$(Q)install -m 0644 $(SRCDIR)/mmitss_prio_solver.service $(INSTALL_SYSTEMD_DIR)
	$(Q)install -m 0755 $(GLPK_LIB) $(INSTALL_LIB_DIR)

endif
