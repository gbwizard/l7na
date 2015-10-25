export NCPU		?= $(shell (grep processor /proc/cpuinfo || echo ?) | wc -l)
export NJOB		?= $(NCPU)

include reporter.mk

.PHONY: default
default: debug

TOP_DIR		:= $(shell readlink -f $(dir $(lastword $(MAKEFILE_LIST))))
CMAKE_OPTIONS += -Wdev

.PHONY: all
all: release

define rule

@BUILD-$(1)/Makefile:
	@$(call do_begin,,cmake-$(1))
	@mkdir -p $$(@D) && cd $$(@D) && \
		cmake $(CMAKE_OPTIONS) -DCMAKE_BUILD_TYPE="$(1)" .. \
		|| $(call do_failure,,cmake-$(1),CMake execution error!)
	@$(call do_done,,cmake-$(1))

# LY: цель test = прогоняет все тесты определенные в cmake
.PHONY: $(1)/test
$(1)/test: $(1)/version $(1)/all
	@$(call do_begin,,$$@)
	@GTEST_OUTPUT=xml:$$$$(readlink -f @BUILD-$(1)/Testing)/ \
		ARGS="--schedule-random --output-on-failure -j $(NJOB)" \
		$(MAKE) -j $(NJOB) -C @BUILD-$(1) test \
		&& $(call do_junit,,$$@,@BUILD-$(1)/Testing/*.xml) \
	|| { $(call do_junit,,$$@,@BUILD-$(1)/Testing/*.xml) ; $(call do_failure,,$$@,Unit tests failed!,) ; }
	@$(call do_done,,$$@,,)

.PHONY: $(1)
$(1): $(1)/all

$(1)/%: @BUILD-$(1)/Makefile
	@$(call do_begin,,build-$(1)-$$*)
	@+PATH=$(PATH) $(MAKE_PREFIX_CMD) $(MAKE) -j $(NJOB) --no-print-directory -C @BUILD-$(1) $$* || $(call do_failure,,build-$(1)-$$*,Make target $(1)-$$* failed!)
	@$(call do_done,,build-$(1)-$$*)

endef

$(eval $(call rule,debug))
$(eval $(call rule,debug-addresssan))
$(eval $(call rule,debug-threadsan))
$(eval $(call rule,release))
$(eval $(call rule,release-addresssan))
$(eval $(call rule,release-threadsan))
$(eval $(call rule,release-check))
$(eval $(call rule,release-check-addresssan))
$(eval $(call rule,release-check-threadsan))

.PHONY: clean
clean:
	@$(call do_begin,,$@)
	@git clean -f -d -X
	@$(call do_done,,$@)

.PHONY: clean-all
clean-all:
	@$(call do_begin,,$@)
	@git clean -f -d -x && rm -rf @BUILD-*
	@$(call do_done,,$@)
