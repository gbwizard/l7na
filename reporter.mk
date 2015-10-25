text_see_into		?= For details please see the
text_shit_happens	?= Failed...
text_have_slipped	?= Success!

# no, flow-id
console_buildno		= == Номер сборки $(1)
# file, flow-id
console_publish		= == Файлы $(1)
# flow-id, block, msg
console_log		= $(if $(2),$(2): )$(3)
# flow-id, msg
console_msg		= -- $(console_log)
# flow-id, error, details
console_error		= !! $(console_log)
# flow-id, block, msg
console_begin		= >> $(console_log)
# flow-id, block, msg, files
console_done		= << $(console_log)$(if $(4), - `basename $(4)`)
# flow-id, block, msg, logs
console_failure		= ** $(console_log)$(if $(4),;\n** $(text_see_into) `basename $(4)`)
# flow-id, path-pattern
console_junit		= -- $(console_log) результаты прогона
console_ctest		= -- $(console_log) результаты прогона

# no, flow-id
do_buildno		= echo -e "$(call console_buildno,$(1),$(2))"
# file, flow-id
do_publish		= echo -e "$(call console_publish,$(1),$(2))"
# flow-id, msg
do_msg			= echo -e "$(call console_msg,$(1),$(2))"
# flow-id, error, details
do_error		= echo -e "$(call console_error,$(1),$(2),$(3))" >&2
# flow-id, block, msg
do_begin		= echo -e "$(call console_begin,$(1),$(2),$(3))"
# flow-id, block, msg, files
do_done			= echo -e "$(call console_done,$(1),$(2),$(if $(3),$(3),$(text_have_slipped)),$(4))"
# flow-id, block, msg, logs
do_failure		= (echo -e "$(call console_failure,$(1),$(2),$(if $(3),$(3),$(text_shit_happens)),$(4))" >&2; exit 1)
# flow-id, path-pattern
do_junit		= echo -e "$(call $(reporter)_junit,$(1),$(2),$(3))"
do_ctest		= echo -e "$(call $(reporter)_ctest,$(1),$(2),$(3))"
