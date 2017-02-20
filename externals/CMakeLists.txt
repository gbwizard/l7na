###############################################################################
# Ethercat tool library
###############################################################################

set(ETHERCAT_TOOL_DIR ${ETHERCAT_DIR}/tool)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-error")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error")
add_library(ethercat_tool STATIC
    ${ETHERCAT_DIR}/master/soe_errors.c
    ${ETHERCAT_TOOL_DIR}/Command.cpp
    ${ETHERCAT_TOOL_DIR}/CommandAlias.cpp
    ${ETHERCAT_TOOL_DIR}/CommandCStruct.cpp
    ${ETHERCAT_TOOL_DIR}/CommandConfig.cpp
    ${ETHERCAT_TOOL_DIR}/CommandData.cpp
    ${ETHERCAT_TOOL_DIR}/CommandDebug.cpp
    ${ETHERCAT_TOOL_DIR}/CommandDomains.cpp
    ${ETHERCAT_TOOL_DIR}/CommandDownload.cpp
    ${ETHERCAT_TOOL_DIR}/CommandFoeRead.cpp
    ${ETHERCAT_TOOL_DIR}/CommandFoeWrite.cpp
    ${ETHERCAT_TOOL_DIR}/CommandGraph.cpp
    ${ETHERCAT_TOOL_DIR}/CommandMaster.cpp
    ${ETHERCAT_TOOL_DIR}/CommandPdos.cpp
    ${ETHERCAT_TOOL_DIR}/CommandRegRead.cpp
    ${ETHERCAT_TOOL_DIR}/CommandRegWrite.cpp
    ${ETHERCAT_TOOL_DIR}/CommandRescan.cpp
    ${ETHERCAT_TOOL_DIR}/CommandSdos.cpp
    ${ETHERCAT_TOOL_DIR}/CommandSiiRead.cpp
    ${ETHERCAT_TOOL_DIR}/CommandSiiWrite.cpp
    ${ETHERCAT_TOOL_DIR}/CommandSlaves.cpp
    ${ETHERCAT_TOOL_DIR}/CommandSoeRead.cpp
    ${ETHERCAT_TOOL_DIR}/CommandSoeWrite.cpp
    ${ETHERCAT_TOOL_DIR}/CommandStates.cpp
    ${ETHERCAT_TOOL_DIR}/CommandUpload.cpp
    ${ETHERCAT_TOOL_DIR}/CommandVersion.cpp
    ${ETHERCAT_TOOL_DIR}/CommandXml.cpp
    ${ETHERCAT_TOOL_DIR}/DataTypeHandler.cpp
    ${ETHERCAT_TOOL_DIR}/FoeCommand.cpp
    ${ETHERCAT_TOOL_DIR}/MasterDevice.cpp
    ${ETHERCAT_TOOL_DIR}/NumberListParser.cpp
    ${ETHERCAT_TOOL_DIR}/SdoCommand.cpp
    ${ETHERCAT_TOOL_DIR}/SoeCommand.cpp
    ${ETHERCAT_TOOL_DIR}/sii_crc.cpp
)
