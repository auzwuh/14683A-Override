################################################################################
######################### User configurable parameters #########################
# filename extensions
CEXTS:=c
ASMEXTS:=s S
CXXEXTS:=cpp c++ cc

# probably shouldn't modify these, but you may need them below
ROOT=.
FWDIR:=$(ROOT)/firmware
BINDIR=$(ROOT)/bin
SRCDIR=$(ROOT)/src
INCDIR=$(ROOT)/include

WARNFLAGS+=
EXTRA_CFLAGS=
EXTRA_CXXFLAGS=

# Set to 1 to enable hot/cold linking
USE_PACKAGE:=1

# Add libraries you do not wish to include in the cold image here
# EXCLUDE_COLD_LIBRARIES:= $(FWDIR)/your_library.a
EXCLUDE_COLD_LIBRARIES:= 

# Set this to 1 to add additional rules to compile your project as a PROS library template
IS_LIBRARY:=0
# TODO: CHANGE THIS! 
# Be sure that your header files are in the include directory inside of a folder with the
# same name as what you set LIBNAME to below.
LIBNAME:=libbest
VERSION:=1.0.0
# EXCLUDE_SRC_FROM_LIB= $(SRCDIR)/unpublishedfile.c
# this line excludes opcontrol.c and similar files
EXCLUDE_SRC_FROM_LIB+=$(foreach file, $(SRCDIR)/main,$(foreach cext,$(CEXTS),$(file).$(cext)) $(foreach cxxext,$(CXXEXTS),$(file).$(cxxext)))

# files that get distributed to every user (beyond your source archive) - add
# whatever files you want here. This line is configured to add all header files
# that are in the directory include/LIBNAME
TEMPLATE_FILES=$(INCDIR)/$(LIBNAME)/*.h $(INCDIR)/$(LIBNAME)/*.hpp

.DEFAULT_GOAL=quick

################################################################################
# Static asset pipeline
#
# Every file under static/ gets objcopy'd into a linkable .o blob and pulled
# into the final binary, so PATH_FOLLOW-style code can read it back with the
# ASSET() macro (include/gen/asset.hpp). objcopy names the embedded symbols
# after the exact relative input path with every non-alnum character turned
# into '_', so a file at static/examplePath.txt becomes:
#   _binary_static_examplePath_txt_start / _end / _size
# which is exactly what ASSET(examplePath_txt) expects. Atticus Terminal's
# codegen (mod/codegen.py, generate_path_asset_name) already assumes this
# convention when it emits ASSET(...) declarations for exported paths.
STATIC_DIR:=static
STATIC_SRC:=$(wildcard $(STATIC_DIR)/*)
STATIC_OBJ:=$(addprefix $(BINDIR)/,$(addsuffix .o,$(STATIC_SRC)))

$(BINDIR)/$(STATIC_DIR)/%.o: $(STATIC_DIR)/%
	$(VV)mkdir -p $(dir $@)
	$(call test_output_2,Embedding $< ,$(OBJCOPY) -I binary -O elf32-littlearm -B arm $< $@,$(DONE_STRING))

ELF_DEPS+=$(STATIC_OBJ)
################################################################################

################################################################################
################################################################################
########## Nothing below this line should be edited by typical users ###########
-include ./common.mk
