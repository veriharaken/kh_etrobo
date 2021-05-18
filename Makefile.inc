mkfile_path := $(dir $(lastword $(MAKEFILE_LIST)))

APPL_COBJS +=

APPL_CXXOBJS +=

SRCLANG := c++

APPL_DIRS += $(mkfile_path)control $(mkfile_path)odometry

INCLUDES += -I$(ETROBO_HRP3_WORKSPACE)/etroboc_common

# COPTS += -fno-use-cxa-atexit
# COPTS += -DMAKE_BT_DISABLE
