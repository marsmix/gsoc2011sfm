# F2C
# Once done, this will define
#
#  F2C_FOUND - system has libf2c
#  F2C_INCLUDE_DIR - the libf2c include directories
#  F2C_LIBRARY - link these to use libf2c

# Finally the library itself
find_library(F2C_LIBRARY
  NAMES f2c
)
