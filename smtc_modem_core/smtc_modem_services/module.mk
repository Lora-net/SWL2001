# ----------------------------------------------------------------------------
# @file module.mk
#
# Contains list of source files to be compiled in this module.
# ----------------------------------------------------------------------------

MODULE_C_SOURCES = \
	src/alc_sync/alc_sync.c \
	src/file_upload/file_upload.c \
	src/stream/rose.c \
	src/stream/stream.c \

MODULE_C_INCLUDES = \
	. \
	src \
	src/alc_sync \
	src/file_upload \
	src/stream \
    headers \

