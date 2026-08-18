"""PlatformIO extra_script: wraps the XTensa compiler with ccache."""
import os
from SCons.Script import Environment

# Put ccache wrappers on PATH so SCons finds xtensa-esp32-elf-gcc there first
ccache_dir = os.path.expanduser("~/.platformio/ccache")
if os.path.isdir(ccache_dir):
    os.environ["PATH"] = ccache_dir + os.pathsep + os.environ.get("PATH", "")
    os.environ["CCACHE_DIR"] = os.path.expanduser("~/.platformio/.ccache")
