#!/usr/bin/env bash
# Builds vendor/apriltag-wasm.js from the AprilRobotics apriltag C library.
# Usage: APRILTAG_SRC=/path/to/apriltag tools/apriltag-wasm/build.sh   (with emsdk_env.sh sourced)
# The committed build used AprilRobotics/apriltag at commit b7c0ebe9aa20f82ec7a828579004f9e706bfecd9.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
src="${APRILTAG_SRC:?set APRILTAG_SRC to an apriltag checkout}"
out="$here/../../vendor/apriltag-wasm.js"
emcc -O3 -DNDEBUG -I"$src" \
  "$here/wrapper.c" "$src/apriltag.c" "$src/apriltag_quad_thresh.c" "$src/tag36h11.c" \
  "$src"/common/{g2d,homography,image_u8,image_u8x3,image_u8x4,image_u8_parallel,matd,pam,pnm,pjpeg,pjpeg-idct,string_util,svd22,time_util,unionfind,workerpool,zarray,zhash,zmaxheap,pthreads_cross}.c \
  -sMODULARIZE=1 -sEXPORT_NAME=createAprilTagModule -sSINGLE_FILE=1 -sALLOW_MEMORY_GROWTH=1 -sENVIRONMENT=web,node \
  -sEXPORTED_FUNCTIONS=_at_init,_at_config,_at_buffer,_at_detect,_at_results,_at_tag_image,_at_tag_count,_malloc,_free \
  -sEXPORTED_RUNTIME_METHODS=HEAPU8,HEAPF64 \
  -o "$out"
echo "wrote $out ($(wc -c < "$out") bytes)"
