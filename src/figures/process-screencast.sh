#!/usr/bin/env bash
#
# Turn a raw GNOME screencast into the WebM the book embeds.
#
# Usage:
#   src/figures/process-screencast.sh <screencast.webm> <output.webm> [crf] [fps]
#
# Example:
#   src/figures/process-screencast.sh \
#       ~/Screencasts/'Screencast from 09-14-2026 12:02:10 PM.webm' \
#       src/figures/logging_sim_video/logging-sim.webm
#
# Two settings are doing the work, and both were arrived at by measuring rather
# than by taste:
#
#   fps=15        GNOME's recorder writes a VARIABLE frame rate on a 1000 fps
#                 timebase, so ffmpeg reads r_frame_rate as 1000/1 and an
#                 encoder will duplicate frames up to it: 85 seconds of screen
#                 became 84,641 frames and a 41 MB file. Pinning the rate makes
#                 the same video 3.6 MB, and nothing in a recording of a user
#                 interface moves faster than 15 fps anyway.
#
#   scale=1280    Wide enough that the AutowareStatePanel rows and the terminal
#                 stay readable, which is the entire point of these clips. At
#                 960 they blur.
#
# The CRF that suits a clip depends on what is moving in it, and the two clips
# in this book differ by four times:
#
#   planning simulation   crf 34, 15 fps -> 3.6 MB for 85 s. Almost every pixel
#                         is static; only the vehicle and the trajectory move.
#   logging simulation    crf 42, 12 fps -> 12.7 MB for 119 s. A live LiDAR scan
#                         redraws the whole view ten times a second, and that
#                         scan is the subject, so it is the last thing to trade
#                         away. crf 34 on this one produced 27.8 MB; 42 keeps
#                         the panel text sharp and the scan readable.
#
# WebM only, deliberately: mkdocs.yml's exclude plugin whitelists
# md/svg/png/gif/jpg/webp/webm, so an mp4 fallback placed beside this would be
# committed and never served. VP9-in-WebM covers Chrome, Firefox, Edge and
# Safari 14.1 or newer.
set -euo pipefail

SRC="${1:?usage: $0 <screencast.webm> <output.webm> [crf] [fps]}"
OUT="${2:?usage: $0 <screencast.webm> <output.webm> [crf] [fps]}"
CRF="${3:-34}"
FPS="${4:-15}"

mkdir -p "$(dirname "$OUT")"
ffmpeg -y -i "$SRC" -an -vf "fps=${FPS},scale=1280:-2" \
    -c:v libvpx-vp9 -crf "$CRF" -b:v 0 -row-mt 1 -deadline good -cpu-used 2 \
    -pix_fmt yuv420p "$OUT"

ls -lh "$OUT"
