#!/usr/bin/env bash
#
# Turn a raw screencast of the planning simulation into the two files the
# tutorial embeds.
#
# Usage:
#   ./process-planning-sim.sh "~/Screencasts/Screencast from ....webm"
#
# Output (beside this script):
#   planning-sim.webm   VP9 -- the only format the site publishes. mkdocs.yml's
#                       exclude plugin whitelists md/svg/png/gif/jpg/webp/webm,
#                       so an mp4 placed here would be committed and never
#                       served. VP9-in-WebM covers Chrome, Firefox, Edge and
#                       Safari 14.1 or newer.
#
# Two settings are doing the work, and both were arrived at by measuring:
#
#   fps=15        GNOME's recorder writes a variable frame rate on a 1000 fps
#                 timebase. ffmpeg reads that as r_frame_rate=1000/1, and x264
#                 duplicates frames up to it -- 84 seconds became 84,641 frames
#                 and a 41 MB file. Pinning the rate makes the same video 3.2 MB.
#   scale=1280    Wide enough that the AutowareStatePanel labels and the
#                 terminal stay readable; 960 blurs them.
set -euo pipefail

SRC="${1:?usage: $0 <screencast.webm>}"
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VF='fps=15,scale=1280:-2'

ffmpeg -y -i "$SRC" -an -vf "$VF" \
    -c:v libvpx-vp9 -crf 34 -b:v 0 -row-mt 1 -deadline good -cpu-used 2 \
    -pix_fmt yuv420p "$DIR/planning-sim.webm"

ls -lh "$DIR"/planning-sim.webm
