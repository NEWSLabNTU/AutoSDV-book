#!/usr/bin/env bash
#
# Process coss_outdoor_run.mp4 video for documentation
#
# This script:
#   1. Extracts a segment (5s to 20s)
#   2. Applies video stabilization (two-pass vidstab)
#   3. Crops portrait video to horizontal (16:9)
#   4. Outputs as WebM animated image for web embedding
#
# Requirements:
#   - ffmpeg with libvidstab and libvpx support
#   - NVIDIA GPU (optional, for CUDA-accelerated decoding)
#
# Usage:
#   ./process-outdoor-video.sh
#
# Output:
#   coss_outdoor_run.webm (in same directory)

set -euo pipefail

# Get script directory (where input/output files are located)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Configuration
START_TIME=5
DURATION=15
CROP_WIDTH=1080
CROP_HEIGHT=608
CROP_Y=500  # Y offset for horizontal crop from portrait video

# Output settings for web-friendly animated image
OUTPUT_WIDTH=640      # Scale down for web
OUTPUT_FPS=15         # Reduce framerate for smaller file
WEBM_CRF=30           # VP9 quality (lower = better, 30 is good for web)

# Input/Output (relative to script directory)
INPUT="$SCRIPT_DIR/coss_outdoor_run.mp4"
OUTPUT="$SCRIPT_DIR/coss_outdoor_run.webm"
TMPDIR="$SCRIPT_DIR/tmp_processing_$$"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

log_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

cleanup() {
    if [[ -d "$TMPDIR" ]]; then
        log_info "Cleaning up temporary files..."
        rm -rf "$TMPDIR"
    fi
}
trap cleanup EXIT

# Check dependencies
check_deps() {
    if ! command -v ffmpeg &>/dev/null; then
        log_error "ffmpeg not found. Please install ffmpeg."
        exit 1
    fi

    local filters
    filters=$(ffmpeg -filters 2>&1)
    if ! echo "$filters" | grep -q vidstabdetect; then
        log_error "ffmpeg lacks vidstab support. Install ffmpeg with --enable-libvidstab"
        exit 1
    fi

    # Check for VP9 encoder
    local encoders
    encoders=$(ffmpeg -encoders 2>&1)
    if ! echo "$encoders" | grep -q libvpx-vp9; then
        log_error "ffmpeg lacks VP9 support. Install ffmpeg with --enable-libvpx"
        exit 1
    fi

    # Check for CUDA support (optional)
    local hwaccels
    hwaccels=$(ffmpeg -hwaccels 2>&1)
    if echo "$hwaccels" | grep -q cuda; then
        log_info "CUDA hardware acceleration available"
        USE_CUDA=true
    else
        log_warn "CUDA not available, using software decoding"
        USE_CUDA=false
    fi
}

# Check input file
check_input() {
    if [[ ! -f "$INPUT" ]]; then
        log_error "Input file not found: $INPUT"
        exit 1
    fi
    log_info "Input: $INPUT"
    log_info "Output: $OUTPUT"
}

# Step 1: Extract segment and fix rotation
extract_segment() {
    log_info "Step 1/4: Extracting segment (${START_TIME}s to $((START_TIME + DURATION))s)..."

    local segment="$TMPDIR/segment.mp4"

    if $USE_CUDA; then
        # Use CUDA for decoding, pipe through rawvideo to strip rotation metadata
        ffmpeg -y -hwaccel cuda -hwaccel_output_format cuda \
            -ss "$START_TIME" -t "$DURATION" -i "$INPUT" \
            -vf "hwdownload,format=nv12,transpose=1" \
            -f rawvideo -pix_fmt yuv420p - 2>/dev/null | \
        ffmpeg -y -f rawvideo -pix_fmt yuv420p -s 1080x1920 -r 30 -i - \
            -c:v libx264 -preset fast -crf 18 "$segment" 2>/dev/null
    else
        # Software decoding path
        ffmpeg -y -ss "$START_TIME" -t "$DURATION" -i "$INPUT" \
            -vf "transpose=1" \
            -f rawvideo -pix_fmt yuv420p - 2>/dev/null | \
        ffmpeg -y -f rawvideo -pix_fmt yuv420p -s 1080x1920 -r 30 -i - \
            -c:v libx264 -preset fast -crf 18 "$segment" 2>/dev/null
    fi

    log_info "Segment extracted: $(du -h "$segment" | cut -f1)"
}

# Step 2: Stabilize video (two-pass)
stabilize_video() {
    log_info "Step 2/4: Stabilizing video..."

    local segment="$TMPDIR/segment.mp4"
    local transforms="$TMPDIR/transforms.trf"
    local stabilized="$TMPDIR/stabilized.mp4"

    # Pass 1: Detect motion
    log_info "  Pass 1: Analyzing motion..."
    ffmpeg -y -i "$segment" \
        -vf "vidstabdetect=stepsize=6:shakiness=5:accuracy=15:result=$transforms" \
        -f null - 2>/dev/null

    # Pass 2: Apply stabilization
    log_info "  Pass 2: Applying stabilization..."
    ffmpeg -y -i "$segment" \
        -vf "vidstabtransform=input=$transforms:zoom=1:smoothing=30" \
        -c:v libx264 -preset fast -crf 18 "$stabilized" 2>/dev/null

    log_info "Stabilization complete: $(du -h "$stabilized" | cut -f1)"
}

# Step 3: Crop to horizontal
crop_video() {
    log_info "Step 3/4: Cropping to horizontal (${CROP_WIDTH}x${CROP_HEIGHT})..."

    local stabilized="$TMPDIR/stabilized.mp4"
    local cropped="$TMPDIR/cropped.mp4"

    ffmpeg -y -i "$stabilized" \
        -vf "crop=${CROP_WIDTH}:${CROP_HEIGHT}:0:${CROP_Y}" \
        -c:v libx264 -preset fast -crf 18 "$cropped" 2>/dev/null

    log_info "Cropped: $(du -h "$cropped" | cut -f1)"
}

# Step 4: Convert to WebM animated image
create_webm() {
    log_info "Step 4/4: Creating WebM animated image..."
    log_info "  Resolution: ${OUTPUT_WIDTH}px width, ${OUTPUT_FPS}fps"

    local cropped="$TMPDIR/cropped.mp4"

    # Calculate height maintaining aspect ratio
    local output_height=$((OUTPUT_WIDTH * CROP_HEIGHT / CROP_WIDTH))

    # Two-pass VP9 encoding for best quality/size ratio
    log_info "  VP9 Pass 1/2..."
    ffmpeg -y -i "$cropped" \
        -vf "scale=${OUTPUT_WIDTH}:${output_height},fps=${OUTPUT_FPS}" \
        -c:v libvpx-vp9 -b:v 0 -crf "$WEBM_CRF" \
        -pass 1 -an -f null /dev/null 2>/dev/null

    log_info "  VP9 Pass 2/2..."
    ffmpeg -y -i "$cropped" \
        -vf "scale=${OUTPUT_WIDTH}:${output_height},fps=${OUTPUT_FPS}" \
        -c:v libvpx-vp9 -b:v 0 -crf "$WEBM_CRF" \
        -pass 2 -an "$OUTPUT" 2>/dev/null

    # Clean up VP9 log files
    rm -f ffmpeg2pass-0.log 2>/dev/null || true

    log_info "WebM created: $(du -h "$OUTPUT" | cut -f1)"
}

# Main
main() {
    log_info "=== Video Processing Script ==="
    log_info "Working directory: $SCRIPT_DIR"

    check_deps
    check_input

    mkdir -p "$TMPDIR"

    extract_segment
    stabilize_video
    crop_video
    create_webm

    log_info "=== Processing Complete ==="
    log_info "Output saved to: $OUTPUT"

    # Show video info
    echo ""
    ffprobe -v error -show_entries stream=width,height,duration -of default=noprint_wrappers=1 "$OUTPUT"
}

main "$@"
