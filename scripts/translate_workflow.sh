#!/bin/bash

# Manual AI translation workflow management script
# Coordinates splitting PO files for manual AI translation and merging results
# Workflow: Split into chunks -> Manual AI translation -> Merge back

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BOOK_DIR="$(dirname "$SCRIPT_DIR")"
PO_FILE="$BOOK_DIR/po/zh-TW.po"
WORKSPACE_DIR="$BOOK_DIR/translation_workspace"
CHUNKS_DIR="$WORKSPACE_DIR/chunks"
CHUNK_SIZE=50

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

log_step() {
    echo -e "${BLUE}[STEP]${NC} $1"
}

# Show usage information
usage() {
    cat << EOF
Manual AI Translation Workflow Script

Usage: $0 [COMMAND] [OPTIONS]

Main Commands:
    start               Start new translation session (reset and split)
    continue            Continue with manual AI translation
    finish              Merge all translations and build
    
Individual Commands:
    status              Check current translation progress
    clean               Clean workspace (removes all chunks)
    split               Split untranslated entries into chunks
    merge [-b]          Merge translations (-b for backup)
    build               Build the book
    
Options:
    -h, --help         Show this help message
    -s SIZE            Set chunk size (default: 20)

Typical Workflow:
    1. $0 start        # Initialize and split into chunks
    2. [Manual AI translation of chunks in /tmp/po_chunks/]
    3. $0 merge        # Merge translated chunks back
    3. $0 finish       # Merge and build when done

EOF
}

# Check translation status with progress bar
check_status() {
    log_info "Translation Progress"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    
    if [ ! -f "$PO_FILE" ]; then
        log_error "PO file not found: $PO_FILE"
        return 1
    fi
    
    # Get statistics
    local stats=$(msgfmt --statistics "$PO_FILE" 2>&1)
    echo "$stats" | sed 's/^/  /'
    
    # Parse numbers for progress bar
    local translated=$(echo "$stats" | grep -oE '[0-9]+ translated' | grep -oE '[0-9]+' || echo "0")
    local untranslated=$(echo "$stats" | grep -oE '[0-9]+ untranslated' | grep -oE '[0-9]+' || echo "0")
    local total=$((translated + untranslated))
    
    if [ $total -gt 0 ]; then
        local percentage=$((translated * 100 / total))
        echo ""
        echo -n "  Progress: ["
        
        # Draw progress bar
        local bar_width=40
        local filled=$((percentage * bar_width / 100))
        for ((i=0; i<filled; i++)); do echo -n "█"; done
        for ((i=filled; i<bar_width; i++)); do echo -n "░"; done
        echo "] ${percentage}%"
    fi
    
    # Check workspace status
    if [ -d "$CHUNKS_DIR" ]; then
        echo ""
        log_info "Workspace Status"
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        
        local total_chunks=$(ls -1 "$CHUNKS_DIR"/chunk_*.po 2>/dev/null | grep -v "_translated" | wc -l)
        local translated_chunks=$(ls -1 "$CHUNKS_DIR"/chunk_*_translated.po 2>/dev/null | wc -l)
        
        echo "  Chunks: $translated_chunks/$total_chunks completed"
        
        if [ $total_chunks -gt 0 ]; then
            # Find next chunk
            for i in $(seq -f "%03g" 1 $total_chunks); do
                if [ ! -f "$CHUNKS_DIR/chunk_${i}_translated.po" ]; then
                    echo "  Next: chunk_${i}.po"
                    break
                fi
            done
        fi
    fi
    
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
}

# Start new translation session
start_session() {
    log_step "Starting new translation session"
    
    # Clean workspace
    log_info "Cleaning workspace..."
    rm -rf "$WORKSPACE_DIR"
    mkdir -p "$CHUNKS_DIR"
    
    # Split PO file
    log_info "Splitting untranslated entries..."
    python3 "$SCRIPT_DIR/split_for_manual_translation.py" reset
    
    local chunk_count=$(ls -1 "$CHUNKS_DIR"/chunk_*.po 2>/dev/null | grep -v "_translated" | wc -l)
    
    if [ $chunk_count -eq 0 ]; then
        log_info "No untranslated entries found! Translation complete."
        return 0
    fi
    
    log_info "Created $chunk_count chunks"
    
    # Auto-translate all chunks
    log_info "Running auto-translation..."
    $0 auto
    
    echo ""
    log_info "Session initialized successfully!"
    echo "  Next: Run '$0 next' to review/improve translations"
    echo "  Or:   Run '$0 finish' to apply current translations"
}

# Continue translation
continue_session() {
    if [ ! -d "$CHUNKS_DIR" ]; then
        log_error "No active session. Run '$0 start' first."
        return 1
    fi
    
    # Process all remaining chunks
    while true; do
        if ! process_next_chunk; then
            break
        fi
        
        echo ""
        read -p "Continue with next chunk? (y/n) " -n 1 -r
        echo ""
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            break
        fi
    done
}

# Process next chunk
process_next_chunk() {
    # Find next untranslated chunk
    local next_chunk=""
    for i in $(seq -f "%03g" 1 100); do
        if [ -f "$CHUNKS_DIR/chunk_${i}.po" ] && [ ! -f "$CHUNKS_DIR/chunk_${i}_translated.po" ]; then
            next_chunk="chunk_${i}"
            break
        fi
    done
    
    if [ -z "$next_chunk" ]; then
        log_info "All chunks processed!"
        echo "  Run: '$0 finish' to merge and build"
        return 1
    fi
    
    log_step "Processing $next_chunk"
    
    # Copy to translated
    cp "$CHUNKS_DIR/${next_chunk}.po" "$CHUNKS_DIR/${next_chunk}_translated.po"
    
    # Show chunk info
    local entries=$(grep -c "^msgid " "$CHUNKS_DIR/${next_chunk}.po" || true)
    echo "  Entries: $entries"
    
    # Auto-translate this chunk
    python3 "$SCRIPT_DIR/auto_translate.py" "$CHUNKS_DIR/${next_chunk}_translated.po"
    
    log_info "Auto-translation applied to $next_chunk"
    
    # Count remaining
    local remaining=0
    for i in $(seq -f "%03g" 1 100); do
        if [ -f "$CHUNKS_DIR/chunk_${i}.po" ] && [ ! -f "$CHUNKS_DIR/chunk_${i}_translated.po" ]; then
            remaining=$((remaining + 1))
        fi
    done
    
    if [ $remaining -gt 0 ]; then
        echo "  Remaining: $remaining chunks"
    fi
    
    return 0
}

# Finish translation session
finish_session() {
    log_step "Finalizing translations"
    
    if [ ! -d "$CHUNKS_DIR" ]; then
        log_error "No active session found"
        return 1
    fi
    
    # Ensure all chunks have translated versions
    log_info "Preparing chunks..."
    for chunk in "$CHUNKS_DIR"/chunk_*.po; do
        if [[ ! "$chunk" =~ _translated\.po$ ]]; then
            base=$(basename "$chunk" .po)
            if [ ! -f "$CHUNKS_DIR/${base}_translated.po" ]; then
                cp "$chunk" "$CHUNKS_DIR/${base}_translated.po"
            fi
        fi
    done
    
    # Merge with backup
    log_info "Merging translations..."
    cp "$PO_FILE" "${PO_FILE}.backup"
    python3 "$SCRIPT_DIR/merge_translated_chunks.py" "$PO_FILE" -c "$CHUNKS_DIR"
    
    # Build
    log_info "Building book..."
    cd "$BOOK_DIR"
    make build
    
    log_info "Translation complete!"
    echo "  View at: book/zh-TW/index.html"
    
    # Show final status
    echo ""
    check_status
}

# Quick workflow - fully automated
quick_workflow() {
    log_step "Running quick automated workflow"
    
    start_session
    
    # Auto-process all chunks
    log_info "Processing all chunks automatically..."
    while process_next_chunk 2>/dev/null; do
        :
    done
    
    finish_session
}

# Main command handler
case "${1:-}" in
    start)
        start_session
        ;;
    continue)
        continue_session
        ;;
    next)
        process_next_chunk
        ;;
    finish)
        finish_session
        ;;
    status)
        check_status
        ;;
    clean)
        log_warn "Cleaning workspace..."
        rm -rf "$WORKSPACE_DIR"
        log_info "Workspace cleaned"
        ;;
    split)
        mkdir -p "$CHUNKS_DIR"
        python3 "$SCRIPT_DIR/split_for_manual_translation.py" reset
        ;;
    auto)
        # Auto-translate all chunks in the workspace
        if [ ! -d "$CHUNKS_DIR" ]; then
            log_error "No active session. Run './scripts/translate_workflow.sh start' first"
            exit 1
        fi
        
        log_info "Auto-translating all chunks..."
        
        chunk_count=0
        total_translations=0
        
        for chunk in "$CHUNKS_DIR"/chunk_*_translated.po; do
            if [[ -f "$chunk" ]]; then
                chunk_name=$(basename "$chunk")
                printf "  %-30s " "$chunk_name:"
                
                # Run auto_translate.py on this chunk
                result=$(python3 "$SCRIPT_DIR/auto_translate.py" "$chunk" 2>&1)
                
                # Extract number of translations added
                if [[ "$result" =~ Added[[:space:]]([0-9]+)[[:space:]]translations ]]; then
                    count="${BASH_REMATCH[1]}"
                    echo "$count translations"
                    total_translations=$((total_translations + count))
                else
                    echo "error"
                fi
                
                chunk_count=$((chunk_count + 1))
            fi
        done
        
        echo ""
        log_info "Processed $chunk_count chunks, added $total_translations translations"
        ;;
    merge)
        shift
        if [ "${1:-}" == "-b" ]; then
            cp "$PO_FILE" "${PO_FILE}.backup"
            log_info "Created backup: ${PO_FILE}.backup"
        fi
        python3 "$SCRIPT_DIR/merge_translated_chunks.py" "$PO_FILE" -c "$CHUNKS_DIR"
        ;;
    build)
        cd "$BOOK_DIR"
        make build
        ;;
    quick)
        quick_workflow
        ;;
    -h|--help|help|"")
        usage
        ;;
    *)
        log_error "Unknown command: $1"
        usage
        exit 1
        ;;
esac