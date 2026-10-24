#!/bin/bash
input=$(cat)

model=$(echo "$input" | jq -r '.model.display_name // "Claude"')
dir=$(echo "$input" | jq -r '.workspace.current_dir // empty' | xargs -I{} basename {} 2>/dev/null)

five_hour_pct=$(echo "$input" | jq -r '.rate_limits.five_hour.used_percentage // empty')
five_hour_reset=$(echo "$input" | jq -r '.rate_limits.five_hour.resets_at // empty')
seven_day_pct=$(echo "$input" | jq -r '.rate_limits.seven_day.used_percentage // empty')
seven_day_reset=$(echo "$input" | jq -r '.rate_limits.seven_day.resets_at // empty')

# ANSI colors
GREEN=$'\033[32m'
YELLOW=$'\033[33m'
RESET=$'\033[0m'

# Window lengths in seconds
FIVE_HOUR=18000
SEVEN_DAY=604800

# GNU date (Linux) uses -d @epoch; BSD/macOS date uses -r epoch.
fmt_dt() {
    date -d "@$1" "+%-m/%-d/%y %-H:%M" 2>/dev/null || date -r "$1" "+%-m/%-d/%y %-H:%M" 2>/dev/null
}

# Extrapolate when a window will hit 100%, assuming the current rate holds.
# args: reset_epoch window_seconds used_pct -> prints exhaustion epoch (or nothing)
extrapolate() {
    local reset=$1 window=$2 pct=$3 now
    now=$(date +%s)
    awk -v reset="$reset" -v window="$window" -v pct="$pct" -v now="$now" 'BEGIN{
        start = reset - window
        elapsed = now - start
        if (pct > 0 && elapsed > 0) printf "%d", start + (100 * elapsed / pct)
    }'
}

# Build a colored "label: pct% (reset; run-out)" segment
build_segment() {
    local label=$1 color=$2 pct=$3 reset=$4 window=$5
    local seg="${color}${label}: ${pct%.*}%"
    if [ -n "$reset" ]; then
        local runout
        runout=$(extrapolate "$reset" "$window" "$pct")
        if [ -n "$runout" ]; then
            seg="$seg ($(fmt_dt "$reset"); $(fmt_dt "$runout"))"
        else
            seg="$seg ($(fmt_dt "$reset"))"
        fi
    fi
    seg="$seg${RESET}"
    printf "%s" "$seg"
}

out="$model"
[ -n "$dir" ] && out="$out | $dir"

if [ -n "$five_hour_pct" ]; then
    out="$out | $(build_segment "5h" "$GREEN" "$five_hour_pct" "$five_hour_reset" "$FIVE_HOUR")"
fi

if [ -n "$seven_day_pct" ]; then
    out="$out | $(build_segment "1wk" "$YELLOW" "$seven_day_pct" "$seven_day_reset" "$SEVEN_DAY")"
fi

printf "%s" "$out"
