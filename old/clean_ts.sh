# Configuration
tailnet="taila49c0.ts.net"  # Replace with your Tailnet's IP or domain
apikey="$TAILSCALE_API_KEY" # Replace with your API key
# # Function to get the date 21 days ago
# get_date_21_days_ago() {
#   if date --version >/dev/null 2>&1; then
#     # GNU date command (Linux)
#     date -d "21 days ago" '+%Y-%m-%d'
#   else
#     # BSD date command (macOS)
#     date -v -21d '+%Y-%m-%d'
#   fi
# }

# # Get the date 21 days ago
# twenty_one_days_ago=$(get_date_21_days_ago)

# # Fetch the list of devices and process each one
# curl -s "https://api.tailscale.com/api/v2/tailnet/$tailnet/devices" -u "$apikey:" | \
#   jq -r '.devices[] | "\(.lastSeen) \(.name)"' | \
#   while read -r seen name; do
#     # Extract the date from the ISO8601 timestamp
#     seen_date=$(echo "$seen" | cut -d'T' -f1)
    
#     # Compare the last seen date with the date 21 days ago
#     if [[ $seen_date < $twenty_one_days_ago ]]; then
#       echo "Device $name was last seen on $seen_date - removing it"
#       curl -s -X DELETE "https://api.tailscale.com/api/v2/device/$name" -u "$apikey:"
#     else
#       echo "Device $name was last seen on $seen_date - keeping it"
#     fi
#   done

# Function to get the date 21 days ago
get_date_21_days_ago() {
  if date --version >/dev/null 2>&1; then
    # GNU date command (Linux)
    date -d "21 days ago" '+%Y-%m-%d'
  else
    # BSD date command (macOS)
    date -v -21d '+%Y-%m-%d'
  fi
}

# Get the date 21 days ago
twenty_one_days_ago=$(get_date_21_days_ago)

# Fetch the list of devices and process each one
curl -s "https://api.tailscale.com/api/v2/tailnet/$tailnet/devices [api.tailscale.com]" -u "$apikey:" | \
  jq -r '.devices[] | "\(.lastSeen) \(.name) \(.id)"' | \
  while read -r seen name id; do
    # Extract the date from the ISO8601 timestamp
    seen_date=$(echo "$seen" | cut -d'T' -f1)
    
    # Compare the last seen date with the date 21 days ago
    if [[ $seen_date < $twenty_one_days_ago ]]; then
      echo "Device $name (ID: $id) was last seen on $seen_date - removing it"
      curl -s -X DELETE "https://api.tailscale.com/api/v2/device/$id [api.tailscale.com]" -u "$apikey:"
    else
      echo "Device $name (ID: $id) was last seen on $seen_date - keeping it"
    fi
  done
