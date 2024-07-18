#!/bin/bash
echo -e "Bandwidth       Topic"
for topic in $(rostopic list)
do
    # Sanitize topic name to create a valid filename
    safe_topic=$(echo "$topic" | tr -d '/' | tr ' ' '_')
    # Run rostopic bw in the background and capture PID
    rostopic bw $topic > "/tmp/topic_bw_${safe_topic}.txt" 2>&1 &
    PID=$!
    sleep 5  # Adjust the sleep time if necessary
    kill $PID
    wait $PID 2>/dev/null

    # Extract bandwidth information
    bw_info=$(grep -m 1 "average:" "/tmp/topic_bw_${safe_topic}.txt" | awk '{print $2}')
    if [ ! -z "$bw_info" ]; then
        echo -e "$bw_info\t${topic}"
    fi

    # Clean up
    #rm "/tmp/topic_bw_${safe_topic}.txt"
done
