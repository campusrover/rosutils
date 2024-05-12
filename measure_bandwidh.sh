#!/bin/bash
echo "Topic \t\t Bandwidth"
for topic in $(rostopic list)
do
    # Run rostopic bw in the background and capture PID
    rostopic bw $topic > /tmp/topic_bw_$topic.txt 2>&1 &
    PID=$!
    sleep 2  # Adjust the sleep time if necessary
    kill $PID
    wait $PID 2>/dev/null

    # Extract bandwidth information
    bw_info=$(grep "average:" /tmp/topic_bw_$topic.txt | awk '{print $2 " " $3}')
    if [ ! -z "$bw_info" ]; then
        echo -e "${topic} \t $bw_info"
    fi

    # Clean up
    rm /tmp/topic_bw_$topic.txt
done
