#!/bin/bash
for topic in $(rostopic list)
do
  echo "Bandwidth for $topic:"
  rostopic bw $topic &
  sleep 5
  kill $!
  echo
done
