#!/usr/bin/env python3
import bagpy
import pandas as pd
import sys

def bag_to_csv(bag_file):
    bag = bagpy.bagreader(bag_file)
    
    # Get a list of topics and their types
    topics = bag.topic_table['Topics'].tolist()
    types = bag.topic_table['Types'].tolist()
    topic_type_dict = dict(zip(topics, types))
    
    csv_file = bag_file.replace('.bag', '.csv')
    
    # Placeholder for combined data
    full_df = pd.DataFrame()
    
    # Process each topic
    for topic in topics:
        data = bag.message_by_topic(topic)
        df = pd.read_csv(data)
        
        # Add topic name and message type columns
        df['Topic'] = topic
        df['MessageType'] = topic_type_dict[topic]
        
        # Concatenate data
        full_df = pd.concat([full_df, df], ignore_index=True)
    
    # Save to CSV
    full_df.to_csv(csv_file, index=False)
    print(f"CSV file created: {csv_file}")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python bag_to_csv.py <bagfile>")
        sys.exit(1)

    bag_to_csv(sys.argv[1])