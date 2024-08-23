import bagpy
import pandas as pd
import sys

def bag_to_csv(bag_file):
    bag = bagpy.bagreader(bag_file)
    
    # Get a list of topics
    topics = bag.topic_table['Topics'].tolist()
    
    # Assume all topics have the same message type
    first_topic = topics[0]
    csv_file = bag_file.replace('.bag', '.csv')
    
    # Read messages from the first topic to determine columns
    data = bag.message_by_topic(first_topic)
    df = pd.read_csv(data)
    
    # Create a dataframe to concatenate all topic data
    full_df = pd.DataFrame(columns=df.columns)
    
    # Process each topic
    for topic in topics:
        data = bag.message_by_topic(topic)
        df = pd.read_csv(data)
        full_df = pd.concat([full_df, df], ignore_index=True)
    
    # Save to CSV
    full_df.to_csv(csv_file, index=False)
    print(f"CSV file created: {csv_file}")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python bag_to_csv.py <bagfile>")
        sys.exit(1)

    bag_to_csv(sys.argv[1])
    