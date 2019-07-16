# This represents a checkpoint of a recorded record.

# Checkpoint stamp, denoting the time/index of record entry
int64 stamp

# Checkpoint digest, denoting the proof for record entry
std_msgs/ByteMultiArray hash
