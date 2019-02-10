# This represents a checkpoint of a recorded record.

# Checkpoint timestamp, denoting the time of arrival for record
#builtin_interfaces/Time stamp
int64 stamp

# Checkpoint digest, denoting the data commitment for record
std_msgs/ByteMultiArray hash

# Checkpoint nonce, denoting the ransom seed of hash chain for record
std_msgs/ByteMultiArray nonce

# # Checkpoint name, denoting the respective name for record
# string name
#
# # Checkpoint type, denoting the respective type for record
# string type
#
# # Checkpoint format, denoting the respective serialization for record
# string serialization_format
