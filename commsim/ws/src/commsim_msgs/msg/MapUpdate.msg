# Compressed occupancy patch, one chunk of a region update.
FleetHeader header
uint16 region_id
uint32 region_seq
uint16 chunk_index
uint16 chunk_count
bool full_refresh
uint8[] data
