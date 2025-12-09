#For swarm demo only
# Map dimensions
int32 width
int32 height

# Flattened grid data (row-major order, values: 1=traversable, 2=2-way split, 3=3-way split)
int32[] grid_data