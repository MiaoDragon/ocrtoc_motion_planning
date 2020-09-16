int32 action  # specify the action to be taken.
int32 StartFilter=1
int32 StopFilter=0
float64[] min_xyz
float64[] max_xyz
float64[] pcd_transform  # TF matrix to transform the pcd-to-filter to the same frame as the box