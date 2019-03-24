# PointCloudViewer

Run:
./PointCloudViewer -i sample.xyz


Debug:
gdb ./PointCloudViewer
r -i sample.xyz
b point_cloud.cpp:55
explore point.X(0)
