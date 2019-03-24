# PointCloudViewer

Run:
./PointCloudViewer -i sample.xyz


Debug:
gdb ./PointCloudViewer
r -i sample.xyz
r -i ../SfMLearner/point_cloud.xyz
b point_cloud.cpp:55
explore point.X(0)

info b   display all breakpoints

