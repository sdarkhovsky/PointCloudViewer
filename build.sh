# see https://www.linuxjournal.com/content/introduction-opengl-programming
g++ -g -std=c++11 -I/home/sd/workspace/eigen-eigen-323c052e1731 PointCloudViewer.cpp point_cloud.cpp -lglut -lGL -lGLU -o PointCloudViewer
./PointCloudViewer -i sample.xyz
