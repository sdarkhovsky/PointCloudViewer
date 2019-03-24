#include "point_cloud.h"

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <cfloat>

using namespace std;
using namespace Eigen;
using namespace pcv;

namespace pcv {

    bool c_point_cloud::read_point_cloud_file(std::string file_path) {
        string line;
        std::ifstream infile;
        size_t width, height;

        for (int i = 0; i < 3; i++) {
            min_coord(i) = FLT_MAX;
            max_coord(i) = FLT_MIN;
        }

        size_t file_path_size = file_path.size();
        bool b_kinect_file = false;
        if (file_path_size > 4) {
            std::string file_path_ext = file_path.substr(file_path_size - 4, 4);
            transform(file_path_ext.begin(), file_path_ext.end(), file_path_ext.begin(), ::tolower);
            if (file_path_ext == ".kin") {
                b_kinect_file = true;
            }
        }

        infile.open(file_path, std::ifstream::in);

        if (b_kinect_file) {
            std::stringstream linestream;

            std::getline(infile, line);
            linestream << line;
            linestream >> height;

            std::getline(infile, line);
            linestream.clear();
            linestream << line;
            linestream >> width;
        }

        // read points
        while (std::getline(infile, line)) {
            c_point_cloud_point point;
            std::stringstream linestream(line);

            if (b_kinect_file) {
                linestream >> point.u;
                linestream >> point.v;
            }
            else {
                point.u = 0;
                point.v = 0;
            }

            for (int i = 0; i < 3; i++) {
                linestream >> point.X(i);
//                cout << "X=" << point.X(i) << " i=" << i << endl;
            }

            for (int i = 0; i < 3; i++) {
                linestream >> point.Clr(i);
//                cout << "Clr=" << point.Clr(i) << " i=" << i << endl;                
            }

            point.Vector = Vector3f::Zero();
            point.Label = Vector3i::Zero();

            if (linestream.good()) {
                for (int i = 0; i < 3; i++) {
                    linestream >> point.Label(i);
                }

                if (linestream.good()) {
                    for (int i = 0; i < 3; i++) {
                        linestream >> point.Vector(i);
                    }
                }
            }

            for (int i = 0; i < 3; i++) {
                if (point.X(i) < min_coord(i)) min_coord(i) = point.X(i);
                if (point.X(i) > max_coord(i)) max_coord(i) = point.X(i);
            }
            point.visible = 1;
            points.push_back(point);
        }
        infile.close();

        return true;
    }

    bool c_point_cloud::write_point_cloud_file(std::string file_path) {
        // read points from xyz file
        string line;
        std::ofstream outfile;
        outfile.open(file_path, std::ifstream::out);

        size_t file_path_size = file_path.size();
        bool b_kinect_file = false;
        if (file_path_size > 4) {
            std::string file_path_ext = file_path.substr(file_path_size - 4, 4);
            transform(file_path_ext.begin(), file_path_ext.end(), file_path_ext.begin(), ::tolower);
            if (file_path_ext == ".kin") {
                b_kinect_file = true;
            }
        }

        int min_u = INT_MAX;
        int max_u = INT_MIN;
        int min_v = INT_MAX;
        int max_v = INT_MIN;
        int height = 0;
        int width = 0;
        if (b_kinect_file) {
            for (auto it = points.begin(); it != points.end(); ++it) {
                if (!it->visible)
                    continue;
                if (it->u < min_u)
                    min_u = it->u;
                if (it->u > max_u)
                    max_u = it->u;
                if (it->v < min_v)
                    min_v = it->v;
                if (it->v > max_v)
                    max_v = it->v;
            }

            height = max_u - min_u + 1;
            width = max_v - min_v + 1;

            outfile << height << std::endl;
            outfile << width << std::endl;
        }

        for (auto it = points.begin(); it != points.end(); ++it) {
            if (!it->visible)
                continue;

            if (b_kinect_file) {
                outfile << (it->u - min_u) << " ";
                outfile << (it->v - min_v) << " ";
            }

            outfile << it->X(0) << " ";
            outfile << it->X(1) << " ";
            outfile << it->X(2) << " ";
            outfile << it->Clr(0) << " ";
            outfile << it->Clr(1) << " ";
            outfile << it->Clr(2); 

            if (b_kinect_file) {
                outfile << " " << it->Label(0) << " ";
                outfile << it->Label(1) << " ";
                outfile << it->Label(2);

                if (it->Vector != Vector3f::Zero()) {
                    outfile << " " << it->Vector(0) << " ";
                    outfile << it->Vector(1) << " ";
                    outfile << it->Vector(2); 
                }
            }
            outfile << std::endl;
        }

        outfile.close();
        return true;
    }

    void c_point_cloud::reset_visibility() {
        for (auto it = points.begin(); it != points.end(); ++it) {
            it->visible = true;
        }
    }
}
