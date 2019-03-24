// PointCloudViewer.cpp : Defines the entry point for the application.
//

#include "GL/freeglut.h"
#include "GL/gl.h"


#include <GL/glu.h> 

#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <cfloat>
#include <limits>
#include <cstddef>

#include <Eigen/Dense>

#include "point_cloud.h"

using namespace std;
using namespace Eigen;
using namespace pcv;


#define BLACK_INDEX     0 
#define RED_INDEX       13 
#define GREEN_INDEX     14 
#define BLUE_INDEX      16 

void refresh_display_lists();

/* OpenGL globals, defines, and prototypes */
float translate_camera_speed;

float rotate_camera_speed;

float edge_length;

float gl_point_size;

int last_mouse_pos_x;
int last_mouse_pos_y;
bool mouse_left_button_up = true;
bool mouse_right_button_up = true;

struct c_viewport {
    c_viewport() {
    }
    c_viewport(int x, int y, int width, int height) {
        this->x = x;
        this->y = y;
        this->width = width;
        this->height = height;
    }
    int x;
    int y;
    int width;
    int height;
};
c_viewport viewport;

#define POINT_CLOUD 1

GLvoid resize_viewport(GLsizei, GLsizei);
GLvoid initializeGL(GLsizei, GLsizei);
GLvoid drawScene(GLvoid);

string input_point_cloud_file_path;
string output_point_cloud_file_path;

c_point_cloud point_cloud;

enum class c_interactive_mode: int
{
    idle,
    selecting_points_to_hide,
    updating_points_to_hide,
    selecting_points_to_keep,
    updating_points_to_keep,
    filter_label
};
c_interactive_mode interactive_mode = c_interactive_mode::idle;

Vector3i filtered_label = Vector3i(1, 0, 0);

Vector2i min_visible;
Vector2i max_visible;

Vector3f eye_pnt;
Vector3f lookat_pnt;

string text_info;
bool display_text_info = false;

bool get_command_line_options(vector< string >& arg_list) {
    int i;
    // requests parameters to be passed in pairs
    if (arg_list.size() % 2)
        return false;

    for (i = 0; i < arg_list.size(); i = i + 2) {
        if (arg_list[i] == "-i") {
            input_point_cloud_file_path = arg_list[i + 1];
        }
        if (arg_list[i] == "-o") {
            output_point_cloud_file_path = arg_list[i + 1];
        }
    }

    if (output_point_cloud_file_path.size() == 0) {
        output_point_cloud_file_path = input_point_cloud_file_path + ".out.kin";
    }

    return true;
}

void reset_camera_position() {
    // note, that in the opengl the camera looks at the negative z direction of the camera reference frame
    // position the camera looking at the object center

    lookat_pnt = (point_cloud.min_coord + point_cloud.max_coord) / 2.0f;
    eye_pnt = lookat_pnt - Vector3f(0, 0, point_cloud.max_coord(2) - point_cloud.min_coord(2));
}

void reset_settings() {
    display_text_info = false;
    filtered_label = Vector3i(1,0,0);

    translate_camera_speed = 0.03f;
    
    rotate_camera_speed = 0.002f;

    edge_length = 0.01f;

    gl_point_size = 3.0f;

    point_cloud.reset_visibility();

    reset_camera_position();

    interactive_mode = c_interactive_mode::idle;
}

bool get_point_screen_coordinate(const c_point_cloud_point& point, const Matrix4f& model_view_matrix, const Matrix4f& projection_matrix, Vector2i& point_screen_coordinates) {
    Vector4f point_coordinates;
    point_coordinates(3) = 1.0f;
    for (int i = 0; i < 3; i++) {
        point_coordinates(i) = point.X(i);
    }

    // see http://www.songho.ca/opengl/gl_transform.html#projection and http://webglfactory.blogspot.com/2011/05/how-to-convert-world-to-screen.html
    Vector4f clip_coordinates = projection_matrix*model_view_matrix*point_coordinates;
    Vector3f normalized_device_coordinates(clip_coordinates(0) / clip_coordinates(3), clip_coordinates(1) / clip_coordinates(3), clip_coordinates(2) / clip_coordinates(3));
    point_screen_coordinates(0) = (int)((float)viewport.width*(normalized_device_coordinates(0) + 1.0) / 2.0) + viewport.x;
    point_screen_coordinates(1) = (int)((float)viewport.height*(-normalized_device_coordinates(1) + 1.0) / 2.0) + viewport.y;
    return true;
}

void printtext() {
    if (display_text_info) {
        Vector2i point_screen_coordinates;

        GLfloat model_view_matrix_raw[16];
        glGetFloatv(GL_MODELVIEW_MATRIX, (GLfloat*)&model_view_matrix_raw);
        Matrix4f model_view_matrix;
        for (int i = 0; i < 4; i++) {
            model_view_matrix.col(i) << model_view_matrix_raw[i * 4], model_view_matrix_raw[i * 4 + 1], model_view_matrix_raw[i * 4 + 2], model_view_matrix_raw[i * 4 + 3];
        }

        GLfloat projection_matrix_raw[16];
        glGetFloatv(GL_PROJECTION_MATRIX, (GLfloat*)&projection_matrix_raw);
        Matrix4f projection_matrix;
        for (int i = 0; i < 4; i++) {
            projection_matrix.col(i) << projection_matrix_raw[i * 4], projection_matrix_raw[i * 4 + 1], projection_matrix_raw[i * 4 + 2], projection_matrix_raw[i * 4 + 3];
        }

        for (auto it = point_cloud.points.begin(); it != point_cloud.points.end(); ++it) {
            if (!it->visible)
                continue;

            if (get_point_screen_coordinate(*it, model_view_matrix, projection_matrix, point_screen_coordinates)) {
                if (abs(point_screen_coordinates(0) - last_mouse_pos_x) < 2 && abs(point_screen_coordinates(1) - last_mouse_pos_y) < 2) {

                    text_info = "u= " + std::to_string(it->u) + " v= " + std::to_string(it->v) + " X= (" + std::to_string(it->X(0)) + ", " + std::to_string(it->X(1)) + ", " + std::to_string(it->X(2)) + ")";
                    glutStrokeString(GLUT_STROKE_ROMAN, (unsigned char*)text_info.c_str());
               }
            }
        }
    }
}


void update_points_visibility() {
    Vector2i point_screen_coordinates;

    GLfloat model_view_matrix_raw[16];
    glGetFloatv(GL_MODELVIEW_MATRIX, (GLfloat*)&model_view_matrix_raw);
    Matrix4f model_view_matrix;
    for (int i = 0; i < 4; i++) {
        model_view_matrix.col(i) << model_view_matrix_raw[i * 4], model_view_matrix_raw[i * 4 + 1], model_view_matrix_raw[i * 4 + 2], model_view_matrix_raw[i * 4 + 3];
    }

    GLfloat projection_matrix_raw[16];
    glGetFloatv(GL_PROJECTION_MATRIX, (GLfloat*)&projection_matrix_raw);
    Matrix4f projection_matrix;
    for (int i = 0; i < 4; i++) {
        projection_matrix.col(i) << projection_matrix_raw[i * 4], projection_matrix_raw[i * 4 + 1], projection_matrix_raw[i * 4 + 2], projection_matrix_raw[i * 4 + 3];
    }

    if (interactive_mode == c_interactive_mode::filter_label) {
        bool found_label = false;
        for (auto it = point_cloud.points.begin(); it != point_cloud.points.end(); ++it) {
            if (it->Label == filtered_label) {
                found_label = true;
                it->visible = 1;
            }
            else {
                it->visible = 0;
            }
        }
        if (found_label) {
            filtered_label += Vector3i(1, 0, 0);
        }
        else {
            for (auto it = point_cloud.points.begin(); it != point_cloud.points.end(); ++it) {
                it->visible = 1;
            }
            filtered_label = Vector3i(1, 0, 0);
        }
    }
    else {
        for (auto it = point_cloud.points.begin(); it != point_cloud.points.end(); ++it) {
            if (!it->visible)
                continue;

            if (get_point_screen_coordinate(*it, model_view_matrix, projection_matrix, point_screen_coordinates)) {
                bool point_in_selected_box = true;
                for (int i = 0; i < 2; i++) {
                    if (point_screen_coordinates(i) < min_visible(i) || point_screen_coordinates(i) > max_visible(i)) {
                        point_in_selected_box = false;
                        break;
                    }
                }

                if (interactive_mode == c_interactive_mode::updating_points_to_keep) {
                    if (!point_in_selected_box) {
                        it->visible = 0;
                    }
                }
                else
                    if (interactive_mode == c_interactive_mode::updating_points_to_hide) {
                        if (point_in_selected_box) {
                            it->visible = 0;
                        }
                    }
            }
        }
    }
    refresh_display_lists();
}

/* OpenGL code */
GLvoid resize_viewport(GLsizei width, GLsizei height)
{
    GLfloat aspect;

    viewport = c_viewport(0, 0, width, height);
    glViewport(viewport.x, viewport.y, viewport.width, viewport.height);

    aspect = (GLfloat)width / height;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    // the OpenGL projection transformation (not to be confused with the projective transformation in the computer vision) transforms 
    // all vertex data from the eye coordinates to the clip coordinates.

    float zNear = (point_cloud.max_coord(2) - point_cloud.min_coord(2)) / 100.0f;;
    float zFar = FLT_MAX;  // zNear + (point_cloud.max_coord(2) - point_cloud.min_coord(2))*10.0f;
    gluPerspective(45.0, aspect, zNear, zFar);

    glMatrixMode(GL_MODELVIEW);
}

void refresh_display_lists() {
    glNewList(POINT_CLOUD, GL_COMPILE);

    for (auto it = point_cloud.points.begin(); it != point_cloud.points.end(); ++it) {
        if (it->visible == 0)
            continue;
        glBegin(GL_POINTS);
        glColor3ub((GLubyte)it->Clr(0), (GLubyte)it->Clr(1), (GLubyte)it->Clr(2));
        glVertex3f(it->X(0), it->X(1), it->X(2));
        cout << "x,y,z,r,g,b:" << it->X(0) << it->X(1) << it->X(2) << it->Clr(0) << (GLubyte)it->Clr(1) << (GLubyte)it->Clr(2) << endl;
        glEnd();

        if (it->Vector != Vector3f::Zero()) {
            glBegin(GL_LINES);
            glColor3ub(255, 0, 0);
            Vector3f normalized_edge = it->Vector;
            normalized_edge.normalize();
            Vector3f v = it->X + normalized_edge * edge_length;

            glVertex3f(it->X(0), it->X(1), it->X(2));
            glVertex3f(v(0), v(1), v(2));

            glEnd();
        }
    }

    glEndList();
}

GLvoid initializeGL(GLsizei width, GLsizei height)
{
    point_cloud.read_point_cloud_file(input_point_cloud_file_path);

    glClearIndex((GLfloat)BLACK_INDEX);
    glClearDepth(1.0);

    glEnable(GL_DEPTH_TEST);

    // white background
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    reset_camera_position();

    resize_viewport(width, height);

    refresh_display_lists();
}

GLvoid drawScene(GLvoid)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glPointSize(gl_point_size);

    glMatrixMode(GL_MODELVIEW);

    glLoadIdentity();

    gluLookAt(eye_pnt(0), eye_pnt(1), eye_pnt(2),
        lookat_pnt(0), lookat_pnt(1), lookat_pnt(2),
        0.0, 1.0, 0.0);

    if (interactive_mode == c_interactive_mode::updating_points_to_hide ||
        interactive_mode == c_interactive_mode::updating_points_to_keep ||
        interactive_mode == c_interactive_mode::filter_label
        ) {
        update_points_visibility();
        interactive_mode = c_interactive_mode::idle;
    }

    glCallList(POINT_CLOUD);
     
    glutSwapBuffers();

    printtext();
}

void drawTriangle()
{
    glClearColor(0.4, 0.4, 0.4, 0.4);
    glClear(GL_COLOR_BUFFER_BIT);

    glColor3f(1.0, 1.0, 1.0);
    glOrtho(-1.0, 1.0, -1.0, 1.0, -1.0, 1.0);

        glBegin(GL_TRIANGLES);
                glVertex3f(-0.7, 0.7, 0);
                glVertex3f(0.7, 0.7, 0);
                glVertex3f(0, -1, 0);
        glEnd();

    glFlush();
}

void special_keyboard_func(int key, int x, int y)
{
    if (key == GLUT_KEY_RIGHT)
        {

        }
    else if (key == GLUT_KEY_LEFT)
        {

        }
    else if (key == GLUT_KEY_DOWN)
        {

        }
    else if (key == GLUT_KEY_UP)
        {
        }

    // Request display update
    glutPostRedisplay();
}

void keyboard_func(unsigned char key, int x, int y)
{
    switch (key)
    {
    case 0x08:
        // Process a backspace. 
        break;
    case 0x0A:
        // Process a linefeed. 
        break;
    case 0x1B:
        // Process an escape. 
        exit(0);
        break;
    case 0x09:
        // Process a tab. 
        break;
    case 0x0D:
        // Process a carriage return. 
        break;

    case 0x44:
    case 0x64:
        // Process D, d
        translate_camera_speed /= 1.2f;
        rotate_camera_speed /= 1.2f;
        break;

    case 0x45:
    case 0x65:
        // Process E, e
        break;
    case 0x46:
    case 0x66:
        // Process F, f
        interactive_mode = c_interactive_mode::filter_label;
        break;
    case 0x48:
    case 0x68:
        // Process H, h
        interactive_mode = c_interactive_mode::selecting_points_to_hide;
        min_visible = Vector2i(INT_MAX, INT_MAX);
        max_visible = Vector2i(INT_MIN, INT_MIN);
        break;
    case 0x49:
    case 0x69:
        // Process I, i
        display_text_info = !display_text_info;
        break;
    case 0x4B:
    case 0x6B:
        // Process K, k
        interactive_mode = c_interactive_mode::selecting_points_to_keep;
        min_visible = Vector2i(INT_MAX, INT_MAX);
        max_visible = Vector2i(INT_MIN, INT_MIN);
        break;

    case 0x50:
    case 0x70:
        // Process P, p
        gl_point_size += 1.0f;
        break;
    case 0x52:
    case 0x72:
        // Process R, r
        reset_settings();
        refresh_display_lists();
        break;

    case 0x53:
    case 0x73:
        // Process S, s
        if (output_point_cloud_file_path.size() > 0) {
            point_cloud.write_point_cloud_file(output_point_cloud_file_path);
        }
        break;

    case 0x55:
    case 0x75:
        // Process U, u
        translate_camera_speed *= 1.2f;
        rotate_camera_speed *= 1.2f;
        break;
    default:
        break;
    }

    // Request display update
    glutPostRedisplay();
}



void mouse_func(int button, int state,
                               int x, int y)
{
    if (button==GLUT_RIGHT_BUTTON && state == GLUT_DOWN) {
        last_mouse_pos_x = x;
        last_mouse_pos_y = y;

        mouse_right_button_up = false;
    }

    if (button==GLUT_RIGHT_BUTTON && state == GLUT_UP) {
        mouse_right_button_up = true;
    }

    if (button==GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
        last_mouse_pos_x = x;
        last_mouse_pos_y = y;

        mouse_left_button_up = false;
    }

    if (button==GLUT_LEFT_BUTTON && state == GLUT_UP) {
        if (interactive_mode == c_interactive_mode::selecting_points_to_hide) {
            interactive_mode = c_interactive_mode::updating_points_to_hide;
        }
        if (interactive_mode == c_interactive_mode::selecting_points_to_keep) {
            interactive_mode = c_interactive_mode::updating_points_to_keep;
        }
        
        mouse_left_button_up = true;
    }
}


void mouse_motion_func(int x, int y)
{
    if (interactive_mode == c_interactive_mode::selecting_points_to_hide ||
        interactive_mode == c_interactive_mode::selecting_points_to_keep
        ) {
        if (!mouse_left_button_up) {
            if (x < min_visible(0))
                min_visible(0) = x;
            if (y < min_visible(1))
                min_visible(1) = y;
            if (x > max_visible(0))
                max_visible(0) = x;
            if (y > max_visible(1))
                max_visible(1) = y;
        }
    } 
    else 
    if (interactive_mode == c_interactive_mode::idle) {
        if (!mouse_left_button_up) {
            Vector3f translation = Vector3f(-(last_mouse_pos_y - y), last_mouse_pos_x - x,0).cross(lookat_pnt-eye_pnt);
            translation.normalize();
            translation = -translation*translate_camera_speed;
            eye_pnt = eye_pnt + translation;
            lookat_pnt = lookat_pnt + translation;
        }
        else 
        if (!mouse_right_button_up) {
            Vector3f ang_vel = rotate_camera_speed*Vector3f((last_mouse_pos_y - y), -(last_mouse_pos_x - x), 0);
            // Rodrigues' formula
            Matrix3f ang_vel_hat;
            ang_vel_hat << 0.0, -ang_vel(2), ang_vel(1),
                ang_vel(2), 0.0, -ang_vel(0),
                -ang_vel(1), ang_vel(0), 0.0;
            float ang_vel_val = ang_vel.norm();

            if (ang_vel_val > 0) {
                //Matrix3f rotation = Matrix3f::Identity() + ang_vel_hat*sin(ang_vel_val) / ang_vel_val + ang_vel_hat*ang_vel_hat / (ang_vel_val*ang_vel_val)*(1 - cos(ang_vel_val));
                Matrix3f rotation = Matrix3f::Identity() + ang_vel_hat; // approximation for small ang_vel_val
                eye_pnt = lookat_pnt + rotation*(eye_pnt - lookat_pnt);
            }
        }
    }

    last_mouse_pos_x = x;
    last_mouse_pos_y = y;
}

void mouse_wheel_func( int wheel, int direction, int x, int y ) {
    int mult = 2;
    
    if (interactive_mode == c_interactive_mode::idle) {
        Vector3f translation = mult*(lookat_pnt - eye_pnt);
        translation.normalize();
        translation = translation*translate_camera_speed;
        eye_pnt = eye_pnt + translation;
        lookat_pnt = lookat_pnt + translation;
    }
}

int main(int argc, char **argv)
{
    vector< string > arg_list;

    reset_settings();

    for (int i = 1; i < argc; ++i) 
        arg_list.push_back(argv[i]);

    if (!get_command_line_options(arg_list))
        return 0;

    // Initialize GLUT and process user parameters
    glutInit(&argc, argv);

    // Request double buffered true color window with Z-buffer
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

    GLsizei win_width = 700;
    GLsizei win_height = 700;

    glutInitWindowSize(win_width,win_height);
    glutInitWindowPosition(100, 100);

    // Create window
    glutCreateWindow("OpenGL - PointCloudViewer");
    initializeGL(win_width, win_height);

    // Enable Z-buffer depth test
    glEnable(GL_DEPTH_TEST);

    // Callback functions
    glutDisplayFunc(drawScene);
    glutReshapeFunc(resize_viewport);

    glutSpecialFunc(special_keyboard_func);
    glutMouseFunc(mouse_func);
    glutKeyboardFunc(keyboard_func);    

    glutMotionFunc(mouse_motion_func);
    glutMouseWheelFunc (mouse_wheel_func);

    // Pass control to GLUT for events
    glutMainLoop();
    return 0;
}
