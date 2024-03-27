#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    Eigen::Matrix4f z_rot;
    rotation_angle = rotation_angle/180.0*MY_PI;
    z_rot << std::cos(rotation_angle), -std::sin(rotation_angle),0,0,
            std::sin(rotation_angle),std::cos(rotation_angle),0,0,
            0,0,1,0,
            0,0,0,1;

    model = z_rot * model;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    float n,f,t,b,l,r;
    //opengl: 0 > n > f
    n = -zNear;
    f = -zFar;
    //45度 1：1 0.1-50 eye_FOV应该是y的吧
    float angel_y = eye_fov/180.0*MY_PI/2;
    t = zNear*std::tan(angel_y);
    b = -t;
    // aspect_radio = 宽/高  宽 = 高*aspect——radio
    r = t*aspect_ratio;
    l = -r;

    Eigen::Matrix4f mat_ortho_0;
    Eigen::Matrix4f mat_ortho_1;
    Eigen::Matrix4f mat_ortho;
    mat_ortho_0 << 2/(r-l),0,0,0,
                   0,2/(t-b),0,0,
                   0,0,2/(n-f),0,
                   0,0,0,1;
    mat_ortho_1 << 1,0,0,-(r+l)/2,
                   0,1,0,-(t+b)/2,
                   0,0,1,-(n+f)/2,
                   0,0,0,1;    
    mat_ortho = mat_ortho_0 * mat_ortho_1;
    Eigen::Matrix4f mat_proj;
    mat_proj<< n,0,0,0,
               0,n,0,0,
               0,0,n+f,-n*f,
               0,0,1,0;

    projection = mat_ortho * mat_proj;
    return projection;
}
using std::cos,std::sin;

Eigen::Matrix4f get_rotation(Vector3f axis, float angle){
    //绕任意轴的旋转矩阵
    float a = angle*180.0/MY_PI;
    Eigen::Matrix4f rot_mat;
    // axis https://zhuanlan.zhihu.com/p/462935097 感觉抄这个公式就行了，然后问题是vector3f可以取索引吗，可以的

    rot_mat  << cos(a) +(1-cos(a))*axis[0]*axis[0],(1-cos(a))*axis[0]*axis[1]-sin(a)*axis[2],(1-cos(a))*axis[0]*axis[2]+sin(a)*axis[1],0,
     (1-cos(a))*axis[0]*axis[1]+sin(a)*axis[2],cos(a) +(1-cos(a))*axis[1]*axis[1],(1-cos(a))*axis[2]*axis[1]-sin(a)*axis[0],0,
     (1-cos(a))*axis[0]*axis[2]-sin(a)*axis[1],(1-cos(a))*axis[2]*axis[1]+sin(a)*axis[0],cos(a) +(1-cos(a))*axis[2]*axis[2],0,
     0,0,0,1;
    
    return rot_mat;
}
int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{1, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        r.set_model(get_model_matrix(angle));
        // r.set_model(get_rotation(Vector3f(1,0,0),angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        // r.set_model(get_rotation(Vector3f(1,0,0),angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
