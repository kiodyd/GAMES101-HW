#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>

// 更详细的解释：https://zhuanlan.zhihu.com/p/453150407

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

Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    float rad = angle * MY_PI / 180;
    Eigen::Matrix4f I, N, Rod;
    Eigen::Vector4f axi;
    Eigen::RowVector4f taxi;

    I << 1,0,0,0,
         0,1,0,0,
         0,0,1,0,
         0,0,0,1;

    N << 0,-axis.z(),axis.y(),0,
         axis.z(),0,-axis.x(),0,
         -axis.y(),axis.x(),0,0,
         0,0,0,1;

    axi << axis.x(),axis.y(),axis.z(),0;
    taxi << axis.x(),axis.y(),axis.z(),0;

    Rod = std::cosf(rad) * I + (1 - std::cosf(rad)) * axi * taxi + std::sinf(rad) * N;
    Rod(3,3) = 1;
    return Rod;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
//    Eigen::Matrix4f ratate;
//    float rotation_rad = rotation_angle * MY_PI / 180;
//    ratate << std::cosf(rotation_rad), -std::sinf(rotation_rad), 0, 0,
//              std::sinf(rotation_rad), std::cosf(rotation_rad),0,0,
//              0,0,1,0,
//              0,0,0,1;

    model = get_rotation({0,0,1}, rotation_angle) * model;
    // Then return it.

    return model;
}


Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    float h, w;
    h = 2 * std::tanf(eye_fov * MY_PI / 180 /2);
    w = h * aspect_ratio;

    Eigen::Matrix4f matrix0, matrix1, matrix2;
    // 将平截头体变换到长方体
    matrix0 << zNear,0,0,0,
               0,zNear,0,0,
               0,0,zNear+zFar,-zNear*zFar,
               0,0,1,0;
    // 做正交投影，这里不需要平移
    matrix1 << 2/w,0,0,0,
               0,2/h,0,0,
               0,0,2/(zNear-zFar),0,
               0,0,0,1;
    // 处理下镜像问题
    matrix2 << -1,0,0,0,
               0,-1,0,0,
               0,0,1,0,
               0,0,0,1;
    projection = matrix2 * matrix1 * matrix0 * projection;
    // Then return it.

    return projection;
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
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
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
