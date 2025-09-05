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

    model << cos(rotation_angle / 180 * MY_PI), -sin(rotation_angle / 180 * MY_PI), 0, 0,
        sin(rotation_angle / 180 * MY_PI), cos(rotation_angle * MY_PI), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{

    float height = 2 * tan(eye_fov / 2 / 180 * MY_PI) * abs(zNear);
	float width = height * aspect_ratio;
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f projectionmove = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f projectionscale = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f projectionotho = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    projectionmove << 1, 0, 0, height / 2,
        0, 1, 0, width / 2,
        0, 0, 1, -(zNear + zFar) / 2,
		0, 0, 0, 1;

    projectionscale << 2/height, 0, 0, 0,
        0, 2/width, 0, 0,
        0, 0, abs(zNear - zFar)/2, 0,
		0, 0, 0, 1;

	projectionotho = projectionscale * projectionmove;

    projection << zNear, 0, 0, 0,
        0, zNear, 0, 0,
        0, 0, zNear + zFar, -zNear * zFar,
        0, 0, 1, 0;

	projection = projectionotho * projection;

    projection << zNear, 0, 0, 0,
        0, zNear, 0, 0,
        0, 0, zNear + zFar, -zNear * zFar,
        0, 0, 1, 0;

    return projection;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    Eigen::Matrix4f rotationToAxis = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f rotationback = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f rotation_angle = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();

    float a = axis[0];
    float b = axis[1];
    float c = axis[2];

    rotationToAxis << -b / (sqrt(a * a + b * b)), a / (sqrt(a * a + b * b)), 0, 0,
        -c / (sqrt(a * a + c * c)), 0, a / (sqrt(a * a + c * c)), 0,
        a / (sqrt(a * a + b * b + c * c)), b / (sqrt(a * a + b * b + c * c)), c / (sqrt(a * a + b * b + c * c)), 0,
        0, 0, 0, 1;

    rotationback = rotationToAxis.inverse();

    rotation_angle << cos(angle / 180 * MY_PI), -sin(angle / 180 * MY_PI), 0, 0,
        sin(angle / 180 * MY_PI), cos(angle / 180 * MY_PI), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    rotation = rotationback * rotation_angle * rotationToAxis;

    return rotation;
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
