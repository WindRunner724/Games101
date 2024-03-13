#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;
constexpr float coff = MY_PI / 180.0f;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 
        1, 0, 0, -eye_pos[0],
        0, 1, 0, -eye_pos[1],
        0, 0, 1, -eye_pos[2],
        0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    Eigen::Matrix4f translate;
    translate <<
        std::cos(rotation_angle * coff), -std::sin(rotation_angle * coff), 0, 0,
        std::sin(rotation_angle * coff), std::cos(rotation_angle * coff), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    model = translate * model;

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
    Eigen::Matrix4f per_projection = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f mov_projection = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f sca_projection = Eigen::Matrix4f::Identity();
    per_projection <<
        zNear, 0, 0, 0,
        0, zNear, 0, 0,
        0, 0, zNear + zFar, -zNear*zFar,
        0, 0, -1, 0;
    mov_projection <<
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, (zFar+zNear)/2,
        0, 0, 0, 1;
    sca_projection <<
        1 / (zNear * std::tan(eye_fov / 2 * coff) * aspect_ratio), 0, 0, 0,
        0, 1 / (zNear * std::tan(eye_fov / 2 * coff)), 0, 0,
        0, 0, 2 / (zFar - zNear), 0,
        0, 0, 0, 1;

    projection = sca_projection * mov_projection * per_projection * projection;

    return projection;
}

Eigen::Vector3f get_nomalVec(Vector3f vec) {
    float mag = std::sqrt(std::pow(vec[0], 2) + std::pow(vec[1], 2) + std::pow(vec[2], 2));
    return vec / mag;
}

Eigen::Vector4f to_4fVec(Vector3f vec) {
	return Vector4f(vec.x(), vec.y(), vec.z(), 0.0);
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle) {
    /*Eigen::AngleAxisf rotation(angle, axis);
    return rotation.toRotationMatrix();*/
    Eigen::Matrix4f rotation;

    Eigen::Vector3f new_z = get_nomalVec(Vector3f(axis[0], axis[1], axis[2]));
    Eigen::Vector3f new_y = get_nomalVec({0.0, -new_z[2], new_z[1]});
    Eigen::Vector3f new_x = get_nomalVec(new_y.cross(new_z));

    Eigen::Matrix4f axis_trans;
	Eigen::Matrix4f rota_trans;
	Eigen::Matrix4f axis_inverse_trans;

    axis_trans << to_4fVec(new_x), to_4fVec(new_y), to_4fVec(new_z), Vector4f(0.0, 0.0, 0.0, 1.0);
    rota_trans <<
        std::cos(coff * angle), -std::sin(coff * angle), 0.0, 0.0,
        std::sin(coff * angle), -std::cos(coff * angle), 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0;
    axis_inverse_trans = axis_trans.inverse();

    rotation = axis_inverse_trans * rota_trans * axis_trans;
    return rotation;
}

Eigen::Matrix3f get_rotation_v1(Vector3f axis, float angle) {
	Eigen::AngleAxisf rotation(coff * angle, axis);
	return rotation.toRotationMatrix();
}

Eigen::Matrix4f get_rotation_v2(Vector3f axis, float angle) {
	Eigen::Matrix4f Result = Eigen::Matrix4f::Identity();
	Eigen::Matrix3f E = Eigen::Matrix3f::Identity();
	Eigen::Matrix3f N = Eigen::Matrix3f::Identity();//axis vector' cross matrix
	Eigen::Matrix3f ResultMat3 = Eigen::Matrix3f::Identity();

    N <<
        0, -axis[2], axis[1],
        axis[2], 0, -axis[0],
        -axis[1], axis[0], 0;
    ResultMat3 = E * std::cos(coff * angle) + (1 - cos(coff * angle)) * axis * axis.transpose() + sin(coff * angle) * N;
    Result <<
        ResultMat3(0, 0), ResultMat3(0, 1), ResultMat3(0, 2), 0,
        ResultMat3(1, 0), ResultMat3(1, 1), ResultMat3(1, 2), 0,
        ResultMat3(2, 0), ResultMat3(2, 1), ResultMat3(2, 2), 0,
        0, 0, 0, 1;//complement 3f matrix to 4f
    return Result;
}

int main(int argc, const char** argv) 
{
    float angle = 0;

    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default. Transfer string to float
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}}; // verticals pos

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos); //auto adapting variable type
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);//Clear frame and depth buffer

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
        //r.set_model(get_rotation_v1(Eigen::Vector3f(1.0, 0.0, 0.0), angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection( get_projection_matrix(45, 1, 0.1, 50));

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
