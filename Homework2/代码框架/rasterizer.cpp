// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const std::array<Vector4f, 3> &_v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    int TRIANGLE_POINT_SIZE = 3;
    Vector3f lastV;
    for (int i = 0; i < TRIANGLE_POINT_SIZE; ++i) {
        int next_index = (i + 1) % TRIANGLE_POINT_SIZE;
        Vector3f v1 = {_v[next_index].x() - _v[i].x(), _v[next_index].y() - _v[i].y(), 0};
        Vector3f v2 = {x - _v[i].x(), y - _v[i].y(), 0};
        Vector3f r1 = v1.cross(v2);
        if (i != 0 && r1.dot(lastV) < 0)
            return false;
        lastV = r1;
    }
    return true;

//    Eigen::Vector2f p;      //检测目标
//    p << x, y;
//
//    //.head(2)指这个点的前两个数值，即x,y
//    Eigen::Vector2f a, b, c;   //被检测的三角形三边向量
//    a = _v[0].head(2) - _v[1].head(2);          //a = A - B  即B->A
//    b = _v[1].head(2) - _v[2].head(2);          //b = B - C  即C->B
//    c = _v[2].head(2) - _v[0].head(2);          //c = C - A  即A->C
//
//    Eigen::Vector2f AP, BP, CP;
//    AP = p - _v[0].head(2);
//    BP = p - _v[1].head(2);
//    CP = p - _v[2].head(2);
//
//    //由于我这里的向量方向都是逆时针的，所以如果点p在内，那么所有边向量叉乘对应的XP都应该为正值，指向z的正半轴。
//    return AP[0] * c[1] - AP[1] * c[0] > 0 && BP[0] * a[1] - BP[1] * a[0] > 0 && CP[0] * b[1] - CP[1] * b[0] > 0;
}

static bool insideSampleTriangle(int x, int y, const std::array<Vector4f, 3> &_v, std::vector<bool> &sample_inside, const std::vector<Eigen::Vector2f> &sample_pos) {
    bool result = false;
    for (int i = 0; i < sample_inside.size(); ++i) {
        float sampleX = x + sample_pos[i].x();
        float sampleY = y + sample_pos[i].y();
        bool ret = insideTriangle(sampleX, sampleY, _v);
        sample_inside[i] = ret;
        result = result || ret;
    }
    return result;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    float right = std::max(v[0].x(), std::max(v[1].x(), v[2].x()));
    float left = std::min(v[0].x(), std::min(v[1].x(), v[2].x()));
    float top = std::max(v[0].y(), std::max(v[1].y(), v[2].y()));
    float bottom = std::min(v[0].y(), std::min(v[1].y(), v[2].y()));

    right = static_cast<int>(std::ceil(right));
    left = static_cast<int>(std::floor(left));
    top = static_cast<int>(std::ceil(top));
    bottom = static_cast<int>(std::floor(bottom));

    // iterate through the pixel and find if the current pixel is inside the triangle
    std::vector<Eigen::Vector2f> sample_pos
    {
        {0.25,0.25},                        //左下
        {0.75,0.25},                        //右下
        {0.25,0.75},                        //左上
        {0.75,0.75}                         //右上
    };
    for (int y = bottom; y <= top; ++y) {
        for (int x = left; x <= right; ++x) {
            std::vector<bool> sample_inside {false, false, false, false};
            int pass_count = 0;
            if (insideSampleTriangle(x, y, v, sample_inside, sample_pos)) {
                for (int i = 0; i < sample_inside.size(); ++i) {
                    if (sample_inside[i]) {
                        // If so, use the following code to get the interpolated z value.
                        float sample_x = x + sample_pos[i].x();
                        float sample_y = y + sample_pos[i].y();
                        auto[alpha, beta, gamma] = computeBarycentric2D(sample_x, sample_y, t.v);
                        float w_reciprocal = 1.0f/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;
                        int point_index = get_super_index(x, y, i);
                        if (z_interpolated < super_depth_buf[point_index]) {
                            super_depth_buf[point_index] = z_interpolated;
                            supper_frame_buf[point_index] = t.getColor();
                            pass_count++;
                        }
                    }
                }


                if (pass_count > 0){
                    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
                    Eigen::Vector3f color;
                    color = supper_frame_buf[get_super_index(x,y,0)] + supper_frame_buf[get_super_index(x,y,1)] + supper_frame_buf[get_super_index(x,y,2)] + supper_frame_buf[get_super_index(x,y,3)];
                    color /= 4;
                    set_pixel({x, y, 0}, color);
                }
            }
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        std::fill(supper_frame_buf.begin(), supper_frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        float inf = std::numeric_limits<float>::infinity();
        std::fill(depth_buf.begin(), depth_buf.end(), inf);
        std::fill(super_depth_buf.begin(), super_depth_buf.end(), inf);
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    supper_frame_buf.resize(w * h * 4);
    super_depth_buf.resize(w * h * 4);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

int rst::rasterizer::get_super_index(int x, int y, int n)
{
    return (height-1-y)*width * 4 + x * 4 + n;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;
}

// clang-format on