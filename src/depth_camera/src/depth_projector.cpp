#include <cmath>
#include <cstring>

extern "C" void project_lidar_to_depth(
    const float* cloud, int num_points,
    int image_width, int image_height,
    const float* odom, float camera_offset_z,
    float* depth_output)
{
    float x0 = odom[0], y0 = odom[1], z0 = odom[2];
    float roll = odom[3], pitch = odom[4], yaw = odom[5];

    float cos_r = cos(roll), sin_r = sin(roll);
    float cos_p = cos(pitch), sin_p = sin(pitch);
    float cos_y = cos(yaw), sin_y = sin(yaw);

    memset(depth_output, 0, sizeof(float) * image_width * image_height);

    for (int i = 0; i < num_points; ++i) {
        float x = cloud[3 * i];
        float y = cloud[3 * i + 1];
        float z = cloud[3 * i + 2];

        // 1. shift
        float x1 = x - x0, y1 = y - y0, z1 = z - z0;

        // 2. yaw
        float x2 = x1 * cos_y + y1 * sin_y;
        float y2 = -x1 * sin_y + y1 * cos_y;
        float z2 = z1;

        // 3. pitch
        float x3 = x2 * cos_p - z2 * sin_p;
        float y3 = y2;
        float z3 = x2 * sin_p + z2 * cos_p;

        // 4. roll
        float x4 = x3;
        float y4 = y3 * cos_r + z3 * sin_r;
        float z4 = -y3 * sin_r + z3 * cos_r - camera_offset_z;

        float hori_dis = sqrtf(x4 * x4 + y4 * y4);
        if (hori_dis < 1e-5f) continue;

        int u = (int)(-image_width / (2 * M_PI) * atan2f(y4, x4) + image_width / 2);
        int v = (int)(-image_height / (1 * M_PI) * atanf(z4 / hori_dis) + image_height / 2);

        if (u >= 0 && u < image_width && v >= 0 && v < image_height) {
            float depth = sqrtf(x * x + y * y + z * z);
            int idx = v * image_width + u;
            if (depth_output[idx] == 0 || depth < depth_output[idx]) {
                depth_output[idx] = depth;
            }
        }
    }
}
