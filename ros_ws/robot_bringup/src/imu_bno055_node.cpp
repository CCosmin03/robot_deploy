#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdint>
#include <cmath>
#include <stdexcept>

// ── BNO055 register map ───────────────────────────────────────────────────────
static const uint8_t REG_OPR_MODE    = 0x3D;
static const uint8_t REG_SYS_TRIGGER = 0x3F;
static const uint8_t REG_CALIB_STAT  = 0x35;
static const uint8_t REG_QUA_W_LSB   = 0x20;  // w,x,y,z — 4×int16, scale /16384
static const uint8_t REG_GYR_X_LSB   = 0x14;  // x,y,z   — 3×int16, 16 LSB/dps
static const uint8_t REG_LIA_X_LSB   = 0x28;  // x,y,z   — 3×int16, 100 LSB/(m/s²)

static const uint8_t MODE_CONFIG  = 0x00;
static const uint8_t MODE_IMUPLUS = 0x08;  // accel + gyro, heading relativ
static const uint8_t MODE_NDOF    = 0x0C;  // accel + gyro + mag, heading absolut

static const double DEG2RAD = M_PI / 180.0;

// ── I2C helpers ───────────────────────────────────────────────────────────────
static int g_fd = -1;

static void i2c_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    if (write(g_fd, buf, 2) != 2)
        throw std::runtime_error("I2C write failed");
}

static void i2c_read(uint8_t reg, uint8_t* out, int len)
{
    if (write(g_fd, &reg, 1) != 1)
        throw std::runtime_error("I2C register select failed");
    if (read(g_fd, out, len) != len)
        throw std::runtime_error("I2C read failed");
}

static inline int16_t as_s16(const uint8_t* buf, int offset)
{
    return static_cast<int16_t>(buf[offset] | (buf[offset + 1] << 8));
}

// ── BNO055 init ───────────────────────────────────────────────────────────────
static void bno055_init(uint8_t mode)
{
    i2c_write(REG_OPR_MODE, MODE_CONFIG);
    ros::Duration(0.025).sleep();
    i2c_write(REG_SYS_TRIGGER, 0x20);   // software reset
    ros::Duration(0.700).sleep();        // BNO055 needs ~650 ms after reset
    i2c_write(REG_OPR_MODE, mode);
    ros::Duration(0.020).sleep();

    // BNO055 cu pinii spre fata: X_fizic=dreapta(-robot_Y), Y_fizic=inainte(+robot_X), Z_fizic=sus
    // Remap: out_X <- phys_Y(+robot_X), out_Y <- -phys_X(+robot_Y), out_Z <- phys_Z
    // AXIS_MAP_CONFIG 0x41: bits[1:0]=01(Y), bits[3:2]=00(X), bits[5:4]=10(Z) → 0x21
    // AXIS_MAP_SIGN  0x42: bit1=1 (negate out_Y = negate phys_X) → 0x02
    i2c_write(0x3D, 0x00);          // CONFIG mode pentru axis remapping
    ros::Duration(0.025).sleep();
    i2c_write(0x41, 0x21);          // swap out_X←phys_Y, out_Y←phys_X
    i2c_write(0x42, 0x02);          // negate out_Y
    i2c_write(REG_OPR_MODE, mode);
    ros::Duration(0.020).sleep();

    if (mode == MODE_NDOF)
        ROS_INFO("BNO055: NDOF mode — accel + gyro + magnetometer (heading absolut)");
    else
        ROS_INFO("BNO055: IMUPLUS mode — accel + gyro, fara magnetometru (heading relativ)");
}

// ── main ──────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_bno055_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string i2c_dev;
    std::string imu_mode_str;
    int         i2c_addr;
    int         rate_hz;
    std::string frame_id;

    pnh.param<std::string>("i2c_dev",   i2c_dev,       "/dev/i2c-1");
    pnh.param<int>        ("i2c_addr",  i2c_addr,      0x28);
    pnh.param<std::string>("imu_mode",  imu_mode_str,  "ndof");
    pnh.param<int>        ("rate_hz",   rate_hz,        50);
    pnh.param<std::string>("frame_id",  frame_id,      "imu_link");

    uint8_t imu_mode = (imu_mode_str == "imuplus") ? MODE_IMUPLUS : MODE_NDOF;

    // Open I2C bus
    g_fd = open(i2c_dev.c_str(), O_RDWR);
    if (g_fd < 0)
    {
        ROS_FATAL("Nu pot deschide %s", i2c_dev.c_str());
        return 1;
    }
    if (ioctl(g_fd, I2C_SLAVE, i2c_addr) < 0)
    {
        ROS_FATAL("Nu pot seta adresa I2C slave 0x%02X", i2c_addr);
        close(g_fd);
        return 1;
    }

    try { bno055_init(imu_mode); }
    catch (const std::exception& e)
    {
        ROS_FATAL("BNO055 init esuat: %s", e.what());
        close(g_fd);
        return 1;
    }

    ros::Publisher pub = nh.advertise<sensor_msgs::Imu>("imu", 10);
    ros::Rate loop_rate(rate_hz);

    sensor_msgs::Imu msg;
    msg.header.frame_id = frame_id;

    while (ros::ok())
    {
        try
        {
            uint8_t buf[8];

            // Quaternion: w, x, y, z
            i2c_read(REG_QUA_W_LSB, buf, 8);
            msg.orientation.w = as_s16(buf, 0) / 16384.0;
            msg.orientation.x = as_s16(buf, 2) / 16384.0;
            msg.orientation.y = as_s16(buf, 4) / 16384.0;
            msg.orientation.z = as_s16(buf, 6) / 16384.0;

            // Gyroscop: dps → rad/s
            i2c_read(REG_GYR_X_LSB, buf, 6);
            msg.angular_velocity.x = as_s16(buf, 0) / 16.0 * DEG2RAD;
            msg.angular_velocity.y = as_s16(buf, 2) / 16.0 * DEG2RAD;
            msg.angular_velocity.z = as_s16(buf, 4) / 16.0 * DEG2RAD;

            // Acceleratie liniara (fara gravitatie): m/s²
            i2c_read(REG_LIA_X_LSB, buf, 6);
            msg.linear_acceleration.x = as_s16(buf, 0) / 100.0;
            msg.linear_acceleration.y = as_s16(buf, 2) / 100.0;
            msg.linear_acceleration.z = as_s16(buf, 4) / 100.0;

            // Covariante in functie de statusul calibrarii
            uint8_t calib;
            i2c_read(REG_CALIB_STAT, &calib, 1);
            double cov_ori = ((calib >> 6) & 0x03) >= 2 ? 1e-4 : 1e-1;
            double cov_gyr = ((calib >> 4) & 0x03) >= 2 ? 1e-4 : 1e-1;
            double cov_acc = 1e-3;

            msg.orientation_covariance         = {cov_ori,0,0, 0,cov_ori,0, 0,0,cov_ori};
            msg.angular_velocity_covariance    = {cov_gyr,0,0, 0,cov_gyr,0, 0,0,cov_gyr};
            msg.linear_acceleration_covariance = {cov_acc,0,0, 0,cov_acc,0, 0,0,cov_acc};

            msg.header.stamp = ros::Time::now();
            pub.publish(msg);
        }
        catch (const std::exception& e)
        {
            ROS_WARN_THROTTLE(5.0, "IMU read error: %s", e.what());
        }

        loop_rate.sleep();
    }

    close(g_fd);
    return 0;
}
