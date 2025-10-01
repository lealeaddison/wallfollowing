#include <iostream>
#include <cmath>

#include <signal.h>

#include <mbot_bridge/robot.h>
#include <wall_follower/common/utils.h>

bool ctrl_c_pressed = false;
void ctrlc(int){ ctrl_c_pressed = true; }

// tiny helper
static void sleepFor(double secs){
  std::this_thread::sleep_for(std::chrono::duration<double>(secs));
}

int main(int, const char**)
{
    signal(SIGINT,  ctrlc);
    signal(SIGTERM, ctrlc);

    mbot_bridge::MBot robot;

    // Lidar data containers (filled by the robot API)
    std::vector<float> ranges;
    std::vector<float> thetas;

    // ====== Tunables (start conservative) ======
    const double setpoint_m   = 0.35;   // desired distance from wall (m)
    const double deadband_m   = 0.03;   // don't correct if |error| < deadband
    const double v_tangent    = 0.20;   // m/s along the wall
    const double Kp_normal    = 0.80;   // m/s per meter of error (P gain)
    const double vmax         = 0.30;   // speed clamp (safety)
    const double loop_dt      = 0.05;   // 20 Hz control loop
    // ===========================================

    while (!ctrl_c_pressed) {
        // 1) Read sensors
        robot.readLidarScan(ranges, thetas);
        if (ranges.empty() || thetas.empty() || ranges.size() != thetas.size()){
            robot.stop(); sleepFor(loop_dt); continue;
        }

        // 2) Find closest valid ray (index)
        int idx = findMinDist(ranges);
        if (idx < 0){ // no valid ray: stop defensively
            robot.stop(); sleepFor(loop_dt); continue;
        }

        // 3) Extract distance and angle of that ray
        const double d   = ranges[idx];     // meters
        const double ang = thetas[idx];     // radians

        // 4) Build wall normal (points from robot toward wall)
        //    n = [cos(theta), sin(theta), 0]
        std::vector<float> n = {static_cast<float>(std::cos(ang)),
                                static_cast<float>(std::sin(ang)),
                                0.0f};

        // 5) Tangent direction along wall: t = z × n, where z = [0,0,1]
        std::vector<float> z = {0.0f, 0.0f, 1.0f};
        std::vector<float> t = crossProduct(z, n); // = [-ny, nx, 0]

        // Normalize tangent in XY
        double tmag = std::hypot(t[0], t[1]);
        double tx = (tmag > 1e-6) ? (t[0]/tmag) : 1.0;
        double ty = (tmag > 1e-6) ? (t[1]/tmag) : 0.0;

        // 6) Base command = move along the wall (tangent)
        double vx = v_tangent * tx;
        double vy = v_tangent * ty;

        // 7) P-correction on normal to hold setpoint distance
        double err = d - setpoint_m;           // >0 means too far from wall
        if (std::fabs(err) > deadband_m){
            // negative sign: if too far (+err), push toward wall (-n)
            vx += -Kp_normal * err * n[0];
            vy += -Kp_normal * err * n[1];
        }

        // 8) Clamp speeds for safety
        auto clamp = [&](double u){ return std::max(-vmax, std::min(vmax, u)); };
        vx = clamp(vx);
        vy = clamp(vy);

        // 9) Drive (no in-place turning needed here)
        robot.drive(vx, vy, 0.0);

        sleepFor(loop_dt);
    }

    robot.stop();
    return 0;
}