package org.firstinspires.ftc.teamcode.Libraries;

import static java.lang.Math.abs;
import static java.lang.Math.sqrt;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class WheelMath {
    private HardwareMap hardwareMap;

    public double power_factor; // All wheel powers will be scaled by this factor

    public IMU imu;

    public AprilTagDetector aprilTagDetector;

    public SparkFunOTOS otos;

    public void WheelMathSetUp(HardwareMap hardwareMap){
        this.power_factor = 1.0;

        // IMU initialization
        this.imu = hardwareMap.get(IMU.class, "imu");
        this.imu.resetYaw();

        // Apriltag detector initialization
        this.aprilTagDetector = new AprilTagDetector(hardwareMap);


        // OTOS initialization
        this.otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        this.otos.calibrateImu();
        this.otos.resetTracking();

        this.hardwareMap = hardwareMap;
    }

    // Positional information
    public double angularDirection(double current_angle, double target_angle) {
        double option_1 = target_angle - current_angle;
        double option_2 = option_1 - 360;

        if (abs(option_1) < abs(option_2)) {
            return ((option_1 + 180) % 360) - 180;
        } else {
            return ((option_2 + 180) % 360) - 180;
        }
    }

    public double getHeading() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return angles.getYaw();
    }

    public float getDeltaHeading() {
        AngularVelocity angles = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        return angles.zRotationRate;
    }

    public boolean isFacing(double target_yaw) {
        double current_heading = getHeading();
        boolean isFacing = abs(angularDirection(current_heading, target_yaw)) < 0.1 && abs(getDeltaHeading()) < 10;
        return isFacing;
    }

    SparkFunOTOS.Pose2D getPosition() {
        return otos.getPosition();
    }

    SparkFunOTOS.Pose2D getVelocity() {
        return otos.getVelocity();
    }

    public double distanceTo(double target_x, double target_y, double target_h, boolean check_heading) {
        SparkFunOTOS.Pose2D current_pos = getPosition();
        double dx = target_x - current_pos.x;
        double dy = target_y - current_pos.y;
        double dh = (check_heading ? (target_h - current_pos.h) / 180.0 : 0);

        return sqrt(dx*dx+dy*dy+dh*dh);
    }

    public double distanceTo(SparkFunOTOS.Pose2D pos, boolean check_heading) {
        return distanceTo(pos.x,pos.y,pos.h,check_heading);
    }
}
