package org.firstinspires.ftc.teamcode.Libraries;

import static java.lang.Math.abs;
import static java.lang.Math.min;
import static java.lang.Math.sqrt;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Math.DegTrig;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@SuppressWarnings("unused")
public class DriveBase {
    private HardwareMap hardwareMap;

    public DcMotor front_right;
    public DcMotor front_left;
    public DcMotor back_right;
    public DcMotor back_left;
    public double power_factor; // All wheel powers will be scaled by this factor

    public IMU imu;

    public AprilTagDetector aprilTagDetector;

    public SparkFunOTOS otos;

    public ServoLED led;

    // Constructor
    public DriveBase(HardwareMap hardwareMap)  {
        this.front_right = hardwareMap.get(DcMotor.class, "frontright");
        this.front_left = hardwareMap.get(DcMotor.class, "frontleft");
        this.back_right = hardwareMap.get(DcMotor.class, "backright");
        this.back_left = hardwareMap.get(DcMotor.class, "backleft");

        // Reverse left motors so all motors spin same direction
        this.front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        this.back_left.setDirection(DcMotorSimple.Direction.REVERSE);

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

        // Color initialization
        led = new ServoLED(hardwareMap, "LED");

        this.hardwareMap = hardwareMap;
    }

    // Raw wheel control
    public void setWheelPowers(double Front_Right_Power, double Front_Left_Power, double Back_Right_Power, double Back_Left_Power) {
        this.front_right.setPower(Front_Right_Power);
        this.front_left.setPower(Front_Left_Power);
        this.back_right.setPower(Back_Right_Power);
        this.back_left.setPower(Back_Left_Power);
    }
    public void stop() {
        setWheelPowers(0,0,0,0);
    }
    public void turn(double speed) {
        double fl = front_left.getPower() - speed;
        double fr = front_right.getPower() + speed;
        double bl = front_left.getPower() - speed;
        double br = front_left.getPower() + speed;
        setWheelPowers(fr,fl,br,bl);
    }
    public void omniMove(double Forward, double Right, double Rotate) {
        // THIS HAS BEEN FINICKY
        double fl = Forward + Right - Rotate;
        double fr = Forward - Right + Rotate;
        double bl = Forward - Right - Rotate;
        double br = Forward + Right + Rotate;

        // normalize so no value exceeds 1
        double max = Math.max(1.0, Math.max(abs(fl),
                Math.max(abs(fr), Math.max(abs(bl), abs(br)))));

        fl /= max;
        fr /= max;
        bl /= max;
        br /= max;

        this.setWheelPowers(fr * power_factor, fl * power_factor, br * power_factor, bl * power_factor);
    }
    public void omniMoveController(Gamepad gamepad) {
        double forward = - gamepad.left_stick_y;
        double strafe = gamepad.left_stick_x;
        double turn = - gamepad.right_stick_x;
        this.omniMove(forward,strafe,turn);
    }

    // Positional information
    private double angularDirection(double current_angle, double target_angle) {
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
        return abs(angularDirection(current_heading, target_yaw)) < 0.1 && abs(getDeltaHeading()) < 10;
    }
    SparkFunOTOS.Pose2D getPosition() {
        return otos.getPosition();
    }
    SparkFunOTOS.Pose2D getVelocity() {
        return otos.getVelocity();
    }

    // Reset sensors
    public void resetHeading() {
        this.imu.resetYaw();
    }
    public void resetOtos() {
        otos.resetTracking();
    }

    public void pointTowards(double target_yaw) {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        double current_yaw = angles.getYaw();
        double power = angularDirection(current_yaw+getDeltaHeading()/20.0,target_yaw) / 20.0;
        this.omniMove(0.0,0.0,power);
    }
    public void globalOmniMove(double Forward, double Right, double Rotate) {
        double heading = getHeading();
        double c = DegTrig.cosDeg(heading);
        double s = DegTrig.sinDeg(heading);
        double forward_rotated = Forward * c + Right * s;
        double right_rotated = Forward * s - Right * c;
        omniMove(forward_rotated,right_rotated,Rotate);
    }
    public void globalOmniMoveController(Gamepad gamepad) {
        double forward = - gamepad.left_stick_y;
        double strafe = gamepad.left_stick_x;
        double turn = - gamepad.right_stick_x;
        this.globalOmniMove(forward,strafe,turn);
    }
    public boolean searchForAprilTag(int id, double search_speed) {
        if (aprilTagDetector.last_check.milliseconds() > 100) aprilTagDetector.detect();
        if (aprilTagDetector.findByID(id) != null) {
            stop();
            return true;
        }
        omniMove(0,0,search_speed);
        return false;
    }

    /**
     * @param id: id of target apriltag
     * @return whether it sees the apriltag, will do nothing if it can't see it
     */
    public boolean lookAtApriltag(int id) {
        for (AprilTagDetection detection : aprilTagDetector.current_detections) {
            if (detection.id == id) {
                double target = detection.ftcPose.bearing;
                double turn =  (target - getDeltaHeading() / 20.0) / 18.0;
                omniMove(0,0,turn);
                return true;
            }
        }
        return false;
    }
    public void moveToPosition(SparkFunOTOS.Pose2D target_pos) {
        SparkFunOTOS.Pose2D current_pos = getPosition();
        double x_diff = target_pos.x - current_pos.x;
        double y_diff = target_pos.y - current_pos.y;
        double h_diff = target_pos.h - current_pos.h;

        globalOmniMove(y_diff/10.0,x_diff/10.0,h_diff/10.0);
    }
    double distanceTo(SparkFunOTOS.Pose2D target_pos, boolean check_heading) {
        SparkFunOTOS.Pose2D current_pos = getPosition();
        double dx = target_pos.x - current_pos.x;
        double dy = target_pos.y - current_pos.y;
        double dh = (check_heading ? (target_pos.h - current_pos.h) / 180.0 : 0);

        return sqrt(dx*dx+dy*dy+dh*dh);
    }
    void telemetryPosition(Telemetry telemetry) {
        SparkFunOTOS.Pose2D pose = getPosition();
        telemetry.addData("X", pose.x);
        telemetry.addData("Y", pose.y);
        telemetry.addData("Heading", pose.h);
    }
}
