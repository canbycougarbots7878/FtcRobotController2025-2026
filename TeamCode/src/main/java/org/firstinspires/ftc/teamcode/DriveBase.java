package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
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

    DriveBase(HardwareMap hardwareMap)  {
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

        // Apriltag detector initialization
        this.aprilTagDetector = new AprilTagDetector(hardwareMap);

        this.hardwareMap = hardwareMap;
    }
    public void setWheelPowers(double Front_Right_Power, double Front_Left_Power, double Back_Right_Power, double Back_Left_Power) {
        this.front_right.setPower(Front_Right_Power);
        this.front_left.setPower(Front_Left_Power);
        this.back_right.setPower(Back_Right_Power);
        this.back_left.setPower(Back_Left_Power);
    }
    public void stop() {
        setWheelPowers(0,0,0,0);
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

    public double getHeading() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return angles.getYaw();
    }
    public float getDeltaHeading() {
        AngularVelocity angles = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        return angles.zRotationRate;
    }
    public void pointTowards(double target_yaw) {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        double current_yaw = angles.getYaw();
        double power = (target_yaw - (current_yaw + getDeltaHeading() / 20.0)) / 20.0;
        this.omniMove(0.0,0.0,power);
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
        AprilTagDetection apriltag = aprilTagDetector.findByID(id);
        if(apriltag == null) return false; // Return false to signal that the apriltag wasn't detected

        double target = apriltag.ftcPose.yaw;
        double turn =  target / 18.0;
        omniMove(0,0,turn);

        return true;
    }
}
