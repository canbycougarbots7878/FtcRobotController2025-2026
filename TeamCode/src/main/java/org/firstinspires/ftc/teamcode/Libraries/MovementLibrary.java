package org.firstinspires.ftc.teamcode.Libraries;

import static java.lang.Math.abs;
import static java.lang.Math.hypot;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Math.DegTrig;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class MovementLibrary {
    WheelControl wheelControl;
    WheelMath wheelMath;

    public void MovementLibraryCreation(HardwareMap hardwareMap){
        wheelControl.WheelCreation(hardwareMap);
        wheelMath.WheelMathSetUp(hardwareMap);
    }

    public void stop() {
        wheelControl.setWheelPowers(0,0,0,0);
    }

    public void turn(double speed) {
        double frontLeft = wheelControl.front_left.getPower() - speed;
        double frontRight = wheelControl.front_right.getPower() + speed;
        double backLeft = wheelControl.front_left.getPower() - speed;
        double backRight = wheelControl.front_left.getPower() + speed;
        wheelControl.setWheelPowers(frontRight,frontLeft,backRight,backLeft);
    }

    public void omniMove(double Forward, double Right, double Rotate) {
        // THIS HAS BEEN FINICKY
        double frontLeft = Forward + Right - Rotate;
        double frontRight = Forward - Right + Rotate;
        double backLeft = Forward - Right - Rotate;
        double backRight = Forward + Right + Rotate;

        // normalize so no value exceeds 1
        double max = Math.max(1.0, Math.max(abs(frontLeft), Math.max(abs(frontRight), Math.max(abs(backLeft), abs(backRight)))));

        frontLeft /= max;
        frontRight /= max;
        backLeft /= max;
        backRight /= max;

        wheelControl.setWheelPowers(frontRight * wheelMath.power_factor, frontLeft * wheelMath.power_factor, backRight * wheelMath.power_factor, backLeft * wheelMath.power_factor);
    }

    public void omniMoveController(Gamepad gamepad) {
        double forward = - gamepad.left_stick_y;
        double strafe = gamepad.left_stick_x;
        double turn = - gamepad.right_stick_x;
        this.omniMove(forward,strafe,turn);
    }

    // Reset sensors
    public void resetHeading() {
        wheelMath.imu.resetYaw();
    }
    public void resetOtos() {
        wheelMath.otos.resetTracking();
    }

    public void pointTowards(double target_yaw) {
        YawPitchRollAngles angles = wheelMath.imu.getRobotYawPitchRollAngles();
        double current_yaw = angles.getYaw();
        double power = wheelMath.angularDirection(current_yaw+wheelMath.getDeltaHeading()/20.0,target_yaw) / 20.0;
        this.omniMove(0.0,0.0,power);
    }

    /**
     *
     * @param Forward positive x
     * @param Right positive y
     * @param Rotate counterclockwise
     */
    public void globalOmniMove(double Forward, double Right, double Rotate) {
        double heading = wheelMath.getHeading();
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
        if (wheelMath.aprilTagDetector.last_check.milliseconds() > 100) wheelMath.aprilTagDetector.detect();
        if (wheelMath.aprilTagDetector.findByID(id) != null) {
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
    public boolean lookAtApriltag(int id, double offset) {
        for (AprilTagDetection detection : wheelMath.aprilTagDetector.current_detections) {
            if (detection.id == id) {
                double target = detection.ftcPose.bearing + offset;
                double turn =  (target - wheelMath.getDeltaHeading() / 20.0) / 23.0;
                omniMove(0,0,turn);
                return true;
            }
        }
        return false;
    }

    public void moveToPosition(double target_x, double target_y, double target_h) {
        SparkFunOTOS.Pose2D current_pos = wheelMath.getPosition();
        double x_diff = target_x - current_pos.x;
        double y_diff = target_y - current_pos.y;
        double h_diff = target_h - current_pos.h;

        double magnitude = hypot(x_diff,y_diff);
        double vect_x = 0.25 * 2 * x_diff / magnitude;
        double vect_y = 0.25 * 2 * y_diff / magnitude;

        if(magnitude < 3) {
            vect_x /= 3.0;
            vect_y /= 3.0;
        }

        globalOmniMove(vect_x,vect_y,h_diff/100.0);
    }

    public void moveToPosition(SparkFunOTOS.Pose2D pos) {
        moveToPosition(pos.x,pos.y,pos.h);
    }




}

























