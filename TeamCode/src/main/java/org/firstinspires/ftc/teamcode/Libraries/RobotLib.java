package org.firstinspires.ftc.teamcode.Libraries;

import static java.lang.Math.abs;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@SuppressWarnings("unused")
public class RobotLib {
    public static class Vector2 {
        double x;
        double y;
        public Vector2(double x, double y) {
            this.x = x;
            this.y = y;
        }
        public double magnitude() {
            return Math.sqrt(x*x+y*y);
        }
    }
    public static SparkFunOTOS.Pose2D Pose2DRotate(SparkFunOTOS.Pose2D pose, double angle) {
        // Using rotation matrix to rotate Pose2D to target angle

        double currentAngle = pose.h * 0.01745329251; // Convert degrees to radians for sin and cos functions

        // Apply rotation matrix
        double x = pose.x * Math.cos(angle) - pose.y * Math.sin(angle);
        double y = pose.x * Math.sin(angle) + pose.y * Math.cos(angle);

        SparkFunOTOS.Pose2D newPos = new SparkFunOTOS.Pose2D(x,y,currentAngle+angle);

        return newPos;
    }
    public static SparkFunOTOS.Pose2D Pose2DSetHeading(SparkFunOTOS.Pose2D pos, double targetAngle) {
        // Using rotation matrix to rotate Pose2D to target angle

        double currentAngle = pos.h * 0.01745329251; // Convert degrees to radians for sin and cos functions

        // Apply rotation matrix
        double x = pos.x * Math.cos(targetAngle - currentAngle) - pos.y * Math.sin(targetAngle - currentAngle);
        double y = pos.x * Math.sin(targetAngle - currentAngle) + pos.y * Math.cos(targetAngle - currentAngle);

        SparkFunOTOS.Pose2D newPos = new SparkFunOTOS.Pose2D(x,y,targetAngle);

        return newPos;
    }
    public static class Robot {
        private HardwareMap hardwareMap;

        public DcMotor Front_Right;
        public DcMotor Front_Left;
        public DcMotor Back_Right;
        public DcMotor Back_Left;

        public DcMotor Arm_Motor;

        public SparkFunOTOS otos;

        public VisionPortal visionPortal;
        public AprilTagProcessor aprilTagProcessor;
        public List<AprilTagDetection> currentDetections;
        public ElapsedTime timeSinceLastAprilTagCheck;

        public IMU imu;

        private Boolean OTOS_ENABLED = false;
        private Boolean ARM_ENABLED = false;
        private Boolean IMU_ENABLED = false;
        private Boolean APRILTAG_ENABLED = false;

        public Robot(HardwareMap hardwareMap) {
            this.hardwareMap = hardwareMap;
            this.Front_Right = hardwareMap.get(DcMotor.class, "frontright");
            this.Front_Left = hardwareMap.get(DcMotor.class, "frontleft");
            this.Back_Right = hardwareMap.get(DcMotor.class, "backright");
            this.Back_Left = hardwareMap.get(DcMotor.class, "backleft");
        }
        public Robot enableOtos() {
            this.otos = this.hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
            otos.calibrateImu();
            otos.resetTracking();
            this.OTOS_ENABLED = true;
            return this;
        }
        public Robot enableArm() {
            this.Arm_Motor = this.hardwareMap.get(DcMotor.class, "arm");
            this.Arm_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.Arm_Motor.setTargetPosition(0);
            this.Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.ARM_ENABLED = true;
            return this;
        }
        public Robot enableIMU() {
            this.imu = this.hardwareMap.get(IMU.class, "imu");
            this.IMU_ENABLED = true;
            return this;
        }
        public Robot enableAprilTagDetection() {
            this.aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
            this.visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTagProcessor);
            this.currentDetections = aprilTagProcessor.getDetections();
            this.timeSinceLastAprilTagCheck = new ElapsedTime();
            this.APRILTAG_ENABLED = true;
            return this;
        }
//        public Robot(HardwareMap hardwareMap, boolean findOtos, boolean enableArm) {
//            this.Front_Right = hardwareMap.get(DcMotor.class, "frontright");
//            this.Front_Left = hardwareMap.get(DcMotor.class, "frontleft");
//            this.Back_Right = hardwareMap.get(DcMotor.class, "backright");
//            this.Back_Left = hardwareMap.get(DcMotor.class, "backleft");
//
//
//            if(findOtos) {
//                this.otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
//                this.otos.setLinearUnit(DistanceUnit.METER);
//                this.otos.setAngularUnit(AngleUnit.DEGREES);
//                this.otos.calibrateImu();
//                this.otos.resetTracking();
//                this.OTOS_ENABLED = true;
//            }
//
//            if(enableArm) {
//                // Set up arm
//                this.Arm_Motor = hardwareMap.get(DcMotor.class, "arm");
//                this.Arm_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                this.Arm_Motor.setTargetPosition(0);
//                this.Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                this.ARM_ENABLED = true;
//            }
//
//
//        }
        public Robot(DcMotor Front_Right, DcMotor Front_Left, DcMotor Back_Right, DcMotor Back_Left) {
            this.Front_Right = Front_Right;
            this.Front_Left = Front_Left;
            this.Back_Right = Back_Right;
            this.Back_Left = Back_Left;
        }

        // Basic wheel functions
        public void Set_Wheels(double Front_Right_Power, double Front_Left_Power, double Back_Right_Power, double Back_Left_Power) {
            this.Front_Right.setPower(Front_Right_Power);
            this.Front_Left.setPower(Front_Left_Power);
            this.Back_Right.setPower(Back_Right_Power);
            this.Back_Left.setPower(Back_Left_Power);
        }

        public void Omni_Move(double Forward, double Right, double Rotate, double speed) {
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

            this.Set_Wheels(fr * speed, fl * speed, br * speed, bl * speed);
        }
        public void Omni_Move(Vector2 vec2, double speed) {
            Omni_Move(vec2.y,vec2.x,0.0,speed);
        } public void Omni_Move(Vector2 vec2) { Omni_Move(vec2, 1.0); }
        public void Omni_Move(double Forward, double Right, double RotateCC) {
            Omni_Move(Forward, Right, RotateCC, 1.0); /* If no speed provided, assume full speed (1.0) */
        }
        public void Omni_Move_Controller(Gamepad gamepad, double speed) {
            double forward = - gamepad.left_stick_y;
            double strafe = gamepad.left_stick_x;
            double turn = - gamepad.right_stick_x;
            Omni_Move(forward,strafe,turn,speed);
        }
        public void Omni_Move_Controller(Gamepad gamepad) {
            Omni_Move_Controller(gamepad, 1);
        }

        public void Omni_Move_Transformed(double localForward, double localRight, double RotateCC, double angle, double speed) {
            // Rotated version of Omni_Move
            double forward = localForward * Math.cos(angle) - localRight * Math.sin(angle);
            double right = localForward * Math.sin(angle) + localRight * Math.cos(angle);
            this.Omni_Move(forward,right,RotateCC,speed);
        }

        public void Reverse_These(boolean frontright, boolean frontleft, boolean backright, boolean backleft) {
            if(frontright) {
                this.Front_Right.setDirection(this.Front_Right.getDirection().inverted());
            }
            if(frontleft) {
                this.Front_Left.setDirection(this.Front_Left.getDirection().inverted());
            }
            if(backright) {
                this.Back_Right.setDirection(this.Back_Right.getDirection().inverted());
            }
            if(backleft) {
                this.Back_Left.setDirection(this.Back_Left.getDirection().inverted());
            }
        }
        public void Reverse_Left() {
            this.Reverse_These(false,true,false,true);
        }
        public void Reverse_Right() {
            this.Reverse_These(true,false,true,false);
        }

        public void Stop_Wheels() {
            this.Set_Wheels(0, 0, 0, 0);
        }

        // Otos functions
        public SparkFunOTOS.Pose2D Get_Position() {
            SparkFunOTOS.Pose2D pos = otos.getPosition();
            return pos;
        }
        public void Reset_Otos() {
            otos.resetTracking();
        }
        public void Omni_Move_To_Target(SparkFunOTOS.Pose2D target,double speed) {
            if(!OTOS_ENABLED) return;
            SparkFunOTOS.Pose2D pos = Get_Position();
            double dx = 3 * (target.x - pos.x) / 5.0f;
            double dy = -3 * (target.y - pos.y) / 5.0f;
            double dh = (target.h - pos.h) / 18.0;
            this.Omni_Move_Transformed(dx, dy, dh, pos.h * 0.01745329251, speed);
        }
        public void Return_Home() {
            Omni_Move_To_Target(new SparkFunOTOS.Pose2D(0,0,0), 0.8);
        }
        public double Distance_To(SparkFunOTOS.Pose2D target) {
            SparkFunOTOS.Pose2D pos = Get_Position();
            double dx = (target.x - pos.x);
            double dy = (target.y - pos.y);
            double dh = (target.h - pos.h) / 180.0;
            return Math.sqrt(dx*dx+dy*dy+dh*dh);
        }

        // Arm functions
        public void Set_Arm_Power(double power) {
            this.Arm_Motor.setPower(power);
        }
        public void Set_Arm_Position(int tick) {
            this.Arm_Motor.setTargetPosition(tick);
        }
        public int Get_Arm_Position() {
            return this.Arm_Motor.getCurrentPosition();
        }
        public void Reset_Arm_Reading() {
            Arm_Motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            Arm_Motor.setTargetPosition(0);
            Arm_Motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        }
        public void Arm_Glide_To(int tick) {
        }

        // Imu functions
        public void Reset_IMU() {
            this.imu.resetYaw();
        }
        public void PRM_Move(double localForward, double localRight, double RotateCC, double speed) {
            // Angle processing
            YawPitchRollAngles angles = this.imu.getRobotYawPitchRollAngles();
            double yaw = angles.getYaw() * 0.01745329251; // Get robot yaw converted to radians

            this.Omni_Move_Transformed(localForward,localRight,RotateCC,yaw,speed);
        }
        public void PRM_Move_Controller(Gamepad gamepad, double speed) {
            double forward = - gamepad.left_stick_y;
            double strafe = gamepad.left_stick_x;
            double turn = - gamepad.right_stick_x;
            PRM_Move(forward,strafe,turn,speed);
        }

        // Apriltag functions
        public List<AprilTagDetection> getAprilTagDetections() {
            return aprilTagProcessor.getDetections();
        }
        public void UpdateAprilTagDetections() {
            this.currentDetections = getAprilTagDetections(); // Update tags in memory
            this.timeSinceLastAprilTagCheck.reset(); // Reset timer
        }
        public void TelemetryAprilTags(Telemetry telemetry) {
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ (Raw) %6.1f %6.1f %6.1f", detection.rawPose.x, detection.rawPose.y, detection.rawPose.z));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));

                    telemetry.addLine("\n---- Magic");
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                }
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }
        public boolean LookAtAprilTag(int id, double offset) {
            for (AprilTagDetection detection : currentDetections) {
                if (detection.id == id) {
                    double target = detection.ftcPose.bearing + offset;
                    double turn =  (target - imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate / 20.0) / 18.0;;
                    Omni_Move(0,0,turn);
                    return true;
                }
            }
            return false;
        }
        public SparkFunOTOS.Pose2D GetPositionBasedOnAprilTag() {
            for (AprilTagDetection detection : currentDetections) {
                if (detection.id == 20) {
                    SparkFunOTOS.Pose2D pose = new SparkFunOTOS.Pose2D(detection.ftcPose.x,detection.ftcPose.y,detection.ftcPose.yaw);
                    return Pose2DRotate(pose,-detection.ftcPose.yaw+233.3);
                }
            }
            return new SparkFunOTOS.Pose2D();
        }
    }
}
