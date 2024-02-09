package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.ArmRotatorPosition;
import org.firstinspires.ftc.teamcode.Constants.ClawPosition;
import org.firstinspires.ftc.teamcode.Constants.ClawRotatorPosition;
import org.firstinspires.ftc.teamcode.Constants.ParkServoPosition;
import org.firstinspires.ftc.teamcode.Constants.Powers;
import org.firstinspires.ftc.teamcode.Constants.ShooterPosition;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(name = "teleop")
public class TeleOP extends LinearOpMode {

    public MecanumDrive robot;
    //public ComponentsMap drive;

    @Override
    public void runOpMode() throws InterruptedException{
        robot = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.initTeleOP();
        waitForStart();

        //ACTIONARE CLESTE DREAPTA
        new Thread(new Runnable() {
            @Override
            public void run() {
                while (opModeIsActive()) {
                    if (gamepad1.right_trigger > 0)
                        robot.clawRight.setPosition(ClawPosition.OPEN);

                    if (gamepad1.right_bumper)
                        robot.clawRight.setPosition(ClawPosition.CLOSE);
                }
            }
        }).start();

        //ACTIONARE CLESTE STANGA
        new Thread(new Runnable() {
            @Override
            public void run() {
                while(opModeIsActive()) {
                    if (gamepad1.left_trigger > 0)
                        robot.clawLeft.setPosition(ClawPosition.OPEN);

                    if (gamepad1.left_bumper)
                        robot.clawLeft.setPosition(ClawPosition.CLOSE);
                }
            }
        }).start();

        //ACTIONARE BRAT
        new Thread(new Runnable() {
            @Override
            public void run() {
               // boolean apasatDpadUp = false;
                //boolean apasatDpadDown = false;
                while(opModeIsActive()){
                    if (gamepad2.dpad_up ) {
                       // apasatDpadUp = true;
                        robot.clawRotatorLeft.setPosition(ClawRotatorPosition.TAKE);
                        for (double i = robot.armRotatorLeft.getPosition(); i <= ArmRotatorPosition.UP; i += 0.02) {
                            robot.armRotatorRight.setPosition(i);
                            robot.armRotatorLeft.setPosition(i);
                            sleep(25);
                            if (i >= 0.4 &&
                                    !robot.almostEquals(robot.clawRotatorLeft.getPosition(), ClawRotatorPosition.RIDICARE, 0.02)) {
                                robot.clawRotatorLeft.setPosition(ClawRotatorPosition.PLACE);
                            }
                        }
                        robot.armRotatorLeft.setPosition(ArmRotatorPosition.UP);
                        robot.armRotatorRight.setPosition(ArmRotatorPosition.UP);
                    }
                    //else apasatDpadUp = false;

                    if (gamepad2.dpad_down) {
                       // apasatDpadDown = true;
                        robot.clawRotatorLeft.setPosition(ClawRotatorPosition.PLACE);
                        for (double i = robot.armRotatorLeft.getPosition(); i >= ArmRotatorPosition.DOWN; i -= 0.02) {
                            if (i <= 0.7) {
                                robot.clawRotatorLeft.setPosition(ClawRotatorPosition.TAKE);
                            }
                            robot.armRotatorRight.setPosition(i);
                            robot.armRotatorLeft.setPosition(i);
                            //if(i<=0.3||i>=0.65)
                            sleep(25);
                        }
                        robot.armRotatorLeft.setPosition(ArmRotatorPosition.DOWN);
                        robot.armRotatorRight.setPosition(ArmRotatorPosition.DOWN);
                    }
                    //else apasatDpadDown = false;
                }
            }
        }).start();

        //ACTIONARE CLAW ROTATOR
        new Thread(new Runnable() {
            @Override
            public void run() {
                while(opModeIsActive()) {
                    if (gamepad2.cross)
                        robot.clawRotatorLeft.setPosition(ClawRotatorPosition.RIDICARE);

                    if (gamepad2.triangle) {
                        robot.clawRotatorLeft.setPosition(ClawRotatorPosition.TAKE);
                    }
                }
            }
        }).start();

        //ACTIONARE MOTOARE SUSPENDARE ROBOT
        new Thread(new Runnable() {
            @Override
            public void run() {
                boolean isWorking = false;
                boolean apasat = false;
                while(opModeIsActive()){
                    if(gamepad1.dpad_up){
                        robot.parkLiftRight.setPower(1);
                        robot.parkLiftLeft.setPower(1);
                        robot.parkingServoLeft.setPosition(ParkServoPosition.Down_Left);
                        robot.parkingServoRight.setPosition(ParkServoPosition.Down_Right);
                    }

                    if(gamepad1.dpad_down){
                        robot.parkLiftRight.setPower(-1);
                        robot.parkLiftLeft.setPower(-1);
                    }

                    if(!gamepad1.dpad_down && !gamepad1.dpad_up){
                        robot.parkLiftRight.setPower(0);
                        robot.parkLiftLeft.setPower(0);
                        robot.parkLiftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        robot.parkLiftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    }
                }
            }
        }).start();

        //ACTIONARE BRATE SUSPENDARE ROBOT
        new Thread(new Runnable() {
            @Override
            public void run() {
                while(opModeIsActive()){
                    if(gamepad1.right_stick_button)
                        robot.parkingServoLeft.setPosition(ParkServoPosition.Shooter);

                    if (gamepad1.dpad_right) {
                        robot.parkingServoRight.setPosition(ParkServoPosition.Up_Right);
                        robot.parkingServoLeft.setPosition(ParkServoPosition.Up_Left);
                    }

                    if(gamepad1.dpad_left)
                    {
                        robot.parkingServoRight.setPosition(ParkServoPosition.Down_Right);
                        robot.parkingServoLeft.setPosition(ParkServoPosition.Down_Left);
                    }
                }
            }
        }).start();

        //ACTIONARE SHOOTER
        new Thread(new Runnable() {
            @Override
            public void run() {
                while(opModeIsActive()){
                    if(gamepad2.right_stick_button)
                        robot.Shooter.setPosition(ShooterPosition.SHOOT);
                }
            }
        }).start();

        //ACTIONARE LIFT
        new Thread(new Runnable() {
            @Override
            public void run() {
                while(opModeIsActive()){
                    if((robot.liftLeft.getCurrentPosition() <= -1800 && robot.liftRight.getCurrentPosition() <= -1800) ||
                            robot.liftRight.getCurrentPosition() >= -400 && robot.liftLeft.getCurrentPosition() >= -400)
                        Powers.liftSpeedReductor = 0.4;
                    else Powers.liftSpeedReductor = 0.7;

                    if (gamepad2.left_trigger > 0 && robot.liftRight.getCurrentPosition() <= 40
                        /*&& robot.liftLeft.getCurrentPosition() <=-20*/) {
                        robot.liftLeft.setPower(gamepad2.left_trigger * Powers.liftSpeedReductor);
                        robot.liftRight.setPower(gamepad2.left_trigger * Powers.liftSpeedReductor);
                    } else if (gamepad2.right_trigger > 0
                            && robot.liftRight.getCurrentPosition() >= -2100
                        /*&& robot.liftLeft.getCurrentPosition() >= -2100*/) {
                        robot.liftLeft.setPower(-gamepad2.right_trigger * Powers.liftSpeedReductor);
                        robot.liftRight.setPower(-gamepad2.right_trigger * Powers.liftSpeedReductor);
                    }
                    else if(gamepad2.left_stick_y < 0){
                        robot.liftLeft.setPower(gamepad2.left_stick_y);
                        robot.liftRight.setPower(gamepad2.left_stick_y);
                    }
                    else if(gamepad2.left_stick_y > 0){
                        robot.liftLeft.setPower(gamepad2.left_stick_y);
                        robot.liftRight.setPower(gamepad2.left_stick_y);
                    }
                    else {
                        robot.liftLeft.setPower(0);
                        robot.liftRight.setPower(0);
                        robot.liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        robot.liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    }
                }
            }
        }).start();

        while(opModeIsActive()){
            ///MISCARE DRIVE RELATIVE
            Pose2d poseEstimate = robot.pose;

            Vector2d input = robot.rotated(poseEstimate.heading.toDouble(), -gamepad1.left_stick_x, -gamepad1.left_stick_y );

                robot.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(input.y * Powers.drivePower * Powers.driveSpeedReductor,
                                input.x * Powers.drivePower * Powers.driveSpeedReductor),
                        -gamepad1.right_stick_x * Powers.drivePower * Powers.driveSpeedReductor
                ));

            robot.updatePoseEstimate();

            telemetry.addData("Claw Rotator Position ", robot.clawRotatorLeft.getPosition());
//            telemetry.addData("heading", poseEstimate.heading);
//            telemetry.addData("leftFront : ",robot.leftFront.getCurrentPosition());
//            telemetry.addData("leftBack : ",robot.leftBack.getCurrentPosition());
//            telemetry.addData("rightFront : ",robot.rightFront.getCurrentPosition());
//            telemetry.addData("rightBack : ",robot.rightBack.getCurrentPosition());
            telemetry.addData("lift ", robot.liftLeft.getCurrentPosition());
            telemetry.update();
        }

    }

}
