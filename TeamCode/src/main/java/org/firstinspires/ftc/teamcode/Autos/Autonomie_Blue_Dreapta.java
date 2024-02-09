package org.firstinspires.ftc.teamcode.Autos;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants.ArmRotatorPosition;
import org.firstinspires.ftc.teamcode.Constants.AutonomousPositions;
import org.firstinspires.ftc.teamcode.Constants.ClawPosition;
import org.firstinspires.ftc.teamcode.Constants.ClawRotatorPosition;
import org.firstinspires.ftc.teamcode.Detection.BluePosDetect;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Arrays;


@Autonomous(name="Autonomie_Blue_Dreapta", group="Auto")
public class Autonomie_Blue_Dreapta extends LinearOpMode {
    private MecanumDrive drive;
    FtcDashboard dashboard;

    OpenCvCamera webcam;
    // private SampleMecanumDrive drive = null;
    BluePosDetect pipeline;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        dashboard = FtcDashboard.getInstance();


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new BluePosDetect(telemetry);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                                         @Override
                                         public void onOpened() {
                                             webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
                                             dashboard.startCameraStream(webcam, 120);
                                         }

                                         @Override
                                         public void onError(int errorCode) {

                                         }
                                     }

        );
        drive= new MecanumDrive(hardwareMap, new Pose2d(
                AutonomousPositions.xBlueRightStart,
                AutonomousPositions.yBlueRightStart,
                AutonomousPositions.angleBlueRightStart));


        drive.initAutonomous();

        waitForStart();

        drive.armRotatorLeft.setPosition(ClawRotatorPosition.PLACE);
        wait(1000);

        switch (pipeline.getLocation()) {
            case LEFT:
                CazStanga();
                break;
            case MID:
                CazMijloc();
                break;
            case RIGHT:
                CazDreapta();
                break;

        }
        webcam.stopStreaming();
    }



    public void CazStanga() {
        drive.armRotatorLeft.setPosition(ArmRotatorPosition.Stack);
        drive.armRotatorRight.setPosition(ArmRotatorPosition.Stack);
        drive.clawRotatorLeft.setPosition(ClawRotatorPosition.Take_Autonomous);

        Action Spike=drive.actionBuilder(drive.pose)
                .lineToYLinearHeading(15, Math.toRadians(33))
                .afterTime(0,()->{
                    drive.clawRight.setPosition(0.3);
                })
                .waitSeconds(0.3)
                .build();

        Action Panou=drive.actionBuilder(new Pose2d(-34.87316616600596,17.457028467325244  ,Math.toRadians(33)))

                .strafeToLinearHeading(new Vector2d(-47.21,10.5),Math.toRadians(-180))
                .afterTime(0,()->{
                    drive.clawRight.setPosition(ClawPosition.OPEN);
                })
                .lineToX(-56)//-54.21

                .afterTime(1,()->{
                    drive.clawRight.setPosition(ClawPosition.CLOSE);
                })
                .waitSeconds(1)
                .lineToX(25)
                .afterTime(0,()->{
                    //                    drive.clawRotatorLeft.setPosition(ClawRotatorPosition.PLACE);
                    new Thread(new Runnable() {
                        @Override
                        public void run() {
                            for (double i = drive.armRotatorLeft.getPosition(); i <= ArmRotatorPosition.UP; i += 0.02) {
                                if (i >= 0.7) {
                                    if(i >= 0.8)
                                        drive.clawRotatorLeft.setPosition(ClawRotatorPosition.RIDICARE);
                                    else
                                        drive.clawRotatorLeft.setPosition(ClawRotatorPosition.PLACE);
                                }
                                drive.armRotatorRight.setPosition(i);
                                drive.armRotatorLeft.setPosition(i);
                                //if(i<=0.3||i>=0.65)
                                sleep(25);
                            }
                            drive.armRotatorLeft.setPosition(ArmRotatorPosition.UP);
                            drive.armRotatorRight.setPosition(ArmRotatorPosition.UP);
                        }
                    }).start();
                    drive.runLiftToPosition(-0.4 ,-820);
                })
                .strafeToLinearHeading(new Vector2d(55, 37.8), Math.toRadians(-194))
                .afterTime(3.,()->{
                    // drive.clawRight.setPosition(ClawPosition.OPEN);
                    drive.clawLeft.setPosition(0.3);
                    drive.clawRight.setPosition(0.4);
                })
//                .afterTime(4,()->{
//                    // drive.clawRight.setPosition(ClawPosition.OPEN);
//                    drive.clawRight.setPosition(0.3);
//                })
                .afterTime(5,()->{
                    drive.clawRight.setPosition(ClawPosition.CLOSE);
                    drive.clawLeft.setPosition(ClawPosition.CLOSE);
                    //drive.runLiftToPosition(0.2 ,0);
                    new Thread(new Runnable() {
                        @Override
                        public void run() {
                            drive.clawRotatorLeft.setPosition(ClawRotatorPosition.PLACE);
                            for (double i = drive.armRotatorLeft.getPosition(); i >= ArmRotatorPosition.DOWN_AUTONOMOUS; i -= 0.02) {
                                if (i <= 0.7) {
                                    drive.clawRotatorLeft.setPosition(ClawRotatorPosition.TAKE);
                                }
                                drive.armRotatorRight.setPosition(i);
                                drive.armRotatorLeft.setPosition(i);
                                //if(i<=0.3||i>=0.65)
                                sleep(25);
                            }
                            drive.armRotatorLeft.setPosition(ArmRotatorPosition.DOWN_AUTONOMOUS);
                            drive.armRotatorRight.setPosition(ArmRotatorPosition.DOWN_AUTONOMOUS);
                        }
                    }).start();
                    drive.runLiftToPosition(0.2 ,0);
                })
                .waitSeconds(7)

                .build();
        Actions.runBlocking(new SequentialAction(
                Spike,
                Panou
        ));

    }

    public void CazMijloc() {
        Action Spike = drive.actionBuilder(drive.pose)
                .lineToYLinearHeading(11, Math.PI / 2)
                .afterTime(0, () -> {
                    drive.clawRotatorLeft.setPosition(ClawRotatorPosition.Take_Autonomous);
                    drive.armRotatorLeft.setPosition(ArmRotatorPosition.Stack+0.002);
                    drive.armRotatorRight.setPosition(ArmRotatorPosition.Stack+0.002);
                })
                .afterTime(1,()->{
                    drive.clawRight.setPosition(ClawPosition.OPEN);
                })
                .build();

        Vector2d stackVector = new Vector2d(-44.6,9.8);
        double stackAngle = -182.0;
        Vector2d panouVector = new Vector2d(56.3,36.9); // Modifici aici pe panou
        double panouAngle = -186.0;

        Action Stack = drive.actionBuilder(new Pose2d(AutonomousPositions.xBlueRightStart,11,Math.PI/2))
                .strafeToLinearHeading(stackVector,Math.toRadians(stackAngle))
                .lineToX(-55)//-54.21
                .afterTime(0.8,()->{
                    drive.clawRight.setPosition(ClawPosition.CLOSE);
                })
                .waitSeconds(0.2)
                .build();


        Pose2d nextPose = new Pose2d(stackVector,Math.toRadians(stackAngle));

        Action Panou = drive.actionBuilder(nextPose)
                .lineToX(25) // merge in partea cealalta a terenului
                .afterTime(0,()->{
                    new Thread(new Runnable() {
                        @Override
                        public void run() {
                            for (double i = drive.armRotatorLeft.getPosition(); i <= ArmRotatorPosition.UP; i += 0.02) {
                                if (i >= 0.7) {
                                    if(i >= 0.8)
                                        drive.clawRotatorLeft.setPosition(ClawRotatorPosition.RIDICARE);
                                    else
                                        drive.clawRotatorLeft.setPosition(ClawRotatorPosition.PLACE);
                                }
                                drive.armRotatorRight.setPosition(i);
                                drive.armRotatorLeft.setPosition(i);
                                //if(i<=0.3||i>=0.65)
                                sleep(25);
                            }
                            drive.armRotatorLeft.setPosition(ArmRotatorPosition.UP);
                            drive.armRotatorRight.setPosition(ArmRotatorPosition.UP);
                        }
                    }).start();
                    drive.runLiftToPosition(-0.4 ,-1200);
                })
                .strafeToLinearHeading(panouVector,Math.toRadians(panouAngle))
                .afterTime(3.5,()->{
                    drive.clawRight.setPosition(ClawPosition.OPEN);
                    drive.clawLeft.setPosition(ClawPosition.OPEN);
                })
                .build();

        nextPose = new Pose2d(panouVector,Math.toRadians(panouAngle));

        Action Park = drive.actionBuilder(nextPose)
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(36,36.9))
                .afterTime(0.2, () -> {
                    drive.runLiftToPosition(0.4 ,0);
                    new Thread(new Runnable() {
                        @Override
                        public void run() {
                            drive.runLiftToPosition(0.4, 0);
                            drive.clawRotatorLeft.setPosition(ClawRotatorPosition.PLACE);
                            for (double i = drive.armRotatorLeft.getPosition(); i >= ArmRotatorPosition.DOWN; i -= 0.02) {
                                if (i <= 0.7) {
                                    drive.clawRotatorLeft.setPosition(ClawRotatorPosition.TAKE);
                                }
                                drive.armRotatorRight.setPosition(i);
                                drive.armRotatorLeft.setPosition(i);
                                //if(i<=0.3||i>=0.65)
                                sleep(25);
                            }
                            drive.armRotatorLeft.setPosition(ArmRotatorPosition.Stack);
                            drive.armRotatorRight.setPosition(ArmRotatorPosition.Stack);
                        }
                    }).start();
                })
                .strafeToLinearHeading(new Vector2d(52,2),Math.toRadians(-82))
                        .build();

        Actions.runBlocking(new SequentialAction(
                Spike,
                Stack,
                Panou,
                Park
        ));
    }

    public void CazDreapta() {

        Action Spike = drive.actionBuilder(drive.pose)
                .lineToYLinearHeading(15, Math.PI/6)
                .afterTime(0, () -> {
                    drive.clawRotatorLeft.setPosition(ClawRotatorPosition.Take_Autonomous);
                    drive.armRotatorLeft.setPosition(ArmRotatorPosition.Stack);
                    drive.armRotatorRight.setPosition(ArmRotatorPosition.Stack);
                })
                .afterTime(1,()->{
                    drive.clawRight.setPosition(0.35);
                })
                .waitSeconds(1.2)
                .strafeToLinearHeading(new Vector2d(-47.21,11),Math.toRadians(-180))
                .afterTime(0,()->{
                    drive.clawRight.setPosition(ClawPosition.OPEN);
                })
                .lineToX(-55)//-54.21

                .afterTime(1,()->{
                    drive.clawRight.setPosition(ClawPosition.CLOSE);
                })
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(30,11),Math.toRadians(-180))
                // .turn(Math.toRadians(35))
                .afterTime(0,()->{
                    //                    drive.clawRotatorLeft.setPosition(ClawRotatorPosition.PLACE);
                    new Thread(new Runnable() {
                        @Override
                        public void run() {
                            for (double i = drive.armRotatorLeft.getPosition(); i <= ArmRotatorPosition.UP; i += 0.02) {
                                if (i >= 0.7) {
                                    if(i >= 0.8)
                                        drive.clawRotatorLeft.setPosition(ClawRotatorPosition.RIDICARE);
                                    else
                                        drive.clawRotatorLeft.setPosition(ClawRotatorPosition.PLACE);
                                }
                                drive.armRotatorRight.setPosition(i);
                                drive.armRotatorLeft.setPosition(i);
                                //if(i<=0.3||i>=0.65)
                                sleep(25);
                            }
                            drive.armRotatorLeft.setPosition(ArmRotatorPosition.UP);
                            drive.armRotatorRight.setPosition(ArmRotatorPosition.UP);
                        }
                    }).start();
                    drive.runLiftToPosition(-0.4 ,-1100);
                })
                .splineToLinearHeading(new Pose2d(58, 33.5, Math.toRadians(-196)), Math.toRadians(-10))
                .afterTime(3.,()->{
                    // drive.clawRight.setPosition(ClawPosition.OPEN);
                    drive.clawLeft.setPosition(0.3);
                    drive.clawRight.setPosition(0.3);
                })
//                .afterTime(4,()->{
//                    // drive.clawRight.setPosition(ClawPosition.OPEN);
//                    drive.clawRight.setPosition(0.3);
//                })
                .afterTime(5,()->{
                    drive.clawRight.setPosition(ClawPosition.CLOSE);
                    drive.clawLeft.setPosition(ClawPosition.CLOSE);
                    //drive.runLiftToPosition(0.2 ,0);
                    new Thread(new Runnable() {
                        @Override
                        public void run() {
                            drive.clawRotatorLeft.setPosition(ClawRotatorPosition.PLACE);
                            for (double i = drive.armRotatorLeft.getPosition(); i >= ArmRotatorPosition.DOWN_AUTONOMOUS; i -= 0.02) {
                                if (i <= 0.7) {
                                    drive.clawRotatorLeft.setPosition(ClawRotatorPosition.TAKE);
                                }
                                drive.armRotatorRight.setPosition(i);
                                drive.armRotatorLeft.setPosition(i);
                                //if(i<=0.3||i>=0.65)
                                sleep(25);
                            }
                            drive.armRotatorLeft.setPosition(ArmRotatorPosition.DOWN_AUTONOMOUS);
                            drive.armRotatorRight.setPosition(ArmRotatorPosition.DOWN_AUTONOMOUS);
                        }
                    }).start();
                    drive.runLiftToPosition(0.2 ,0);
                })
                .waitSeconds(7)

                .build();
        Actions.runBlocking(new SequentialAction(
                Spike
                // Panou
        ));
    }



    private void stopDriving(){
        drive.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // drive.intake.setPower(0);
        // drive.getLastTick(robot.ridicareBrat.getCurrentPosition());
    }




}