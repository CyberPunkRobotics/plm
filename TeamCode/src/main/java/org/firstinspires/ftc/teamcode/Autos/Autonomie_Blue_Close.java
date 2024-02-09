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


@Autonomous(name="Autonomie_Blue_Close", group="Auto")
public class Autonomie_Blue_Close extends LinearOpMode {
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
                AutonomousPositions.xBlueLeftStart,
                AutonomousPositions.yBlueLeftStart,
                AutonomousPositions.angleBlueLeftStart));


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

        drive.clawRotatorLeft.setPosition(ClawRotatorPosition.TAKE);
        drive.armRotatorRight.setPosition(ArmRotatorPosition.DOWN_AUTONOMOUS);
        drive.armRotatorLeft.setPosition(ArmRotatorPosition.DOWN_AUTONOMOUS);

        Action Spike = drive.actionBuilder(drive.pose)
                .lineToYLinearHeading(36.5, Math.toRadians(-50))
                .afterTime(0,()->{
                    drive.clawRight.setPosition(ClawPosition.OPEN);
                })
                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(20.5,46), Math.toRadians(-120))
                .build();

        Vector2d panouVector = new Vector2d(56.7, 37); // Modifici aici pe panou
        double panouAngle = Math.toRadians(-180);

        Action Panou = drive.actionBuilder(new Pose2d(AutonomousPositions.xBlueLeftStart, 46, Math.toRadians(-120)))
                .afterTime(0,()->{
                    drive.clawRotatorLeft.setPosition(ClawRotatorPosition.PLACE);
                    drive.clawRight.setPosition(ClawPosition.CLOSE);
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
                .strafeToLinearHeading(panouVector, panouAngle,
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(30),
                                new AngularVelConstraint(Math.PI )
                        )))
                .afterTime(0.3,()->{
                    drive.clawLeft.setPosition(0.3);
                })
                .build();

        Pose2d nextPose = new Pose2d(panouVector,panouAngle);
        Action Park = drive.actionBuilder(nextPose)
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(42,28.5))
                .afterTime(0.2, () -> {
                    drive.runLiftToPosition(0.4 ,0);
                    new Thread(new Runnable() {
                        @Override
                        public void run() {
                            drive.runLiftToPosition(0.2, 0);
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
                .strafeToLinearHeading(new Vector2d(54,60),Math.toRadians(-81))
                .build();

        Actions.runBlocking(new SequentialAction(
                Spike,
                Panou,
                Park
        ));

    }

    public void CazMijloc() {

        drive.clawRotatorLeft.setPosition(ClawRotatorPosition.TAKE);
        drive.armRotatorRight.setPosition(ArmRotatorPosition.DOWN_AUTONOMOUS);
        drive.armRotatorLeft.setPosition(ArmRotatorPosition.DOWN_AUTONOMOUS);

        Action Spike = drive.actionBuilder(drive.pose)
                .lineToYLinearHeading(36.7, -Math.PI/2)
                .afterTime(0,()->{
                    drive.clawRight.setPosition(ClawPosition.OPEN);
                })
                .waitSeconds(0.2) // nu modifica asta ca crapa
                .lineToYConstantHeading(40)
                .build();

        Vector2d panouVector = new Vector2d(57,36.0); // Modifici aici pe panou
        double panouAngle = Math.toRadians(-174);

        Action Panou = drive.actionBuilder(new Pose2d(AutonomousPositions.xBlueLeftStart, 40, -Math.PI/2))
                .afterTime(0,()->{
                    drive.clawRotatorLeft.setPosition(ClawRotatorPosition.PLACE);
                    drive.clawRight.setPosition(ClawPosition.CLOSE);
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
                .strafeToLinearHeading(panouVector, panouAngle,
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(30),
                                new AngularVelConstraint(Math.PI )
                        )))
                .afterTime(0.3,()->{
                    drive.clawLeft.setPosition(0.3);
                })
                .build();

        Pose2d nextPose = new Pose2d(panouVector,panouAngle);
        Action Park = drive.actionBuilder(nextPose)
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(42,28.5))
                .afterTime(0.2, () -> {
                    drive.runLiftToPosition(0.4 ,0);
                    new Thread(new Runnable() {
                        @Override
                        public void run() {
                            drive.runLiftToPosition(0.2, 0);
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
                .strafeToLinearHeading(new Vector2d(54,60),Math.toRadians(-81))
                .build();

        Actions.runBlocking(new SequentialAction(
                Spike,
                Panou,
                Park
        ));
    }

    public void CazDreapta() {

        drive.clawRotatorLeft.setPosition(ClawRotatorPosition.TAKE);
        drive.armRotatorRight.setPosition(ArmRotatorPosition.DOWN_AUTONOMOUS+0.04);
        drive.armRotatorLeft.setPosition(ArmRotatorPosition.DOWN_AUTONOMOUS+0.04);

        Action Spike = drive.actionBuilder(drive.pose)
                .lineToYLinearHeading(36.5, Math.toRadians(-140))
                .afterTime(0,()->{
                    drive.clawRight.setPosition(ClawPosition.OPEN);
                })
                .waitSeconds(0.2)
                .build();

        Vector2d panouVector = new Vector2d(56,29); // Modifici aici pe panou
        double panouAngle = Math.toRadians(-176);

        Action Panou = drive.actionBuilder(new Pose2d(AutonomousPositions.xBlueLeftStart, 36.5, Math.toRadians(-140)))
                .afterTime(0,()->{
                    drive.clawRotatorLeft.setPosition(ClawRotatorPosition.PLACE);
                    drive.clawRight.setPosition(ClawPosition.CLOSE);
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
                .strafeToLinearHeading(panouVector, panouAngle,
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(30),
                                new AngularVelConstraint(Math.PI )
                        )))
                .afterTime(0.3,()->{
                    drive.clawLeft.setPosition(0.3);
                })
                .build();

        Pose2d nextPose = new Pose2d(panouVector,panouAngle);
        Action Park = drive.actionBuilder(nextPose)
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(42,28.5))
                .afterTime(0.2, () -> {
                    drive.runLiftToPosition(0.4 ,0);
                    new Thread(new Runnable() {
                        @Override
                        public void run() {
                            drive.runLiftToPosition(0.2, 0);
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
                .strafeToLinearHeading(new Vector2d(54,60),Math.toRadians(-81))
                .build();

        Actions.runBlocking(new SequentialAction(
                Spike,
                Panou,
                Park
        ));
       /* Action goToRight, Stack, goBack;

        for(int i=0;i<=0;++i) {

            goToRight = drive.actionBuilder(new Pose2d(54.7, 28, Math.toRadians(-176)))
                    .afterTime(0, ()-> {
                        drive.runLiftToPosition(0.2,0);
                    })
                    .strafeToLinearHeading(new Vector2d(20,68),Math.toRadians(-187))
                    .strafeToConstantHeading(new Vector2d(-32,68),
                            new MinVelConstraint(Arrays.asList(
                                    drive.kinematics.new WheelVelConstraint(30),
                                    new AngularVelConstraint(Math.PI )
                            )))
                    .strafeToConstantHeading(new Vector2d(-47,68))
                    .afterTime(2.5, () -> {
                        new Thread(new Runnable() {
                            @Override
                            public void run() {
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
                    .strafeToLinearHeading(new Vector2d(-43,50), Math.toRadians(-168),
                            new MinVelConstraint(Arrays.asList(
                                    drive.kinematics.new WheelVelConstraint(15),
                                    new AngularVelConstraint(Math.PI )
                            )))

                    .build();

            Stack = drive.actionBuilder(new Pose2d(-43, 50, Math.toRadians(-168)))
                    .afterTime(0, () -> {
                        new Thread(new Runnable() {
                            @Override
                            public void run() {
                                drive.clawRotatorLeft.setPosition(ClawRotatorPosition.PLACE);
                                for (double i = drive.armRotatorLeft.getPosition(); i >= ArmRotatorPosition.DOWN; i -= 0.04) {
                                    if (i <= 0.7) {
                                        drive.clawRotatorLeft.setPosition(ClawRotatorPosition.TAKE);
                                    }
                                    drive.armRotatorRight.setPosition(i);
                                    drive.armRotatorLeft.setPosition(i);
                                    //if(i<=0.3||i>=0.65)
                                    sleep(25);
                                }
                                drive.armRotatorLeft.setPosition(ArmRotatorPosition.Stack-0.10);
                                drive.armRotatorRight.setPosition(ArmRotatorPosition.Stack-0.10);
                            }
                        }).start();
                    })
                    .afterTime(0.1, () -> {
                        drive.clawRight.setPosition(ClawPosition.OPEN);
                    })
                    .waitSeconds(0.2)
                    .strafeToConstantHeading(new Vector2d(-46.3,50),
                            new MinVelConstraint(Arrays.asList(
                                    drive.kinematics.new WheelVelConstraint(10),
                                    new AngularVelConstraint(Math.PI )
                            )))
                    .afterTime(0.2, () -> {
                        drive.clawRight.setPosition(ClawPosition.CLOSE);
                    })
                    .afterTime(1, () -> {
                        drive.clawRotatorLeft.setPosition(ClawRotatorPosition.RIDICARE);
                    })
                    .build();

            goBack = drive.actionBuilder(new Pose2d(-46.3, 50, Math.toRadians(-168)))
                    .strafeToLinearHeading(new Vector2d(-47,70), Math.toRadians(-185),
                            new MinVelConstraint(Arrays.asList(
                                    drive.kinematics.new WheelVelConstraint(15),
                                    new AngularVelConstraint(Math.PI )
                            )))
                    .build();

            Actions.runBlocking(new SequentialAction(
                    goToRight,
                    Stack,
                    goBack
            ));

        }

        */ // E scris ieri


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