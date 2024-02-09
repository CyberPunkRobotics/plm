package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants.ArmRotatorPosition;
import org.firstinspires.ftc.teamcode.Constants.AutonomousPositions;
import org.firstinspires.ftc.teamcode.Constants.ClawPosition;
import org.firstinspires.ftc.teamcode.Constants.ClawRotatorPosition;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Disabled
@Config
@Autonomous(name="AutonomieAlbastruDreapta", group = "Autonomous")
public class AutonomieAlbastruDreapta extends LinearOpMode {
    private MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        //Pozitie start
        drive = new MecanumDrive(hardwareMap, new Pose2d(
                AutonomousPositions.xBlueRightStart,
                AutonomousPositions.yBlueRightStart,
                AutonomousPositions.angleBlueRightStart));

        drive.initAutonomous();

        waitForStart();

        Caz2();
    }

    public void Caz1() {
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

    public void Caz2() {
        Action Spike = drive.actionBuilder(drive.pose)
                .lineToYLinearHeading(11, Math.PI / 2)
                .afterTime(0, () -> {
                    drive.clawRotatorLeft.setPosition(ClawRotatorPosition.Take_Autonomous);
                    drive.armRotatorLeft.setPosition(ArmRotatorPosition.Stack);
                    drive.armRotatorRight.setPosition(ArmRotatorPosition.Stack);
                })
                .afterTime(1,()->{
                    drive.clawRight.setPosition(ClawPosition.OPEN);
                })
                .waitSeconds(1.2)
                .strafeToLinearHeading(new Vector2d(-47.21,10),Math.toRadians(-180))
                .lineToX(-55)//-54.21

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
                    drive.runLiftToPosition(-0.4 ,-850);
                })
                .splineToLinearHeading(new Pose2d(55, 34.5,Math.toRadians(-192)), Math.toRadians(-10))
                .afterTime(3.5,()->{
                    drive.clawRight.setPosition(ClawPosition.OPEN);
                    drive.clawLeft.setPosition(ClawPosition.OPEN);
                })
                .afterTime(4.5,()->{
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

    public void Caz3() {

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
}
