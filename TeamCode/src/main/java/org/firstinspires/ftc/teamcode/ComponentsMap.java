package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants.ArmRotatorPosition;
import org.firstinspires.ftc.teamcode.Constants.ClawPosition;
import org.firstinspires.ftc.teamcode.Constants.ClawRotatorPosition;

public class ComponentsMap {

    public DcMotorEx liftLeft, liftRight, parkLiftLeft, parkLiftRight;

    public Servo armRotatorLeft, armRotatorRight, clawLeft, clawRotatorLeft, clawRight, parkServoLeft, parkServoRight;

    public ComponentsMap(HardwareMap hardwareMap)
    {
        //Parcare pe bari
        parkLiftLeft = hardwareMap.get(DcMotorEx.class, "parkLiftLeft");
        parkLiftRight = hardwareMap.get(DcMotorEx.class, "parkLiftRight");

        parkServoLeft = hardwareMap.get(Servo.class, "parkingLeft");
        parkServoRight = hardwareMap.get(Servo.class, "parkingRight");

        // Motoare ridicare glisiera
        liftLeft = hardwareMap.get(DcMotorEx.class, "liftLeft");
        liftRight = hardwareMap.get(DcMotorEx.class, "liftRight");

        // Rotatie cleste
        clawRotatorLeft = hardwareMap.get(Servo.class, "clawRotatorLeft");

        // Rotatie BRAT - Axon
        armRotatorLeft = hardwareMap.get(Servo.class, "armRotatorLeft");
        armRotatorRight = hardwareMap.get(Servo.class, "armRotatorRight");

        // Cleste
        clawLeft = hardwareMap.get(Servo.class, "clawLeft");
        clawRight = hardwareMap.get(Servo.class, "clawRight");

        // Reverse la motoare, unde e cazul:
        liftLeft.setDirection(DcMotorEx.Direction.REVERSE);

        // Reverse la servo
        clawRotatorLeft.setDirection(Servo.Direction.REVERSE);
    }

    public void initTeleOp(){
        clawLeft.setPosition(ClawPosition.OPEN);
        clawRight.setPosition(ClawPosition.OPEN);
        armRotatorLeft.setPosition(ArmRotatorPosition.DOWN);
        armRotatorRight.setPosition(ArmRotatorPosition.DOWN);
        clawRotatorLeft.setPosition(ClawRotatorPosition.TAKE);
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}
