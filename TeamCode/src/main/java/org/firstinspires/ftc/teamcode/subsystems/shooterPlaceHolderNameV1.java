package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.controllers.Controller;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class shooterPlaceHolderNameV1 {
    private DcMotor left, right;
    Controller lC, rC;

    public shooterPlaceHolderNameV1(String lName, String rName, HardwareMap hardwareMap) {
        left = hardwareMap.get(DcMotor.class, lName);
        right = hardwareMap.get(DcMotor.class, rName);
        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setPower(double power){
        left.setPower(power);
        right.setPower(power);
    }
}
