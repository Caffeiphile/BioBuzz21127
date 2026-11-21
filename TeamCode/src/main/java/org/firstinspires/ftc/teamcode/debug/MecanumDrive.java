package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (group = "Testing", name = "Mecanum Drive")
public class MecanumDrive extends OpMode {
    DcMotor fl, fr, bl, br;
    DcMotor intake;
    @Override
    public void init() {
        fl = hardwareMap.get(DcMotor.class, "FL");
        fr = hardwareMap.get(DcMotor.class, "FR");
        bl = hardwareMap.get(DcMotor.class, "BL");
        br = hardwareMap.get(DcMotor.class, "BR");
        intake = hardwareMap.get(DcMotor.class, "intake");
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    @Override
    public void loop() {

        intake.setPower(gamepad1.right_trigger);

        double y = gamepad1.left_stick_y, x = -gamepad1.left_stick_x, r = -gamepad1.right_stick_x;
        if(y!=0||x!=0||r!=0) {
            double norm = Math.max(1, Math.abs(x) + Math.abs(y) + Math.abs(r));
            fl.setPower((y + x + r) / norm);
            fr.setPower((y - x - r) / norm);
            bl.setPower((y - x + r) / norm);
            br.setPower((y + x - r) / norm);
        } else {
            fl.setPower(gamepad1.triangle?1:0);
            fr.setPower(gamepad1.circle?1:0);
            bl.setPower(gamepad1.square?1:0);
            br.setPower(gamepad1.cross?1:0);
        }
    }
}
