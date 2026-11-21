package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Utils.CachedMotor;

public class Intake {
    private final CachedMotor motor;

    public Intake(HardwareMap hw) {
        motor = new CachedMotor(hw, RobotConstants.Intake.MOTOR);
        motor.setDirection(RobotConstants.Intake.DIR);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setPower(double power) {
        motor.setPower(Range.clip(power, -1.0, 1.0));
    }

    public void setPower(double power, double voltage) {
        double vScale = RobotConstants.NOMINAL_VOLTAGE / Math.max(voltage, 8.0);
        setPower(power * vScale);
    }

    public void stop() {
        motor.setPower(0.0);
    }

    public Command run(double power) {
        return Command.build()
                .requiring(this)
                .setExecute(() -> setPower(power))
                .setEnd(r -> stop());
    }

    public Command backoff() {
        ElapsedTime timer = new ElapsedTime();
        return Command.build()
                .requiring(this)
                .setStart(() -> {
                    timer.reset();
                    setPower(RobotConstants.Intake.BACKOFF_PWR);
                })
                .setDone(() -> timer.seconds() >= RobotConstants.Intake.BACKOFF_SEC)
                .setEnd(r -> stop());
    }

    public Command feed(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        return Command.build()
                .requiring(this)
                .setStart(() -> {
                    timer.reset();
                    setPower(0.45);
                })
                .setDone(() -> timer.seconds() >= seconds)
                .setEnd(r -> stop());
    }
}