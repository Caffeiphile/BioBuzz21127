package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Wraps DcMotorEx to suppress redundant I2C/USB power writes when delta < epsilon,
 * saving ~3-4ms per skipped write on an 8-motor robot. Always passes exact 0.0 stops.
 */
public class CachedMotor {
    private static final double EPSILON = 0.005;

    public final DcMotorEx motor;
    private double lastPower = Double.NaN;

    public CachedMotor(HardwareMap hw, String name) {
        this.motor = hw.get(DcMotorEx.class, name);
    }

    public void setPower(double power) {
        boolean stopping = (power == 0.0 && lastPower != 0.0);
        if (stopping || Double.isNaN(lastPower) || Math.abs(power - lastPower) >= EPSILON) {
            motor.setPower(power);
            lastPower = power;
        }
    }

    public double getVelocity() {
        return motor.getVelocity();
    }

    public void setDirection(DcMotorSimple.Direction dir) {
        motor.setDirection(dir);
    }

    public void setMode(DcMotor.RunMode mode) {
        motor.setMode(mode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        motor.setZeroPowerBehavior(behavior);
    }
}