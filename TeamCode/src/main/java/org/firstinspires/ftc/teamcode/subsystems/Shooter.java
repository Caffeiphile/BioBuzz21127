package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.controllers.Controller;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Utils.CachedMotor;

import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;
import java.util.function.DoubleSupplier;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;

public class Shooter {
    private final CachedMotor left, right;
    private final Controller leftPid, rightPid;
    private final Controller ff = Controller.proportionalFeedforward(RobotConstants.Outtake.KF);
    private final TreeMap<Double, Double> rpmTable = new TreeMap<>();

    private double targetWheelRpm = 10000.0;
    private double leftWheelRpm = 0.0;
    private double rightWheelRpm = 0.0;
    private boolean closedLoop = false;
    private boolean idleHoldActive = true;

    public Shooter(HardwareMap hw) {
        left = new CachedMotor(hw, RobotConstants.Outtake.LEFT_MOTOR);
        right = new CachedMotor(hw, RobotConstants.Outtake.RIGHT_MOTOR);

        left.setDirection(RobotConstants.Outtake.LEFT_DIR);
        right.setDirection(RobotConstants.Outtake.RIGHT_DIR);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftPid = Controller.pid(RobotConstants.Outtake.KP, 0.0, RobotConstants.Outtake.KD);
        rightPid = Controller.pid(RobotConstants.Outtake.KP, 0.0, RobotConstants.Outtake.KD);

        rpmTable.put(36.0, 6000.0);
        rpmTable.put(60.0, 6000.0);
        rpmTable.put(84.0, 6000.0);
        rpmTable.put(108.0, 6000.0);
        rpmTable.put(132.0, 6000.0);
    }

    public void setPower(double power) {
        closedLoop = false;
        targetWheelRpm = 0.0;
        idleHoldActive = false;
        left.setPower(power);
        right.setPower(power);
    }

    public void setRpm(double wheelRpm) {
        closedLoop = true;
        idleHoldActive = false;
        targetWheelRpm = Range.clip(wheelRpm, 0.0, RobotConstants.Outtake.MAX_WHEEL_RPM);
    }

    public double rpmForDist(double inches) {
        return lerp(rpmTable, inches);
    }

    public void setDist(double inches) {
        setRpm(rpmForDist(inches));
    }

    public void update(double voltage) {
        leftWheelRpm = ticksToWheelRpm(left.getVelocity());
        rightWheelRpm = ticksToWheelRpm(right.getVelocity());

        double vScale = RobotConstants.NOMINAL_VOLTAGE / Math.max(voltage, 8.0);

        if (!closedLoop || targetWheelRpm <= 1.0) {
            if (idleHoldActive) {
                double holdPwr = Range.clip(RobotConstants.Outtake.IDLE_HOLD_PWR * vScale, -0.5, 0.0);
                left.setPower(holdPwr);
                right.setPower(holdPwr);
            }
            return;
        }

        left.setPower(computePower(targetWheelRpm, leftWheelRpm, leftPid, vScale));
        right.setPower(computePower(targetWheelRpm, rightWheelRpm, rightPid, vScale));
    }

    private double computePower(double target, double current, Controller pid, double vScale) {
        double error = target - current;
        if (error > RobotConstants.Outtake.BANG_BANG_RPM) {
            return RobotConstants.Outtake.BANG_BANG_PWR;
        }
        double raw = (Math.abs(error) <= RobotConstants.Outtake.DEADBAND_RPM)
                ? ff.calculate(target, error)
                : ff.calculate(target, error) + pid.calculate(target, error);
        return Range.clip(raw * vScale, -1.0, 1.0);
    }

    public double leftWheelRpm() { return leftWheelRpm; }
    public double rightWheelRpm() { return rightWheelRpm; }
    public double leftMotorRpm() { return leftWheelRpm / RobotConstants.Outtake.GEAR_RATIO; }
    public double rightMotorRpm() { return rightWheelRpm / RobotConstants.Outtake.GEAR_RATIO; }

    public boolean ready() {
        return closedLoop
                && targetWheelRpm > 1.0
                && Math.abs(targetWheelRpm - leftWheelRpm) <= RobotConstants.Outtake.DEADBAND_RPM
                && Math.abs(targetWheelRpm - rightWheelRpm) <= RobotConstants.Outtake.DEADBAND_RPM;
    }

    /** Returns to gentle idle reverse hold during active OpModes. */
    public void stop() {
        leftPid.reset();
        rightPid.reset();
        closedLoop = false;
        targetWheelRpm = 0.0;
        idleHoldActive = true;
    }

    /** Completely cuts power when an OpMode finishes. */
    public void shutdown() {
        leftPid.reset();
        rightPid.reset();
        setPower(0.0);
    }

    private static double ticksToWheelRpm(double ticksPerSec) {
        return (ticksPerSec * 60.0 / RobotConstants.Outtake.TICKS_PER_REV) * RobotConstants.Outtake.GEAR_RATIO;
    }

    public static double lerp(NavigableMap<Double, Double> map, double x) {
        Map.Entry<Double, Double> lo = map.floorEntry(x);
        Map.Entry<Double, Double> hi = map.ceilingEntry(x);
        if (lo == null) return hi.getValue();
        if (hi == null || lo.getKey().equals(hi.getKey())) return lo.getValue();
        double t = (x - lo.getKey()) / (hi.getKey() - lo.getKey());
        return lo.getValue() + t * (hi.getValue() - lo.getValue());
    }

    // --- IVY COMMAND FACTORIES ---
    public Command spinUp(Intake intake, DoubleSupplier rpmSupplier) {
        ElapsedTime backoffTimer = new ElapsedTime();
        ElapsedTime rampTimer = new ElapsedTime();

        Command backoff = parallel(
                intake.backoff(),
                Command.build()
                        .requiring(this)
                        .setStart(() -> {
                            backoffTimer.reset();
                            setPower(RobotConstants.Outtake.BACKOFF_PWR);
                        })
                        .setDone(() -> backoffTimer.seconds() >= RobotConstants.Intake.BACKOFF_SEC)
                        .setEnd(r -> setPower(0.0))
        );

        Command ramp = Command.build()
                .requiring(this)
                .setStart(rampTimer::reset)
                .setExecute(() -> setRpm(rpmSupplier.getAsDouble()))
                .setDone(() -> ready() || rampTimer.seconds() >= 1.0);

        return sequential(backoff, ramp);
    }

    public Command shoot(Intake intake, Turret turret, DoubleSupplier rpmSupplier, double feedSec) {
        return shoot(intake, turret, rpmSupplier, feedSec, false);
    }

    public Command shoot(Intake intake, Turret turret, DoubleSupplier rpmSupplier, double feedSec, boolean alreadySpunUp) {
        ElapsedTime waitTimer = new ElapsedTime();
        ElapsedTime feedTimer = new ElapsedTime();

        Command waitReady = Command.build()
                .requiring(this)
                .setStart(waitTimer::reset)
                .setExecute(() -> setRpm(rpmSupplier.getAsDouble()))
                .setDone(() -> (ready() && turret.aligned()) || waitTimer.seconds() >= 0.75);

        Command feedAndFire = parallel(
                Command.build()
                        .requiring(this)
                        .setStart(feedTimer::reset)
                        .setExecute(() -> setRpm(rpmSupplier.getAsDouble()))
                        .setDone(() -> feedTimer.seconds() >= feedSec),
                intake.feed(feedSec)
        );

        if (alreadySpunUp) {
            return sequential(waitReady, feedAndFire, instant(this::stop));
        }
        return sequential(spinUp(intake, rpmSupplier), waitReady, feedAndFire, instant(this::stop));
    }
}