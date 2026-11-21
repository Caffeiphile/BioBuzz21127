package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.controllers.Controller;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Utils.CachedMotor;

import java.util.function.Supplier;

public class Turret {
    private static final double TICKS_PER_TURRET_RAD =
            (RobotConstants.Turret.TICKS_PER_REV * RobotConstants.Turret.GEAR_RATIO) / (2.0 * Math.PI);

    private final CachedMotor motor;
    private final Supplier<Pose> poseSupplier;
    private final Supplier<Velocity> velSupplier;

    private final Controller pdController = Controller.pid(
            RobotConstants.Turret.KP,
            0.0,
            RobotConstants.Turret.KD
    );

    private final ElapsedTime stepTimer = new ElapsedTime();
    private double profilePos = 0.0;
    private double profileVel = 0.0;
    private boolean profileReady = false;

    private int zeroOffsetTicks = 0;
    private double targetRad = 0.0;
    private double virtualDist = 60.0;
    private double openLoopPwr = 0.0;
    private boolean closedLoop = false;

    public Turret(HardwareMap hw, Supplier<Pose> poseSupplier, Supplier<Velocity> velSupplier) {
        this.motor = new CachedMotor(hw, RobotConstants.Turret.MOTOR);
        this.poseSupplier = poseSupplier;
        this.velSupplier = velSupplier;

        this.motor.setDirection(RobotConstants.Turret.DIR);
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetZero();
    }

    /** Captures current physical position as 0 rad (straight forward). */
    public void resetZero() {
        setCurrentAngleRad(0.0);
    }

    /** Restores known turret angle (e.g., from OpModeStorage after Autonomous). */
    public void setCurrentAngleRad(double currentAngleRad) {
        int currentTicks = motor.motor.getCurrentPosition();
        zeroOffsetTicks = currentTicks - (int) Math.round(AngleUnit.normalizeRadians(currentAngleRad) * TICKS_PER_TURRET_RAD);
        profilePos = AngleUnit.normalizeRadians(currentAngleRad);
        profileVel = 0.0;
        profileReady = false;
        pdController.reset();
    }

    public boolean hasEncoder() {
        return true;
    }

    public double angleRad() {
        int ticks = motor.motor.getCurrentPosition() - zeroOffsetTicks;
        return AngleUnit.normalizeRadians(ticks / TICKS_PER_TURRET_RAD);
    }

    public void setOpenLoop(double power) {
        closedLoop = false;
        openLoopPwr = Range.clip(power, -0.5, 0.5);
    }

    public void aimAt(Vector2D target, boolean sotm) {
        Pose pose = poseSupplier.get();
        Velocity vel = sotm ? velSupplier.get() : null;
        double vx = (vel != null) ? vel.vx : 0.0;
        double vy = (vel != null) ? vel.vy : 0.0;

        double dx = target.x() - pose.x();
        double dy = target.y() - pose.y();
        double rawDist = Math.hypot(dx, dy);

        double flightTime = rawDist / RobotConstants.Turret.SHOT_SPEED_IN_PER_SEC;
        double leadX = dx - vx * flightTime;
        double leadY = dy - vy * flightTime;

        virtualDist = Math.hypot(leadX, leadY);
        double fieldAngle = Math.atan2(leadY, leadX);
        targetRad = AngleUnit.normalizeRadians(fieldAngle - pose.heading());
        closedLoop = true;
    }

    public void update(double voltage) {
        if (!closedLoop) {
            profileReady = false;
            pdController.reset();
            motor.setPower(openLoopPwr);
            return;
        }

        double current = angleRad();
        if (!profileReady) {
            profilePos = current;
            profileVel = 0.0;
            stepTimer.reset();
            profileReady = true;
        }

        double dt = Math.min(stepTimer.seconds(), 0.05);
        stepTimer.reset();

        double errToGoal = AngleUnit.normalizeRadians(targetRad - profilePos);
        double maxStepVel = Math.min(
                RobotConstants.Turret.MAX_VEL,
                Math.sqrt(2.0 * RobotConstants.Turret.MAX_ACCEL * Math.abs(errToGoal))
        ) * Math.signum(errToGoal);

        double maxDv = RobotConstants.Turret.MAX_ACCEL * dt;
        double nextVel = profileVel + Range.clip(maxStepVel - profileVel, -maxDv, maxDv);
        double accel = (dt > 1e-4) ? (nextVel - profileVel) / dt : 0.0;
        profileVel = nextVel;
        profilePos = AngleUnit.normalizeRadians(profilePos + profileVel * dt);

        double posError = AngleUnit.normalizeRadians(profilePos - current);
        double ff = RobotConstants.Turret.KS * Math.signum(profileVel)
                + RobotConstants.Turret.KV * profileVel
                + RobotConstants.Turret.KA * accel;
        double fb = pdController.calculate(profilePos, posError);

        double vScale = RobotConstants.NOMINAL_VOLTAGE / Math.max(voltage, 8.0);
        motor.setPower(Range.clip((ff + fb) * vScale, -1.0, 1.0));
    }

    public double dist() {
        return virtualDist;
    }

    public boolean aligned() {
        return Math.abs(AngleUnit.normalizeRadians(targetRad - angleRad())) <= RobotConstants.Turret.TOLERANCE_RAD;
    }

    public Command track(Supplier<Vector2D> targetSupplier, Supplier<Boolean> sotmSupplier) {
        return Command.build()
                .requiring(this)
                .setExecute(() -> aimAt(targetSupplier.get(), sotmSupplier.get()))
                .setEnd(r -> setOpenLoop(0.0));
    }
}