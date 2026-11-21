package org.firstinspires.ftc.teamcode.comp;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.skeletonarmy.marrow.BulkCaching;
import org.firstinspires.ftc.teamcode.Utils.FieldConstants;
import org.firstinspires.ftc.teamcode.Utils.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@BulkCaching
@TeleOp(name = "BIOBUZZ TeleOp", group = "Competition")
public class TeleOpMain extends OpMode {
    private Follower follower;
    private Turret turret;
    private Shooter shooter;
    private Intake intake;
    private VoltageSensor voltageSensor;

    private Command activeShot;
    private Vector2D targetHive;
    private boolean useHive2 = false;
    private boolean sotmEnabled = false;
    private boolean autoAim = false;

    private boolean prevTriangle, prevSquare, prevCircle, prevLb, prevBack;

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(OpModeStorage.autoEndPose);
        follower.update();

        turret = new Turret(hardwareMap, follower::pose, follower::velocity);
        turret.setCurrentAngleRad(OpModeStorage.autoEndTurretRad);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        targetHive = OpModeStorage.mirror(FieldConstants.HIVE_1, OpModeStorage.alliance);
    }

    @Override
    public void loop() {
        double voltage = voltageSensor.getVoltage();

        if (gamepad1.triangle && !prevTriangle) {
            useHive2 = !useHive2;
            targetHive = OpModeStorage.mirror(useHive2 ? FieldConstants.HIVE_2 : FieldConstants.HIVE_1, OpModeStorage.alliance);
        }
        if (gamepad1.square && !prevSquare) sotmEnabled = !sotmEnabled;
        if (gamepad1.left_bumper && !prevLb) autoAim = !autoAim;
        if (gamepad1.back && !prevBack) turret.resetZero();
        if (gamepad1.dpad_down) {
            Pose corner = OpModeStorage.mirror(new Pose(8, 7, Math.PI / 2.0), OpModeStorage.alliance);
            follower.setPose(corner);
        }
        prevTriangle = gamepad1.triangle;
        prevSquare = gamepad1.square;
        prevLb = gamepad1.left_bumper;
        prevBack = gamepad1.back;

        double slow = gamepad1.left_trigger > 0.2 ? 0.4 : 1.0;
        double fwd = deadband(-gamepad1.left_stick_y, 0.05) * slow;
        double str = deadband(-gamepad1.left_stick_x, 0.05) * slow;
        double rawTurn = deadband(-gamepad1.right_stick_x, 0.05);
        double turn = Math.copySign(rawTurn * rawTurn, rawTurn) * slow;
//        ManualDrive.driveOrHold(follower, fwd, str, turn, 0.2, follower.pose().heading());
        follower.manual(fwd, str, turn);

        if (autoAim && turret.hasEncoder()) {
//            turret.aimAt(targetHive, sotmEnabled);
        } else {
            double nudge = (gamepad1.dpad_left ? 0.1 : 0.0) - (gamepad1.dpad_right ? 0.1 : 0.0);
            turret.setOpenLoop(nudge);
        }

        if (gamepad1.circle && !prevCircle) {
            activeShot = shooter.shoot(intake, turret, () -> shooter.rpmForDist(turret.dist()), 1, true);
            Scheduler.schedule(activeShot);
        }
        prevCircle = gamepad1.circle;

        if (activeShot == null || !Scheduler.isScheduled(activeShot)) {
            if (gamepad1.right_trigger > 0.1) intake.setPower(gamepad1.right_trigger, voltage);
            else if (gamepad1.right_bumper) intake.setPower(-0.8, voltage);
            else intake.stop();
        }

        follower.update();
        Scheduler.execute();
        turret.update(voltage);
        shooter.update(voltage);

        telemetry.addData("Alliance / Target", "%s | %s", OpModeStorage.alliance, useHive2 ? "HIVE 2" : "HIVE 1");
        telemetry.addData("SOTM / AutoAim", "%s | %s", sotmEnabled ? "ON" : "OFF", autoAim ? "ON" : "MANUAL");
        telemetry.addData("Turret Deg", "%.1f", Math.toDegrees(turret.angleRad()));
        telemetry.addData("Dist / Wheel RPM", "%.1f in | %.0f RPM", turret.dist(), shooter.leftWheelRpm());
        telemetry.addData("Ready", shooter.ready() && turret.aligned());
        telemetry.addData("Position x:", follower.pose().x());
        telemetry.addData("Position y:", follower.pose().y());
        telemetry.addData("Position heading:", follower.pose().heading());
        telemetry.update();
    }

    @Override
    public void stop() {
        Scheduler.reset();
        shooter.shutdown();
        intake.stop();
        turret.setOpenLoop(0.0);
    }

    private static double deadband(double val, double band) {
        return Math.abs(val) < band ? 0.0 : val;
    }
}