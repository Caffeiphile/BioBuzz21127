package org.firstinspires.ftc.teamcode.debug;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.tuning.autotune.*;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.Utils.FieldConstants;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.Locale;
import java.util.Map;
import java.util.TreeMap;

public class OuttakeTuner extends Procedure {

    public OuttakeTuner() {
        super("Outtake Distance-RPM Tuner", "Sample ideal flywheel RPMs at multiple field distances with live backoff + shot testing.");
    }

    @Override
    public void run() throws InterruptedException {
        TreeMap<Double, Double> samples = new TreeMap<>();

        Inputs setup = inputs("Setup", "Place the robot at your starting pose (60, 8, 90°) or enter manual tape distances each round.");
        Inputs.Field<Boolean> useOdometryDist = setup.b("Use Pinpoint Distance to HIVE_1").withDefault(true);
        awaitInputs(setup);

        boolean keepSampling = true;
        double lastRpm = 1800.0;

        while (keepSampling) {
            Inputs pointInput = inputs("Test Point Configuration", "Set distance (if manual) and starting Wheel RPM to test.");
            Inputs.Field<Double> manualDist = pointInput.d("Manual Distance (inches, ignored if Pinpoint enabled)").withDefault(60.0);
            Inputs.Field<Double> startRpm = pointInput.d("Starting Wheel RPM").withDefault(lastRpm);
            awaitInputs(pointInput);

            double[] measured = runOpMode(new LiveShotTest(
                    useOdometryDist.get(),
                    manualDist.get(),
                    startRpm.get()
            ));

            double recordedDist = measured[0];
            double recordedRpm = measured[1];
            lastRpm = recordedRpm;

            Inputs confirmPoint = inputs("Record Sample", String.format(Locale.US,
                    "Measured Distance: %.1f in | Final Wheel RPM: %.0f RPM", recordedDist, recordedRpm));
            Inputs.Field<Boolean> savePoint = confirmPoint.b("Save this data point?").withDefault(true);
            Inputs.Field<Double> overrideDist = confirmPoint.d("Distance to save (in)").withDefault(Math.round(recordedDist * 10.0) / 10.0);
            Inputs.Field<Double> overrideRpm = confirmPoint.d("Wheel RPM to save").withDefault(recordedRpm);
            Inputs.Field<Boolean> addAnother = confirmPoint.b("Test another distance?").withDefault(true);
            awaitInputs(confirmPoint);

            if (savePoint.get()) {
                samples.put(overrideDist.get(), overrideRpm.get());
                result(String.format(Locale.US, "dist_%.1f_in", overrideDist.get()), overrideRpm.get());
            }
            keepSampling = addAnother.get();
        }

        if (samples.isEmpty()) {
            abort("No distance-RPM samples were saved.");
        }

        StringBuilder sb = new StringBuilder("// Paste into Shooter.java constructor:\n");
        for (Map.Entry<Double, Double> e : samples.entrySet()) {
            sb.append(String.format(Locale.US, "rpmTable.put(%.1f, %.1f);\n", e.getKey(), e.getValue()));
        }
        code(Language.JAVA, sb.toString());
    }
}

class LiveShotTest extends TuningOpMode<double[]> {
    private final boolean useOdo;
    private final double manualDist;
    private double testRpm;

    public LiveShotTest(boolean useOdo, double manualDist, double startRpm) {
        super("Live Outtake Test",
                "Circle/B: Backoff & Shoot | D-Pad Up/Down: +/-25 RPM | RT/RB: Intake/Eject | Click Stop when dialed in.",
                true);
        this.useOdo = useOdo;
        this.manualDist = manualDist;
        this.testRpm = startRpm;
    }

    @Override
    protected double[] runTuningOpMode() {
        Scheduler.reset();
        Follower follower = Constants.create(hardwareMap);
        follower.setPose(FieldConstants.RED_START);
        follower.update();

        Turret turret = new Turret(hardwareMap, follower::pose, follower::velocity);
        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

        Vector2D hive = FieldConstants.HIVE_1;
        Command activeShot = null;
        boolean prevUp = false, prevDown = false, prevFire = false, prevZero = false;
        double liveDist = manualDist;

        waitForStart();

        while (opModeIsActive()) {
            double voltage = voltageSensor.getVoltage();
            follower.update();

            if (useOdo) {
                Pose p = follower.pose();
                liveDist = Math.hypot(hive.x() - p.x(), hive.y() - p.y());
            }

            if (gamepad1.dpad_up && !prevUp) testRpm = Math.min(RobotConstants.Outtake.MAX_WHEEL_RPM, testRpm + 25.0);
            if (gamepad1.dpad_down && !prevDown) testRpm = Math.max(400.0, testRpm - 25.0);
            if (gamepad1.back && !prevZero) turret.resetZero();

            boolean fireBtn = gamepad1.circle || gamepad1.b;
            if (fireBtn && !prevFire) {
                final double shotRpm = testRpm;
                activeShot = shooter.shoot(intake, turret, () -> shotRpm, 1.5);
                Scheduler.schedule(activeShot);
            }

            prevUp = gamepad1.dpad_up;
            prevDown = gamepad1.dpad_down;
            prevFire = fireBtn;
            prevZero = gamepad1.back;

            // Keep turret stationary (open-loop 0) while loose, or nudge gently with D-pad Left/Right
            double nudge = (gamepad1.dpad_left ? 0.20 : 0.0) - (gamepad1.dpad_right ? 0.20 : 0.0);
            turret.setOpenLoop(nudge);

            if (activeShot == null || !Scheduler.isScheduled(activeShot)) {
                if (gamepad1.right_trigger > 0.1) intake.setPower(gamepad1.right_trigger, voltage);
                else if (gamepad1.right_bumper) intake.setPower(-0.8, voltage);
                else intake.stop();
            }

            Scheduler.execute();
            turret.update(voltage);
            shooter.update(voltage);

            telemetry.addData("Distance", "%.1f in (%s)", liveDist, useOdo ? "Pinpoint" : "Manual");
            telemetry.addData("Test Wheel RPM", "%.0f RPM (Motor: %.0f RPM)", testRpm, testRpm / RobotConstants.Outtake.GEAR_RATIO);
            telemetry.addData("Actual L / R RPM", "%.0f / %.0f", shooter.leftWheelRpm(), shooter.rightWheelRpm());
            telemetry.update();
        }

        Scheduler.reset();
        shooter.setPower(0.0);
        intake.stop();
        turret.setOpenLoop(0.0);
        return new double[]{liveDist, testRpm};
    }
}