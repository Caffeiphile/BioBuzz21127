package org.firstinspires.ftc.teamcode.debug;

import com.pedropathing.tuning.autotune.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.RobotConstants;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static com.pedropathing.utils.Utils.linearFit;

public class MechanismTuner extends Procedure {

    private enum MotorDir {
        @DisplayName("Forward") FORWARD,
        @DisplayName("Reversed") REVERSE
    }

    public MechanismTuner() {
        super("Mechanism Tuner", "Polarity and SysID tuner for flywheels, 6:1 encoder turret, and intake.");
    }

    @Override
    public void run() throws InterruptedException {
        Inputs hw = inputs("Mechanism Configuration", "Verify HardwareMap names and gearing (bare motors = 28 ticks/rev).");
        Inputs.Field<String> leftFwName = hw.s("Left Flywheel Motor").withDefault(RobotConstants.Outtake.LEFT_MOTOR);
        Inputs.Field<String> rightFwName = hw.s("Right Flywheel Motor").withDefault(RobotConstants.Outtake.RIGHT_MOTOR);
        Inputs.Field<String> turretName = hw.s("Turret Motor").withDefault(RobotConstants.Turret.MOTOR);
        Inputs.Field<String> intakeName = hw.s("Intake Motor").withDefault(RobotConstants.Intake.MOTOR);
        Inputs.Field<Double> bareTicks = hw.d("Bare Motor Ticks/Rev").withDefault(28.0);
        Inputs.Field<Double> fwGearRatio = hw.d("Flywheel Ratio (wheel/motor)").withDefault(RobotConstants.Outtake.GEAR_RATIO);
        Inputs.Field<Double> turretGearRatio = hw.d("Turret Reduction (motor/turret)").withDefault(RobotConstants.Turret.GEAR_RATIO);
        awaitInputs(hw);

        confirmation("Motor Polarity Checks", "Each motor will spin gently one at a time. Click Stop once you observe its direction.");

        MotorDir leftFwDir = testMotor("Left Flywheel", leftFwName.get(), 0.25, true, "Should spin in the shooting direction.");
        MotorDir rightFwDir = testMotor("Right Flywheel", rightFwName.get(), 0.25, true, "Should spin in the shooting direction (matching Left Flywheel).");
        MotorDir intakeDir = testMotor("Intake", intakeName.get(), 0.35, false, "Should pull elements inward toward the transfer.");
        MotorDir turretDir = testMotor("Turret", turretName.get(), 0.20, false, "Should rotate counter-clockwise when viewed from above.");

        confirmation("Turret SysID (6:1 Motor Encoder)", "Align the turret straight forward with ~150° of clear travel. It will spin for 0.85s to identify kV, kA, kP, and kD.");
        double[] turretSysId = step(new TurretSysId(
                turretName.get(),
                toSdkDir(turretDir),
                bareTicks.get(),
                turretGearRatio.get()
        ));

        confirmation("Flywheel SysID (kF, kP, kD)", "Clear the shooter! Both flywheels will spin at 70% power for 2.5s to measure steady-state RPM and spin-up time constant.");
        double[] fwSysId = step(new FlywheelSysId(
                leftFwName.get(),
                toSdkDir(leftFwDir),
                rightFwName.get(),
                toSdkDir(rightFwDir),
                fwGearRatio.get(),
                bareTicks.get()
        ));

        result("leftFlywheelDirection", leftFwDir);
        result("rightFlywheelDirection", rightFwDir);
        result("intakeDirection", intakeDir);
        result("turretDirection", turretDir);
        result("outtake_kF", fwSysId[0]);
        result("outtake_kP", fwSysId[1]);
        result("outtake_kD", fwSysId[2]);
        result("leftSteadyWheelRPM", fwSysId[3]);
        result("rightSteadyWheelRPM", fwSysId[4]);
        result("batteryVoltage", fwSysId[5]);
        result("turret_kV", turretSysId[0]);
        result("turret_kA", turretSysId[1]);
        result("turret_kP", turretSysId[2]);
        result("turret_kD", turretSysId[3]);

        code(Language.JAVA, String.format(Locale.US,
                "package org.firstinspires.ftc.teamcode.subsystems;\n\n" +
                        "import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;\n\n" +
                        "public class RobotConstants {\n" +
                        "    public static final double NOMINAL_VOLTAGE = 12.0;\n\n" +
                        "    public static class Intake {\n" +
                        "        public static final String MOTOR = \"%s\";\n" +
                        "        public static final Direction DIR = Direction.%s;\n" +
                        "        public static final double BACKOFF_PWR = %.2f;\n" +
                        "        public static final double BACKOFF_SEC = %.2f;\n" +
                        "    }\n\n" +
                        "    public static class Outtake {\n" +
                        "        public static final String LEFT_MOTOR = \"%s\";\n" +
                        "        public static final String RIGHT_MOTOR = \"%s\";\n" +
                        "        public static final Direction LEFT_DIR = Direction.%s;\n" +
                        "        public static final Direction RIGHT_DIR = Direction.%s;\n\n" +
                        "        public static final double TICKS_PER_REV = %.1f;\n" +
                        "        public static final double GEAR_RATIO = %.2f;\n" +
                        "        public static final double MAX_WHEEL_RPM = %.1f;\n\n" +
                        "        public static final double KF = %.6f; // %.0f / %.0f wheel RPM @ %.2fV\n" +
                        "        public static final double KP = %.6f;\n" +
                        "        public static final double KD = %.6f;\n\n" +
                        "        public static final double BANG_BANG_RPM = %.1f;\n" +
                        "        public static final double BANG_BANG_PWR = %.2f;\n" +
                        "        public static final double DEADBAND_RPM = %.1f;\n" +
                        "        public static final double BACKOFF_PWR = %.2f;\n" +
                        "    }\n\n" +
                        "    public static class Turret {\n" +
                        "        public static final String MOTOR = \"%s\";\n" +
                        "        public static final Direction DIR = Direction.%s;\n\n" +
                        "        public static final double TICKS_PER_REV = %.1f;\n" +
                        "        public static final double GEAR_RATIO = %.2f;\n\n" +
                        "        public static final double KS = %.4f;\n" +
                        "        public static final double KV = %.5f;\n" +
                        "        public static final double KA = %.5f;\n" +
                        "        public static final double KP = %.4f;\n" +
                        "        public static final double KD = %.4f;\n\n" +
                        "        public static final double MAX_VEL = %.2f;\n" +
                        "        public static final double MAX_ACCEL = %.2f;\n" +
                        "        public static final double TOLERANCE_RAD = Math.toRadians(%.1f);\n" +
                        "        public static final double SHOT_SPEED_IN_PER_SEC = %.1f;\n" +
                        "    }\n" +
                        "}",
                intakeName.get(), intakeDir,
                RobotConstants.Intake.BACKOFF_PWR, RobotConstants.Intake.BACKOFF_SEC,
                leftFwName.get(), rightFwName.get(), leftFwDir, rightFwDir,
                bareTicks.get(), fwGearRatio.get(), RobotConstants.Outtake.MAX_WHEEL_RPM,
                fwSysId[0], fwSysId[3], fwSysId[4], fwSysId[5],
                fwSysId[1], fwSysId[2],
                RobotConstants.Outtake.BANG_BANG_RPM, RobotConstants.Outtake.BANG_BANG_PWR,
                RobotConstants.Outtake.DEADBAND_RPM, RobotConstants.Outtake.BACKOFF_PWR,
                turretName.get(), turretDir,
                bareTicks.get(), turretGearRatio.get(),
                RobotConstants.Turret.KS,
                turretSysId[0], turretSysId[1], turretSysId[2], turretSysId[3],
                RobotConstants.Turret.MAX_VEL, RobotConstants.Turret.MAX_ACCEL,
                Math.toDegrees(RobotConstants.Turret.TOLERANCE_RAD),
                RobotConstants.Turret.SHOT_SPEED_IN_PER_SEC
        ));
    }

    private MotorDir testMotor(String label, String hwName, double power, boolean floatMode, String hint) throws InterruptedException {
        step(new SpinMechanismMotor(label, hwName, power, floatMode, hint));

        Inputs dirInput = inputs(label + " Direction", hint);
        Inputs.Field<Boolean> spun = dirInput.b("Did the " + label + " motor spin?").withDefault(true);
        Inputs.Field<MotorDir> dir = dirInput.e("Which direction makes " + label + " move properly?", MotorDir.class);
        awaitInputs(dirInput);

        if (!spun.get()) {
            abort(label + " ('" + hwName + "') did not spin. Check wiring/port configuration and run again.");
        }
        return dir.get();
    }

    private <T> T step(TuningOpMode<T> opMode) throws InterruptedException {
        try {
            return runOpMode(opMode);
        } catch (RuntimeException e) {
            abort(opMode.name + " failed: " + e.getMessage());
            throw e;
        }
    }

    private static DcMotorSimple.Direction toSdkDir(MotorDir dir) {
        return dir == MotorDir.FORWARD ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE;
    }
}

class SpinMechanismMotor extends TuningOpMode<Void> {
    private final String hwName;
    private final double power;
    private final boolean floatMode;

    public SpinMechanismMotor(String label, String hwName, double power, boolean floatMode, String hint) {
        super(label + " Polarity", hint + " Click Stop once you see which direction it rotates.", true);
        this.hwName = hwName;
        this.power = power;
        this.floatMode = floatMode;
    }

    @Override
    protected Void runTuningOpMode() {
        DcMotor motor = hardwareMap.dcMotor.get(hwName);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(floatMode ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        motor.setPower(power);
        while (opModeIsActive()) {
        }
        motor.setPower(0.0);
        return null;
    }
}

class TurretSysId extends TuningOpMode<double[]> {
    private static final double POWER = 0.22;
    private static final double RUNTIME = 0.85;
    private static final double ALPHA = 14.0;
    private static final int TAIL_SAMPLES = 10;

    private final String motorName;
    private final DcMotorSimple.Direction dir;
    private final double ticksPerTurretRad;

    public TurretSysId(String motorName, DcMotorSimple.Direction dir, double bareTicks, double gearRatio) {
        super("Turret SysID", "Stepping turret at 22% power for 0.85s using motor quadrature encoder.", false);
        this.motorName = motorName;
        this.dir = dir;
        this.ticksPerTurretRad = (bareTicks * gearRatio) / (2.0 * Math.PI);
    }

    @Override
    protected double[] runTuningOpMode() throws InterruptedException {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, motorName);
        motor.setDirection(dir);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        List<Double> times = new ArrayList<>();
        List<Double> vels = new ArrayList<>();
        ElapsedTime timer = new ElapsedTime();

        Thread.sleep(800);
        waitForStart();

        int startTicks = motor.getCurrentPosition();
        timer.reset();
        motor.setPower(POWER);

        while (!isStopRequested()) {
            double now = timer.seconds();
            int ticks = motor.getCurrentPosition() - startTicks;
            double rad = ticks / ticksPerTurretRad;
            double omega = Math.abs(motor.getVelocity() / ticksPerTurretRad);

            times.add(now);
            vels.add(omega);

            if (now >= RUNTIME || Math.abs(rad) > Math.toRadians(150.0)) {
                break;
            }
            Thread.sleep(5);
        }
        motor.setPower(0.0);

        int n = times.size();
        if (n < 6) {
            throw new IllegalStateException("Too few samples (" + n + ") during turret step.");
        }

        int start = Math.max(0, n - TAIL_SAMPLES);
        double sum = 0.0;
        for (int i = start; i < n; i++) sum += vels.get(i);
        double steadyVel = sum / (n - start);
        double k = steadyVel / POWER;
        if (k <= 0.1) {
            throw new IllegalStateException("Turret barely moved (steady vel = " + steadyVel + " rad/s). Check encoder cable.");
        }

        List<Double> x = new ArrayList<>();
        List<Double> y = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            double vNorm = vels.get(i) / POWER;
            if (vNorm < 0.1 * k || vNorm > 0.8 * k) continue;
            x.add(times.get(i));
            y.add(Math.log(k - vNorm));
        }

        double tau = 0.08;
        if (x.size() >= 2) {
            double[] fit = linearFit(x.toArray(new Double[0]), y.toArray(new Double[0]));
            if (Double.isFinite(fit[1]) && fit[1] < 0) {
                tau = -1.0 / fit[1];
            }
        }

        double kV = 1.0 / k;
        double kA = tau / k;
        double kP = (tau * ALPHA * ALPHA) / k;
        double kD = Math.max(0.0, (2.0 * tau * ALPHA - 1.0) / k);

        return new double[]{kV, kA, kP, kD};
    }
}

class FlywheelSysId extends TuningOpMode<double[]> {
    private static final double TEST_POWER = 0.70;
    private static final double SETTLE_SEC = 1.80;
    private static final double TOTAL_SEC = 2.50;
    private static final double ALPHA = 6.0;

    private final String leftName, rightName;
    private final DcMotorSimple.Direction leftDir, rightDir;
    private final double gearRatio, ticksPerRev;

    public FlywheelSysId(String leftName, DcMotorSimple.Direction leftDir,
                         String rightName, DcMotorSimple.Direction rightDir,
                         double gearRatio, double ticksPerRev) {
        super("Flywheel SysID", "Spinning dual flywheels at 70% power for 2.5s to identify kF, kP, and kD.", false);
        this.leftName = leftName;
        this.leftDir = leftDir;
        this.rightName = rightName;
        this.rightDir = rightDir;
        this.gearRatio = gearRatio;
        this.ticksPerRev = ticksPerRev;
    }

    @Override
    protected double[] runTuningOpMode() throws InterruptedException {
        DcMotorEx left = hardwareMap.get(DcMotorEx.class, leftName);
        DcMotorEx right = hardwareMap.get(DcMotorEx.class, rightName);
        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

        left.setDirection(leftDir);
        right.setDirection(rightDir);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        List<Double> times = new ArrayList<>();
        List<Double> rpms = new ArrayList<>();
        ElapsedTime timer = new ElapsedTime();

        double sumLeftRpm = 0.0;
        double sumRightRpm = 0.0;
        double sumV = 0.0;
        int steadySamples = 0;

        Thread.sleep(1000);
        waitForStart();

        timer.reset();
        left.setPower(TEST_POWER);
        right.setPower(TEST_POWER);

        while (!isStopRequested() && timer.seconds() < TOTAL_SEC) {
            double now = timer.seconds();
            double lRpm = (left.getVelocity() * 60.0 / ticksPerRev) * gearRatio;
            double rRpm = (right.getVelocity() * 60.0 / ticksPerRev) * gearRatio;
            double avgRpm = 0.5 * (lRpm + rRpm);

            times.add(now);
            rpms.add(avgRpm);

            if (now >= SETTLE_SEC) {
                sumLeftRpm += lRpm;
                sumRightRpm += rRpm;
                sumV += voltageSensor.getVoltage();
                steadySamples++;
            }
            Thread.sleep(10);
        }

        left.setPower(0.0);
        right.setPower(0.0);

        if (steadySamples == 0) {
            throw new IllegalStateException("No steady-state flywheel samples collected.");
        }

        double avgLeftRpm = sumLeftRpm / steadySamples;
        double avgRightRpm = sumRightRpm / steadySamples;
        double avgVoltage = sumV / steadySamples;
        double avgWheelRpm = 0.5 * (avgLeftRpm + avgRightRpm);

        if (avgWheelRpm < 200.0) {
            throw new IllegalStateException("Measured wheel RPM (" + avgWheelRpm + ") too low. Check motor encoders and polarity.");
        }

        double vScale = avgVoltage / RobotConstants.NOMINAL_VOLTAGE;
        double k = avgWheelRpm / (TEST_POWER * vScale);
        double kF = 1.0 / k;

        List<Double> x = new ArrayList<>();
        List<Double> y = new ArrayList<>();
        for (int i = 0; i < times.size(); i++) {
            double rpm = rpms.get(i);
            if (rpm < 0.1 * avgWheelRpm || rpm > 0.8 * avgWheelRpm) continue;
            x.add(times.get(i));
            y.add(Math.log(avgWheelRpm - rpm));
        }

        double tau = 0.35;
        if (x.size() >= 2) {
            double[] fit = linearFit(x.toArray(new Double[0]), y.toArray(new Double[0]));
            if (Double.isFinite(fit[1]) && fit[1] < 0) {
                tau = -1.0 / fit[1];
            }
        }

        double kP = (tau * ALPHA * ALPHA) / k;
        double kD = Math.max(0.0, (2.0 * tau * ALPHA - 1.0) / k);

        return new double[]{kF, kP, kD, avgLeftRpm, avgRightRpm, avgVoltage};
    }
}