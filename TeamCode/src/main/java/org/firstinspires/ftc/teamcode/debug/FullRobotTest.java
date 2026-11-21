package org.firstinspires.ftc.teamcode.debug;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.skeletonarmy.marrow.BulkCaching;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@BulkCaching
@TeleOp(name = "Full Robot Test", group = "Debug")
public class FullRobotTest extends OpMode {
    private Follower follower;
    private Shooter shooter;
    private Turret turret;
    private Intake intake;
    private VoltageSensor voltageSensor;

    private double testWheelRpm = 2000.0;
    private boolean shooterOn = false;
    private boolean prevUp, prevDown, prevA;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(0, 0, 0));
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap, follower::pose, follower::velocity);
        intake = new Intake(hardwareMap);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    @Override
    public void loop() {
        double voltage = voltageSensor.getVoltage();

        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y * 0.6,
                -gamepad1.left_stick_x * 0.6,
                -gamepad1.right_stick_x * 0.6,
                follower.pose().heading()
        );
        follower.manual(powers);

        if (gamepad1.dpad_up && !prevUp) testWheelRpm = Math.min(2800.0, testWheelRpm + 100.0);
        if (gamepad1.dpad_down && !prevDown) testWheelRpm = Math.max(500.0, testWheelRpm - 100.0);
        if (gamepad1.a && !prevA) shooterOn = !shooterOn;
        prevUp = gamepad1.dpad_up;
        prevDown = gamepad1.dpad_down;
        prevA = gamepad1.a;

        if (shooterOn) shooter.setRpm(testWheelRpm);
        else shooter.stop();

        if (gamepad1.dpad_left) turret.setOpenLoop(0.25);
        else if (gamepad1.dpad_right) turret.setOpenLoop(-0.25);
        else turret.setOpenLoop(0.0);

        if (gamepad1.right_trigger > 0.1) intake.setPower(gamepad1.right_trigger, voltage);
        else if (gamepad1.left_trigger > 0.1) intake.setPower(-gamepad1.left_trigger, voltage);
        else intake.stop();

        follower.update();
        turret.update(voltage);
        shooter.update(voltage);

        telemetry.addData("Battery", "%.2f V", voltage);
        telemetry.addData("Shooter (A)", shooterOn ? "ON (" + (int) testWheelRpm + " wheel / " + (int) (testWheelRpm * 2) + " motor RPM)" : "OFF");
        telemetry.addData("Left RPM (Wheel / Motor)", "%.0f / %.0f", shooter.leftWheelRpm(), shooter.leftMotorRpm());
        telemetry.addData("Right RPM (Wheel / Motor)", "%.0f / %.0f", shooter.rightWheelRpm(), shooter.rightMotorRpm());
        telemetry.addData("Ready", shooter.ready());
        telemetry.addData("Turret Enc", turret.hasEncoder() ? String.format("%.1f deg", Math.toDegrees(turret.angleRad())) : "UNMOUNTED (Dpad L/R)");
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.stop();
        intake.stop();
        turret.setOpenLoop(0.0);
    }
}