package org.firstinspires.ftc.teamcode.comp;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.skeletonarmy.marrow.BulkCaching;
import com.skeletonarmy.marrow.TimerEx;
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.prompts.Prompter;
import com.skeletonarmy.marrow.settings.Settings;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.FieldConstants;
import org.firstinspires.ftc.teamcode.Utils.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@BulkCaching
@Autonomous(name = "BIOBUZZ Auto", group = "Competition")
public class AutoMain extends OpMode {
    private enum AutoMode { ONE_TIP_PARK, TWO_TIP_PARK }

    private Follower follower;
    private Turret turret;
    private Shooter shooter;
    private Intake intake;
    private VoltageSensor voltageSensor;

    private Prompter prompter;
    private final TimerEx matchTimer = new TimerEx(30.0);

    private AllianceColor alliance = AllianceColor.RED;
    private AutoMode mode = AutoMode.TWO_TIP_PARK;
    private Vector2D activeHive = FieldConstants.HIVE_1;
    private Pose parkPose = FieldConstants.PARK;

    private Path toShoot1, toPickup1, toPickup2, toShoot2;
    private boolean bailedToPark = false;

    @Override
    public void init() {
        Scheduler.reset();
        alliance = Settings.get("alliance", AllianceColor.RED);

        follower = Constants.create(hardwareMap);
        turret = new Turret(hardwareMap, follower::pose, follower::velocity);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        Pose start = OpModeStorage.mirror(FieldConstants.RED_START, alliance);
        Pose shoot1 = OpModeStorage.mirror(FieldConstants.SHOOT_1, alliance);
        Pose pickup1 = OpModeStorage.mirror(FieldConstants.PICKUP_1, alliance);
        Pose pickup2 = OpModeStorage.mirror(FieldConstants.PICKUP_2, alliance);
        Pose shoot2 = OpModeStorage.mirror(FieldConstants.SHOOT_2, alliance);
        parkPose = OpModeStorage.mirror(FieldConstants.PARK, alliance);
        activeHive = OpModeStorage.mirror(FieldConstants.HIVE_1, alliance);

        follower.setPose(start);
        follower.update();

        toShoot1 = line(start, shoot1).linear(start.heading(), shoot1.heading());
        toPickup1 = line(shoot1, pickup1).linear(shoot1.heading(), pickup1.heading());
        toPickup2 = line(pickup1, pickup2).linear(pickup1.heading(), pickup2.heading());
        toShoot2 = line(pickup2, shoot2).linear(pickup2.heading(), shoot2.heading());

        prompter = new Prompter(this);
        prompter.prompt("mode", new OptionPrompt<>("Select Mode", AutoMode.TWO_TIP_PARK, AutoMode.ONE_TIP_PARK))
                .label("Auto Routine")
                .showSummary()
                .onComplete(() -> {
                    mode = prompter.getOrDefault("mode", AutoMode.TWO_TIP_PARK);
                    telemetry.addData("Alliance", alliance);
                    telemetry.addData("Mode", mode);
                    telemetry.addData("Turret", "Aligned 0.0 deg (6:1 Encoder)");
                    telemetry.update();
                });
    }

    @Override
    public void init_loop() {
        prompter.run();
        mode = prompter.getOrDefault("mode", AutoMode.TWO_TIP_PARK);
    }

    @Override
    public void start() {
        matchTimer.restart();
        Scheduler.reset();
        mode = prompter.getOrDefault("mode", AutoMode.TWO_TIP_PARK);
        Scheduler.schedule(turret.track(() -> activeHive, () -> false));

        Command tip1 = sequential(
                parallel(
                        follow(follower, toShoot1),
                        shooter.spinUp(intake, () -> shooter.rpmForDist(turret.dist()))
                ),
                shooter.shoot(intake, turret, () -> shooter.rpmForDist(turret.dist()), 0.85, true),
                parallel(
                        follow(follower, toPickup1),
                        intake.run(1.0)
                )
        );

        if (mode == AutoMode.ONE_TIP_PARK) {
            Scheduler.schedule(sequential(
                    tip1,
                    instant(this::driveToPark)
            ));
        } else {
            Scheduler.schedule(sequential(
                    tip1,
                    instant(() -> activeHive = OpModeStorage.mirror(FieldConstants.HIVE_2, alliance)),
                    parallel(
                            follow(follower, toPickup2),
                            intake.run(1.0)
                    ),
                    parallel(
                            follow(follower, toShoot2),
                            shooter.spinUp(intake, () -> shooter.rpmForDist(turret.dist()))
                    ),
                    shooter.shoot(intake, turret, () -> shooter.rpmForDist(turret.dist()), 0.85, true),
                    instant(this::driveToPark)
            ));
        }
    }

    @Override
    public void loop() {
        double voltage = voltageSensor.getVoltage();

        if (!bailedToPark) {
            double distToPark = Math.hypot(parkPose.x() - follower.pose().x(), parkPose.y() - follower.pose().y());
            double neededSec = (distToPark / 28.0) + 1.2;
            if (matchTimer.getRemaining() <= neededSec) {
                bailedToPark = true;
                Scheduler.reset();
                shooter.stop();
                intake.stop();
                driveToPark();
            }
        }

        follower.update();
        Scheduler.execute();
        turret.update(voltage);
        shooter.update(voltage);

        telemetry.addData("Pose", follower.pose());
        telemetry.addData("Turret Deg", "%.1f", Math.toDegrees(turret.angleRad()));
        telemetry.addData("Time Left", String.format("%.1fs", matchTimer.getRemaining()));
        telemetry.update();
    }

    @Override
    public void stop() {
        Scheduler.reset();

        OpModeStorage.alliance = alliance;
        OpModeStorage.autoEndPose = follower.pose();
        OpModeStorage.autoEndTurretRad = turret.angleRad();

        shooter.shutdown();
        intake.stop();
        turret.setOpenLoop(0.0);
    }

    private void driveToPark() {
        Pose cur = follower.pose();
        if (Math.hypot(parkPose.x() - cur.x(), parkPose.y() - cur.y()) > 3.0) {
            follower.follow(line(cur, parkPose).linear(cur.heading(), parkPose.heading()));
        }
    }
}