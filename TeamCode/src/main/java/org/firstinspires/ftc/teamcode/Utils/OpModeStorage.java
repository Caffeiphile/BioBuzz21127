package org.firstinspires.ftc.teamcode.Utils;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/** Persists robot pose, turret angle, and alliance selection between Autonomous and TeleOp. */
public final class OpModeStorage {
    public static Pose autoEndPose = new Pose(0, 0, 0);
    public static double autoEndTurretRad = 0.0;
    public static AllianceColor alliance = AllianceColor.RED;

    private OpModeStorage() {}

    public static Pose mirror(Pose redPose, AllianceColor color) {
        if (color == AllianceColor.RED) return redPose;
        return new Pose(
                144.0 - redPose.x(),
                144.0 - redPose.y(),
                AngleUnit.normalizeRadians(redPose.heading() + Math.PI)
        );
    }

    public static Vector2D mirror(Vector2D redPt, AllianceColor color) {
        if (color == AllianceColor.RED) return redPt;
        return Vector2D.cartesian(144.0 - redPt.x(), 144.0 - redPt.y());
    }
}