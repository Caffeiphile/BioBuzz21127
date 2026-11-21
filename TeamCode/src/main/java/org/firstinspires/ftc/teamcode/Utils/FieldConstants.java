package org.firstinspires.ftc.teamcode.Utils;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;

/** Red-side field coordinates and autonomous waypoints (0-144" Pedro frame). */
public final class FieldConstants {
    public static final Pose RED_START = new Pose(60, 8, Math.PI / 2.0);

    public static final Vector2D HIVE_1 = Vector2D.cartesian(60.0, 52.0);
    public static final Vector2D HIVE_2 = Vector2D.cartesian(60.0, 92.0);

    public static final Pose SHOOT_1 = new Pose(60, 28.0, Math.PI / 2.0);
    public static final Pose PICKUP_1 = new Pose(14.0, 12.0, Math.PI);

    public static final Pose SHOOT_2 = new Pose(36.0, 90.0, 0.0);
    public static final Pose PICKUP_2 = new Pose(14.0, 108.0, Math.PI);

    public static final Pose PARK = new Pose(132.0, 36.0, 0.0);

    private FieldConstants() {}
}