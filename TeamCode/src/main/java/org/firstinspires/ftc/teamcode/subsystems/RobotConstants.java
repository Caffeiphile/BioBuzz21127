package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

public class RobotConstants {
    public static final double NOMINAL_VOLTAGE = 12.0;

    public static class Intake {
        public static final String MOTOR = "intake";
        public static final Direction DIR = Direction.FORWARD;
        public static final double BACKOFF_PWR = -0.35;
        public static final double BACKOFF_SEC = 0.12;
    }

    public static class Outtake {
        public static final String LEFT_MOTOR = "LFW";
        public static final String RIGHT_MOTOR = "RFW";
        public static final Direction LEFT_DIR = Direction.REVERSE;
        public static final Direction RIGHT_DIR = Direction.FORWARD;

        // Bare motor = 28 ticks/rev; 1:2 overdrive (1 motor turn = 2 wheel turns)
        public static final double TICKS_PER_REV = 28.0;
        public static final double GEAR_RATIO = 2.00;
        public static final double MAX_WHEEL_RPM = 9000.0;

        // Scaled 4x from 0.50 -> 2.00 ratio (re-run MechanismTuner to refine under new load)
        public static final double KF = 0.000095;
        public static final double KP = 0.000367;
        public static final double KD = 0.000027;

        public static final double BANG_BANG_RPM = 450.0;
        public static final double BANG_BANG_PWR = 1.00;
        public static final double DEADBAND_RPM = 85.0;
        public static final double BACKOFF_PWR = -0.25;
        public static final double IDLE_HOLD_PWR = -0.12;
    }

    public static class Turret {
        public static final String MOTOR = "turret";
        public static final Direction DIR = Direction.FORWARD;

        public static final double TICKS_PER_REV = 28.0;
        public static final double GEAR_RATIO = 6.00;

        public static final double KS = 0.0300;
        public static final double KV = 0.01720;
        public static final double KA = 0.00133;
        public static final double KP = 0.2608;
        public static final double KD = 0.0201;

        public static final double MAX_VEL = 5.50;
        public static final double MAX_ACCEL = 18.00;
        public static final double TOLERANCE_RAD = Math.toRadians(5);
        public static final double SHOT_SPEED_IN_PER_SEC = 240.0; // Faster exit speed at ~6000 wheel RPM
    }
}