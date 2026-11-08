package org.firstinspires.ftc.teamcode.debug;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.skeletonarmy.marrow.BulkCaching;

import org.firstinspires.ftc.teamcode.subsystems.shooterPlaceHolderNameV1;

@BulkCaching
@TeleOp
public class FullRobotTest extends OpMode {
    shooterPlaceHolderNameV1 shooter;

    @Override
    public void init() {
        shooter = new shooterPlaceHolderNameV1("leftShooter", "rightShooter", hardwareMap);
    }

    @Override
    public void loop() {
        shooter.setPower(1);
    }
}
