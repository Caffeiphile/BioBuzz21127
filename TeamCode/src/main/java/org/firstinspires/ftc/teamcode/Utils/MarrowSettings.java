package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.settings.SettingsOpMode;

@TeleOp
public class MarrowSettings extends SettingsOpMode {
    @Override
    public void defineSettings() {
        add("alliance", "Alliance Color", new OptionPrompt<>("Which alliance",AllianceColor.RED,AllianceColor.BLUE));
    }
}