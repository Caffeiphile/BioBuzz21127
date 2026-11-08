package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.hardware.lynx.LynxModule;
import com.skeletonarmy.marrow.BulkCachingSettings;
import com.sun.tools.javac.util.Context;

import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;

public final class AutoBulkConfig {
    @OnCreateEventLoop
    public static void configure(Context context, FtcEventLoop ftcEventLoop) {
        BulkCachingSettings.defaultMode = LynxModule.BulkCachingMode.AUTO;
    }
}