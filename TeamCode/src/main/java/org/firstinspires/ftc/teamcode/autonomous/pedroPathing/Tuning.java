package org.firstinspires.ftc.teamcode.autonomous.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.OTOSLocalizer;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.procedures.OTOSTuner;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.procedures.Tests;

/**
 * Pedro Pathing 3.0 AutoTune procedures.
 * <p>
 * Deploy this code, connect to the robot's Wi-Fi and open http://192.168.43.1:10158 in a browser to run them
 * (see https://pedropathing.com/docs/pathing/tuning). Paste the configs that AutoTune generates into
 * {@link Constants}.
 * <p>
 * The tuners below assume a mecanum drivetrain with a goBILDA Pinpoint. If your robot differs, swap in the
 * matching tuner and localizer/drivetrain from the procedures package (e.g. {@code OTOSTuner},
 * {@code ThreeWheelTuner}, {@code TwoWheelTuner}, {@code OctoQuadTuner}).
 */
public class Tuning {
    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    @Tuner
    public static Procedure otosTuner() {
        return new OTOSTuner();
    }

    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner(
                hardwareMap -> new OTOSLocalizer(hardwareMap, Constants.otosConfig),
                hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig)
        );
    }

    @Tuner
    public static Procedure tests() {
        return new Tests(
                hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig),
                hardwareMap -> new OTOSLocalizer(hardwareMap, Constants.otosConfig),
                () -> new Foresight(Constants.foresightConfig)
        );
    }
}
