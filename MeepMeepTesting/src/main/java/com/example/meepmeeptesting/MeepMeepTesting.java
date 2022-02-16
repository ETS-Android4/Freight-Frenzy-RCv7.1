package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);



        // Declare out second bot
        RoadRunnerBotEntity mySecondBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(81.7388454100392 * 0.95, 81.7388454100392 * 0.95, 7, 7, 10)
                .setStartPose(new Pose2d(11.3, -60, Math.toRadians(90)))
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(11.3, -60, Math.toRadians(90)))
                                .lineTo(new Vector2d(3 + 11.3, 24 + -60))
                                .lineToLinearHeading(new Pose2d(15 + 11.3, 24 + -60, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(7.25 + 11.3, 72 + -60, Math.abs(300)))
                                .lineToLinearHeading(new Pose2d(-12 + 11.3, 0 + -60, Math.toRadians(360)))
                                .forward(24)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)

                // Add both of our declared bot entities
                .addEntity(mySecondBot)
                .start();
    }
}