package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Right {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        int parkNum = 1;
        Pose2d vectorPark = new Pose2d(0, 0);

        switch (parkNum) {
            case 1:
                vectorPark = new Pose2d(57.5, -12, Math.toRadians(180));
                break;
            case 2:
                vectorPark = new Pose2d(35, -12, Math.toRadians(180));
                break;
            case 3:
                vectorPark = new Pose2d(12, -12, Math.toRadians(180));
                break;

        }

        Pose2d finalVectorPark = vectorPark;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(56, 56, Math.toRadians(180), Math.toRadians(180), 14.65)
                .setDimensions(16, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35, -61.8, Math.toRadians(90)))

                                // preload
                                .lineToSplineHeading(new Pose2d(35, -5, Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(35, -11.5, Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(29.5, -6.5, Math.toRadians(133)))
                                .lineToSplineHeading(new Pose2d(38.2, -11.5, Math.toRadians(0)))



                                // cycle 1
                                .lineToSplineHeading(new Pose2d(60, -11.5, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(38.2, -11.5, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(29.5, -6.5, Math.toRadians(133)))
                                .lineToSplineHeading(new Pose2d(38.2, -11.5, Math.toRadians(0)))

                                // cycle 2
                                .lineToSplineHeading(new Pose2d(60, -11.5, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(38.2, -11.5, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(29.5, -6.5, Math.toRadians(133)))
                                .lineToSplineHeading(new Pose2d(38.2, -11.5, Math.toRadians(0)))

                                // cycle 3
                                .lineToSplineHeading(new Pose2d(60, -11.5, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(38.2, -11.5, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(29.5, -6.5, Math.toRadians(133)))
                                .lineToSplineHeading(new Pose2d(38.2, -11.5, Math.toRadians(0)))

                                // cycle 4
                                .lineToSplineHeading(new Pose2d(60, -11.5, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(38.2, -11.5, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(29.5, -6.5, Math.toRadians(133)))
                                .lineToSplineHeading(new Pose2d(38.2, -11.5, Math.toRadians(0)))

                                // cycle 4
                                .lineToSplineHeading(new Pose2d(60, -11.5, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(38.2, -11.5, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(29.5, -6.5, Math.toRadians(133)))

                                // park
                                .lineToSplineHeading(finalVectorPark)



                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}