package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(3000, 3000, 1000, 1000, 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-12, 61, Math.toRadians(90)))
                .strafeTo(new Vector2d(-6.8,34))//pune piesa pe submersibil
                .waitSeconds(0.1)
                .splineToSplineHeading(new Pose2d(-35, 40.3, Math.toRadians(-90)), Math.toRadians(180))
                .strafeTo(new Vector2d(-37,11))
                .strafeTo(new Vector2d(-48,11))
                .setTangent(Math.toRadians(90))
                .lineToY(50)
                .lineToY(11)
                .strafeTo(new Vector2d(-57,11))
                .setTangent(Math.toRadians(90))
                .lineToY(50)
                .lineToY(11)
                .strafeTo(new Vector2d(-65,11))
                .setTangent(Math.toRadians(90))
                .lineToY(50)
                .splineToSplineHeading(new Pose2d(-47.1,59, Math.toRadians(-90)), Math.toRadians(130))
                .waitSeconds(0.1)//se duce la pozitia hp
                .splineToSplineHeading(new Pose2d(-6.8,34, Math.toRadians(89.9)), Math.toRadians(350))
                .waitSeconds(0.1)//se duce la pozitia hp
                .splineToSplineHeading(new Pose2d(-47.1,59, Math.toRadians(-90)), Math.toRadians(130))
                .waitSeconds(0.1)//se duce la pozitia hp
                .splineToSplineHeading(new Pose2d(-6.8,34, Math.toRadians(89.9)), Math.toRadians(350))
                .waitSeconds(0.1)//se duce la pozitia hp
                .splineToSplineHeading(new Pose2d(-47.1,59, Math.toRadians(-90)), Math.toRadians(130))
                .waitSeconds(0.1)//se duce la pozitia hp
                .splineToSplineHeading(new Pose2d(-6.8,34, Math.toRadians(89.9)), Math.toRadians(350))
                .waitSeconds(0.1)//se duce la pozitia hp
                .splineToSplineHeading(new Pose2d(-47.1,59, Math.toRadians(-90)), Math.toRadians(130))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}