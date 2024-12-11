package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-12, 61, Math.toRadians(90)))
                .strafeTo(new Vector2d(-6.8,34))//pune piesa pe submersibil
                .waitSeconds(1)
                .splineTo(new Vector2d(-35, 40.3),Math.toRadians(90))
                .strafeTo(new Vector2d(-37,11))
                .strafeTo(new Vector2d(-45,11))
                .turn(Math.toRadians(180))
                .setTangent(Math.toRadians(90))
                .lineToY(50)
                .lineToY(11)
                .strafeTo(new Vector2d(-57,11))
                .setTangent(Math.toRadians(90))
                .lineToY(50)
                .lineToY(11)
                .strafeTo(new Vector2d(-62,11))
                .setTangent(Math.toRadians(90))
                .lineToY(50)
                .strafeTo(new Vector2d(-47.1,59))
                .waitSeconds(1)//se duce la pozitia hp
                .strafeTo (new Vector2d(-6.8,34))
                .strafeTo(new Vector2d(-47.1,59))
                .waitSeconds(1)//se duce la pozitia hp
                .strafeTo (new Vector2d(-6.8,34))
                .strafeTo(new Vector2d(-47.1,59))
                .waitSeconds(1)//se duce la pozitia hp
                .strafeTo (new Vector2d(-6.8,34))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}