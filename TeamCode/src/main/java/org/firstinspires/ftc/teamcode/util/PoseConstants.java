package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

@Config
public class PoseConstants {

    public static class Start {
        public static Pose redBasket = new Pose(8.5, 111.5, Math.toRadians(-90));
        public static Pose redObsv = new Pose(8.5, 63.5, Math.toRadians(90));
        public static Pose blueBasket = new Pose(111.5, 7.5, Math.toRadians(180));
        public static Pose blueObsv = new Pose(-39.5 + 72, -63.5 + 72, Math.toRadians(180));
    }

    public static class Score {
        public static Pose redBasket = new Pose(-54 + 72, -54 + 72, Math.toRadians(45));
        public static Pose redBasketWall = new Pose(-50 + 72, -63.5 + 72, Math.toRadians(0));
        public static Pose redBasketAngle = new Pose(15, 123, Math.toRadians(-45));

        public static Pose redChamber = new Pose(37, 73, Math.toRadians(90));
        // 14,

        public static Pose blueBasket = new Pose(54 + 72, 54 + 72, Math.toRadians(225));
        public static Pose blueBasketAngle = new Pose(56 + 72, 58 + 72, Math.toRadians(225));
    }

}
