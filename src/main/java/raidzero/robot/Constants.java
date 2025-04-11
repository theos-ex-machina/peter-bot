package raidzero.robot;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

public class Constants {
    public static class Arm {
        public static class ProximalJoint {
            public static final int MOTOR_ID = 10;
            public static final double SENSOR_TO_MECHANISM_RATIO = 1.0;
            public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;

            public static final double STATOR_CURRENT_LIMIT = 40;
            public static final double SUPPLY_CURRENT_LIMIT = 50;

            public static final double FORWARD_SOFT_LIMIT = 0.45;
            public static final double REVERSE_SOFT_LIMIT = 0.05;

            public static final double P = 1.0;
            public static final double I = 0.0;
            public static final double D = 0.0;

            public static final double S = 0.0;
            public static final double G = 0.0;
            public static final double V = 0.0;
            public static final double A = 0.0;
            public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Arm_Cosine;

            public static final double CRUISE_VELOCITY = 1.0;
            public static final double ACCELERATION = 1.0;

            public static final double LENGTH = 1.0;
        }

        public static class DistalJoint {
            public static final int MOTOR_ID = 11;
            public static final double SENSOR_TO_MECHANISM_RATIO = 1.0;
            public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;

            public static final double STATOR_CURRENT_LIMIT = 40;
            public static final double SUPPLY_CURRENT_LIMIT = 50;

            public static final double P = 1.0;
            public static final double I = 0.0;
            public static final double D = 0.0;

            public static final double S = 0.0;
            public static final double G = 0.0;
            public static final double V = 0.0;
            public static final double A = 0.0;
            public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Arm_Cosine;

            public static final double CRUISE_VELOCITY = 1.0;
            public static final double ACCELERATION = 1.0;

            public static final double LENGTH = 1.0;
        }
    }
}