package raidzero.robot.subsystems.arm;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidzero.lib.LazyFXS;
import raidzero.robot.Constants.Arm.DistalJoint;
import raidzero.robot.Constants.Arm.ProximalJoint;
import raidzero.robot.Constants.Arm.Wrist;

public class Arm extends SubsystemBase {
    private LazyFXS proximalJoint, distalJoint;
    private LazyFXS wrist;

    private static Arm system;

    private Arm() {
        proximalJoint = new LazyFXS(
            ProximalJoint.MOTOR_ID,
            ProximalJoint.MOTOR_ARRANGEMENT,
            ProximalJoint.SENSOR_TO_MECHANISM_RATIO,
            ProximalJoint.INVERTED,
            ProximalJoint.STATOR_CURRENT_LIMIT,
            ProximalJoint.SUPPLY_CURRENT_LIMIT
        ).withSoftLimits(
            true, ProximalJoint.FORWARD_SOFT_LIMIT,
            true, ProximalJoint.REVERSE_SOFT_LIMIT
        ).withMotionMagicConfiguration(
            ProximalJoint.P, ProximalJoint.I, ProximalJoint.D,
            ProximalJoint.S, ProximalJoint.G, ProximalJoint.V, ProximalJoint.A,
            ProximalJoint.GRAVITY_TYPE,
            ProximalJoint.CRUISE_VELOCITY, ProximalJoint.ACCELERATION
        ).build();

        distalJoint = new LazyFXS(
            DistalJoint.MOTOR_ID,
            DistalJoint.MOTOR_ARRANGEMENT,
            DistalJoint.SENSOR_TO_MECHANISM_RATIO,
            DistalJoint.INVERTED,
            DistalJoint.STATOR_CURRENT_LIMIT,
            DistalJoint.SUPPLY_CURRENT_LIMIT
        ).withMotionMagicConfiguration(
            DistalJoint.P, DistalJoint.I, DistalJoint.D,
            DistalJoint.S, DistalJoint.G, DistalJoint.V, DistalJoint.A,
            DistalJoint.GRAVITY_TYPE,
            DistalJoint.CRUISE_VELOCITY, DistalJoint.ACCELERATION
        ).build();

        wrist = new LazyFXS(
            Wrist.MOTOR_ID, Wrist.MOTOR_ARRANGEMENT, Wrist.SENSOR_TO_MECHANISM_RATIO, Wrist.INVERTED_VALUE, Wrist.STATOR_CURRENT_LIMIT,
            Wrist.SUPPLY_CURRENT_LIMIT
        ).withMotionMagicConfiguration(
            Wrist.P, Wrist.I, Wrist.D,
            Wrist.S, Wrist.G, Wrist.V, Wrist.A,
            Wrist.GRAVITY_TYPE,
            Wrist.CRUISE_VELOCITY, Wrist.ACCELERATION
        ).build();
    }

    /**
     * Moves the arm to the given setpoint with the given wrist angle
     * 
     * @param setpoint first 2 numbers are x and y position, next number is the wrist angle
     * @return a {@link Command}
     */
    public Command moveTo(double[] setpoint) {
        return run(() -> {
            double r = Math.sqrt(setpoint[0] * setpoint[0] + setpoint[1] * setpoint[1]); // Distance from origin to target

            double cosTheta2 = Math.max(
                -1, Math.min(
                    1, (r * r - ProximalJoint.LENGTH * ProximalJoint.LENGTH - DistalJoint.LENGTH * DistalJoint.LENGTH) /
                        (2 * ProximalJoint.LENGTH * DistalJoint.LENGTH)
                )
            );
            double theta2 = Math.acos(cosTheta2);

            double theta1 = Math.atan2(setpoint[0], setpoint[1]) - Math.atan2(
                DistalJoint.LENGTH * Math.sin(theta2),
                ProximalJoint.LENGTH + DistalJoint.LENGTH * Math.cos(theta2)
            );

            double proximalSetpoint = theta1 / 2.0 * Math.PI;
            double distalSetpoint = theta2 / 2.0 * Math.PI;

            moveWithRotations(proximalSetpoint, distalSetpoint);
            wrist.moveTo(setpoint[2]);
        });
    }

    public Command home() {
        return run(() -> {
            moveWithRotations(0.25, 0.75);

            if (Intake.system().hasCoral().getAsBoolean()) {
                wrist.moveTo(0);
            } else {
                wrist.moveTo(0.25);
            }
        });
    }

    private void moveWithRotations(double proximalSetpoint, double distalSetpoint) {
        proximalJoint.moveTo(proximalSetpoint);
        distalJoint.moveTo(distalSetpoint);
    }

    public static Arm system() {
        if (system == null) {
            system = new Arm();
        }
        return system;
    }
}