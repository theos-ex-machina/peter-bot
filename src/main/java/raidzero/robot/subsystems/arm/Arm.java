package raidzero.robot.subsystems.arm;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidzero.robot.Constants.Arm.DistalJoint;
import raidzero.robot.Constants.Arm.ProximalJoint;

public class Arm extends SubsystemBase {
    private static Arm system;
    private ArmIO io;

    private Arm(ArmIO io) {
        this.io = io;
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

            io.moveWithRotations(proximalSetpoint, distalSetpoint);
            io.moveWrist(setpoint[2] - theta2);
        });
    }

    public Command home() {
        return run(() -> {
            io.moveWithRotations(0.25, 0.75);

            if (Intake.system().hasCoral().getAsBoolean()) {
                io.moveWrist(0);
            } else {
                io.moveWrist(0.25);
            }
        });
    }

    public static Arm system() {
        if (system == null) {
            if(Utils.isSimulation())
                system = new Arm(new ArmIO.Sim());
            else 
                system = new Arm(new ArmIO.Real());
        }
        return system;
    }
}