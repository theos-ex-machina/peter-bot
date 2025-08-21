package raidzero.robot.subsystems.arm;

import static raidzero.robot.subsystems.arm.ArmConstants.Intake.*;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import raidzero.lib.R0Subsystem;

public class Intake extends R0Subsystem<IntakeIO> {
    private static Intake system;

    /**
     * Constructs a {@link CoralIntake} subsystem instance
     *
     * @param io the hardware IO to use (simulation/real)
     */
    private Intake(IntakeIO io) {
        super(io);
    }

    /**
     * Intakes a coral
     *
     * @return A {@link Command} that intakes a coral
     */
    public Command intakeCoral() {
        return run(() -> io.setSpeed(INTAKE_SPEED_PERCENT)).until(io.bottomLaserWithinThreshold());
    }

    /**
     * Extakes a coral
     *
     * @return A {@link Command} that extakes a coral
     */
    public Command extakeCoral() {
        return run(() -> io.setSpeed(EXTAKE_SPEED_PERCENT));
    }

    /**
     * Intakes an lgae

     * @return A {@link Command} that intakes an algae
     */
    public Command intakeAlgae() {
        return run(() -> io.setSpeed(INTAKE_SPEED_PERCENT));
    }

    /**
     * Extakes an algae
     *
     * @return A {@link Command} that extakes an algae
     */
    public Command extakeAlgae() {
        return run(() -> io.setSpeed(ALGAE_EJECT_SPEED_PERCENT));
    }

    /**
     * Holds the algae by applying a small amount of voltage
     *
     * @return A {@link Command} that holds the algae
     */
    public Command holdAlgae() {
        return run(() -> io.setSpeed(ALGAE_HOLD_SPEED_PERCENT));
    }

    /**
     * Stops the intake
     *
     * @return A {@link Command} that stops the intake
     */
    public Command stop() {
        return runOnce(() -> io.stop());
    }

    /**
     * Holds algae if in the intake, else not
     *
     * @return A {@link Command}
     */
    public Command idleBehavior() {
        return run(() -> {
            if (io.topLaserWithinThreshold().getAsBoolean() && !io.bottomLaserWithinThreshold().getAsBoolean()) {
                io.setSpeed(ALGAE_HOLD_SPEED_PERCENT);
            } else {
                io.stop();
            }
        });
    }

    /**
     * Trigger to detect if algae is in the intake
     *
     * @return a {@link Trigger}
     */
    public Trigger hasAlgae() {
        return io.topLaserWithinThreshold().and(io.bottomLaserWithinThreshold().negate());
    }

    /**
     * Trigger to detect if coral is in the intake
     *
     * @return a {@link Trigger}
     */
    public Trigger hasCoral() {
        return io.topLaserWithinThreshold().and(io.bottomLaserWithinThreshold());
    }

    /**
     * Runs the roller at the specified speed
     *
     * @param speed The speed to run the roller at [-1, 1]
     * @return A {@link Command} to run the roller at the specified speed
     */
    public Command run(double speed) {
        return run(() -> io.setSpeed(speed));
    }

    /**
     * Gets the {@link CoralIntake} subsystem instance
     *
     * @return The {@link CoralIntake} subsystem instance
     */
    public static Intake system() {
        if (system == null) {
            if (Utils.isSimulation())
                system = new Intake(new IntakeIO.Sim());
            else
                system = new Intake(new IntakeIO.Real());
        }

        return system;
    }
}
