package raidzero.robot.subsystems.arm;

import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import raidzero.lib.wrappers.LazyCan;
import raidzero.lib.wrappers.motors.LazyFXS;

import static raidzero.robot.subsystems.arm.ArmConstants.Intake.*;

public class Intake extends SubsystemBase {
    private LazyFXS roller;
    private LazyCan bottomLaser, topLaser;

    private static Intake system;

    /**
     * Constructs a {@link CoralIntake} subsystem instance
     */
    private Intake() {
        roller = new LazyFXS(
            LEADER_ID, MOTOR_ARRANGEMENT,
            SENSOR_TO_MECHANISM_RATIO, INVERTED_VALUE,
            STATOR_CURRENT_LIMIT, SUPPLY_CURRENT_LIMIT
        ).withFollower(FOLLOWER_ID, FOLLOWER_INVERTED).build();

        bottomLaser = new LazyCan(BOTTOM_LASERCAN_ID).withRangingMode(RangingMode.SHORT)
            .withRegionOfInterest(14, 8, 16, 16).withTimingBudget(TimingBudget.TIMING_BUDGET_20MS)
            .withThreshold(BOTTOM_LASER_THRESHOLD_MM);

        topLaser = new LazyCan(TOP_LASERCAN_ID).withRangingMode(RangingMode.LONG)
            .withRegionOfInterest(8, 14, 16, 4).withTimingBudget(TimingBudget.TIMING_BUDGET_20MS)
            .withThreshold(TOP_LASER_THRESHOLD_MM);
    }

    /**
     * Intakes a coral
     *
     * @return A {@link Command} that intakes a coral
     */
    public Command intakeCoral() {
        return run(() -> roller.set(INTAKE_SPEED)).until(() -> bottomLaser.withinThreshold());
    }

    /**
     * Extakes a coral
     *
     * @return A {@link Command} that extakes a coral
     */
    public Command extakeCoral() {
        return run(() -> roller.set(EXTAKE_SPEED));
    }

    /**
     * Intakes an lgae

     * @return A {@link Command} that intakes an algae
     */
    public Command intakeAlgae() {
        return run(() -> roller.set(INTAKE_SPEED));
    }

    /**
     * Extakes an algae
     *
     * @return A {@link Command} that extakes an algae
     */
    public Command extakeAlgae() {
        return run(() -> roller.set(ALGAE_EJECT_SPEED));
    }

    /**
     * Holds the algae by applying a small amount of voltage
     *
     * @return A {@link Command} that holds the algae
     */
    public Command holdAlgae() {
        return run(() -> roller.set(ALGAE_HOLD_SPEED));
    }

    /**
     * Stops the intake
     *
     * @return A {@link Command} that stops the intake
     */
    public Command stop() {
        return runOnce(() -> roller.stop());
    }

    /**
     * Holds algae if in the intake, else not
     *
     * @return A {@link Command}
     */
    public Command idleBehavior() {
        return run(() -> {
            if (topLaser.withinThreshold() && !bottomLaser.withinThreshold()) {
                roller.set(ALGAE_HOLD_SPEED);
            } else {
                roller.stop();
            }
        });
    }

    /**
     * Trigger to detect if algae is in the intake
     *
     * @return a {@link Trigger}
     */
    public Trigger hasAlgae() {
        return new Trigger(() -> topLaser.withinThreshold() && !bottomLaser.withinThreshold());
    }

    /**
     * Trigger to detect if coral is in the intake
     *
     * @return a {@link Trigger}
     */
    public Trigger hasCoral() {
        return new Trigger(() -> topLaser.withinThreshold() && bottomLaser.withinThreshold());
    }

    /**
     * Runs the roller at the specified speed
     *
     * @param speed The speed to run the roller at [-1, 1]
     * @return A {@link Command} to run the roller at the specified speed
     */
    public Command run(double speed) {
        return run(() -> roller.set(speed));
    }

    /**
     * Gets the {@link CoralIntake} subsystem instance
     *
     * @return The {@link CoralIntake} subsystem instance
     */
    public static Intake system() {
        if (system == null) {
            system = new Intake();
        }

        return system;
    }
}
