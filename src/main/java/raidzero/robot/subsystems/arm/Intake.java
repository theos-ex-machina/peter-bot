package raidzero.robot.subsystems.arm;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidzero.lib.LazyCan;
import static raidzero.robot.Constants.Arm.Intake.*;

public class Intake extends SubsystemBase {
    private TalonFXS roller, follower;
    private LazyCan bottomLaser, topLaser;

    private static Intake system;

    /**
     * Constructs a {@link CoralIntake} subsystem instance
     */
    private Intake() {
        roller = new TalonFXS(LEADER_ID);
        roller.getConfigurator().apply(rollerConfiguration());

        follower = new TalonFXS(FOLLOWER_ID);
        follower.getConfigurator().apply(followerConfiguration());
        follower.setControl(new Follower(LEADER_ID, false));

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
    public Command intake() {
        return run(() -> roller.set(INTAKE_SPEED)).until(() -> bottomLaser.withinThreshold());
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
        return runOnce(() -> roller.stopMotor());
    }

    /**
     * Extakes a coral
     *
     * @return A {@link Command} that extakes a coral
     */
    public Command extake() {
        return run(() -> roller.set(EXTAKE_SPEED));
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
     * Gets the {@link TalonFXSConfiguration} for the roller motor
     *
     * @return The {@link TalonFXSConfiguration} for the roller motor
     */
    private TalonFXSConfiguration rollerConfiguration() {
        TalonFXSConfiguration configuration = new TalonFXSConfiguration();

        configuration.Commutation.MotorArrangement = MOTOR_ARRANGEMENT;
        configuration.MotorOutput.Inverted = INVERTED_VALUE;

        configuration.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
        configuration.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
        configuration.CurrentLimits.SupplyCurrentLowerTime = SUPPLY_CURRENT_LOWER_TIME;

        configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        return configuration;
    }

    /**
     * Gets the {@link TalonFXSConfiguration} for the follower motor
     *
     * @return The {@link TalonFXSConfiguration} for the follower motor
     */
    private TalonFXSConfiguration followerConfiguration() {
        TalonFXSConfiguration configuration = new TalonFXSConfiguration();

        configuration.Commutation.MotorArrangement = MOTOR_ARRANGEMENT;
        configuration.MotorOutput.Inverted = INVERTED_VALUE;

        configuration.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
        configuration.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
        configuration.CurrentLimits.SupplyCurrentLowerTime = SUPPLY_CURRENT_LOWER_TIME;

        configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        return configuration;
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
