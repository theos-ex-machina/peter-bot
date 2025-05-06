package raidzero.robot.subsystems.drivetrain;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import raidzero.robot.subsystems.drivetrain.TunerConstants.TunerSwerveDrivetrain;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;
import static raidzero.robot.Constants.Simulation.*;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class Swerve extends TunerSwerveDrivetrain implements Subsystem {
    private Notifier simNotifier = null;

    // Blue alliance sees forward as 0 degrees (toward red alliance wall)
    private static final Rotation2d BLUE_ALLLIANCE_PERSPECTIVE_ROTATION = Rotation2d.kZero;
    // Red alliance sees forward as 180 degrees (toward blue alliance wall)
    private static final Rotation2d RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.k180deg;
    private boolean hasAppliedOperatorPerspective = false;

    // Swerve requests to apply during SysId characterization
    private final SwerveRequest.ApplyRobotSpeeds pathplannerSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    private StructArrayPublisher<SwerveModuleState> modulePublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
    private StructPublisher<Pose2d> botpose = NetworkTableInstance.getDefault().getStructTopic("botPoseNT", Pose2d.struct).publish();

    private final Field2d field = new Field2d();

    private boolean waypointsTransformed = false;

    private static Swerve system;

    /** ... */
    public Swerve(SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency, SwerveModuleConstants<?, ?, ?>... modules) {
        super(
            drivetrainConstants, odometryUpdateFrequency,
            // modules
            MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules)
        );
        if (Utils.isSimulation()) {
            startSimThread();
        }
        SmartDashboard.putData("Field", field);

        configureAutoBuilder();
        initializeOtf();
    }

    /** ... */
    public Swerve(SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency, Matrix<N3, N1> odometryStandardDeviation, Matrix<N3, N1> visionStandardDeviation, SwerveModuleConstants<?, ?, ?>... modules) {
        super(
            drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
            // modules
            MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules)
        );
        if (Utils.isSimulation()) {
            startSimThread();
        }
        SmartDashboard.putData("Field", field);

        configureAutoBuilder();
        initializeOtf();
    }

    /**
     * Returns a command that applies the specified control request to this swerve
     * drivetrain.
     *
     * @param requestSupplier Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Uses PathPlanner's {@link AutoBuilder#pathfindToPose} to move to the desired pose
     *
     * @param pose The desired pose
     * @return A {@link DeferredCommand} that moves the robot to the desired pose
     */
    public Command goToPose(Pose2d pose) {
        return defer(
            () -> AutoBuilder.pathfindToPose(
                pose,
                new PathConstraints(
                    3.5,
                    4.0,
                    Units.degreesToRadians(540),
                    Units.degreesToRadians(720)
                )
            ).finallyDo((interrupted) -> this.stop())
        );
    }

    /**
     * Returns a {@link BooleanSupplier} that checks if the robot is not in a climb zone
     *
     * @return A {@link BooleanSupplier} that checks if the robot is not in a climb zone
     */
    public BooleanSupplier isNotInClimbZone() {
        return () -> {
            Translation2d currTranslation = this.getState().Pose.getTranslation();

            return currTranslation.getX() > 7.525 && currTranslation.getX() < 10.025;
        };
    }

    /**
     * Stops the swerve
     *
     * @return A {@link Command} to stop the swerve
     */
    public Command stop() {
        return runOnce(
            () -> this.setControl(
                new SwerveRequest.RobotCentric().withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0.0)
            )
        );
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should
         * apply
         * it regardless of DS state.
         * This allows us to correct the perspective in case the robot code
         * restarts
         * mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled.
         * This ensures driving behavior doesn't change until an explicit
         * disable event
         * occurs during testing.
         */
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red ?
                        RED_ALLIANCE_PERSPECTIVE_ROTATION :
                        BLUE_ALLLIANCE_PERSPECTIVE_ROTATION
                );
                this.hasAppliedOperatorPerspective = true;
            });
        }

        modulePublisher.set(this.getState().ModuleStates);
        botpose.set(this.getState().Pose);
        field.setRobotPose(this.getState().Pose);
    }

    /**
     * Starts the simulation thread.
     */
    // private void startSimThread() {
    // this.lastSimTime = Utils.getCurrentTimeSeconds();

    // /*
    // * Run simulation at a faster rate so PID gains behave more reasonably
    // */
    // this.simNotifier = new Notifier(() -> {
    // final double currentTime = Utils.getCurrentTimeSeconds();
    // double deltaTime = currentTime - lastSimTime;
    // lastSimTime = currentTime;

    // /* use the measured time delta, get battery voltage from WPILib */
    // updateSimState(deltaTime, RobotController.getBatteryVoltage());
    // });

    // this.simNotifier.startPeriodic(SIM_LOOP_PERIOD);
    // }

    private MapleSimSwerveDrivetrain mapleSimSwerveDrivetrain = null;

    @SuppressWarnings("unchecked")
    private void startSimThread() {
        mapleSimSwerveDrivetrain = new MapleSimSwerveDrivetrain(
            Seconds.of(SIM_LOOP_PERIOD_S),
            Pounds.of(BOT_WEIGHT_LBS),
            Inches.of(BOT_LENGTH_BUMPERS), // bumper length
            Inches.of(BOT_WIDTH_BUMPERS), // bumper width
            DCMotor.getKrakenX60(1), // drive motor type
            DCMotor.getFalcon500(1), // steer motor type
            WHEEL_COF, // wheel COF
            getModuleLocations(),
            getPigeon2(),
            getModules(),
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight
        );

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        simNotifier = new Notifier(mapleSimSwerveDrivetrain::update);
        simNotifier.startPeriodic(SIM_LOOP_PERIOD_S);
    }

    /**
     * <ul>
     * <li>Initializes the on-the-fly (OTF) waypoints for the robot
     * <li>Transforms the waypoints for the red alliance if needed
     * </ul>
     */
    public void initializeOtf() {
        if ((DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) && !waypointsTransformed) {
            waypointsTransformed = true;
        }
    }

    /**
     * Configures the AutoBuilder for the Swerve subsystem
     */
    private void configureAutoBuilder() {
        try {
            AutoBuilder.configure(
                () -> this.getState().Pose,
                this::resetPose,
                () -> this.getState().Speeds,
                (speeds, feedforwards) -> setControl(
                    pathplannerSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    new PIDConstants(3.3, 0, 0),
                    new PIDConstants(3, 0, 0)
                ),
                RobotConfig.fromGUISettings(),
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this
            );
        } catch (Exception e) {
            DriverStation.reportError("Failed to load PathPlanner config and configure Autobuilder", e.getStackTrace());
        }
    }

    @Override
    public void resetPose(Pose2d pose) {
        if (this.mapleSimSwerveDrivetrain != null) {
            mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(pose);
            Timer.delay(0.05); // Wait for simulation to update
        }
        super.resetPose(pose);
    }

    /**
     * Gets the {@link Swerve} subsystem instance
     *
     * @return The {@link Swerve} subsystem instance
     */
    public static Swerve system() {
        if (system == null) {
            system = new Swerve(
                TunerConstants.DrivetrainConstants,
                100,
                TunerConstants.FrontLeft,
                TunerConstants.FrontRight,
                TunerConstants.BackLeft,
                TunerConstants.BackRight
            );
        }

        return system;
    }
}