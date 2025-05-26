package raidzero.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

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
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;
import java.util.function.Supplier;

import raidzero.robot.subsystems.drivetrain.SwerveConstants.Setpoints;
import raidzero.robot.subsystems.drivetrain.TunerConstants.TunerSwerveDrivetrain;
import raidzero.robot.subsystems.drivetrain.commands.GoToNearestReef;

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
    private double lastSimTime;

    private static Swerve system;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public Swerve(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);

        SmartDashboard.putData("Field", field);

        configureAutoBuilder();
        initializeOtf();

        if (Utils.isSimulation()) {
            this.startSimThread();
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public Swerve(SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);

        SmartDashboard.putData("Field", field);

        configureAutoBuilder();
        initializeOtf();

        if (Utils.isSimulation()) {
            this.startSimThread();
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve
     *                                  drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz
     *                                  on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public Swerve(SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency, Matrix<N3, N1> odometryStandardDeviation, Matrix<N3, N1> visionStandardDeviation, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);

        SmartDashboard.putData("Field", field);

        configureAutoBuilder();
        initializeOtf();

        if (Utils.isSimulation()) {
            this.startSimThread();
        }
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
     * Finds and paths to the nearest reef waypoint
     * 
     * @return a {@link Command}
     */
    public Command goToNearestReef(int level) {
        return new GoToNearestReef(level, system);
    }

    public Command goToNearestStation() {
        return defer(() -> {
            return this.goToPose(this.getState().Pose.nearest(Setpoints.STATION_WAYPOINTS));
        });
    }

    /**
     * TODO: do this method
     * @return a {@link Command}
     */
    public Command goToNearestL2Algae() {
        return null;
    }

    /**
     * TODO: do this method
     * @return a {@link Command}
     */
    public Command goToNearestL3Algae() {
        return null;
    }

    /**
     * TODO: do this method
     * @return
     */
    public Command goToProcessor() {
        return null;
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

    private void startSimThread() {
        this.lastSimTime = Utils.getCurrentTimeSeconds();

        /*
         * Run simulation at a faster rate so PID gains behave more reasonably
         */
        this.simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });

        this.simNotifier.startPeriodic(0.002);
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

            transformWaypointsForAlliance(Setpoints.LEFT_REEF_WAYPOINTS);
            transformWaypointsForAlliance(Setpoints.RIGHT_REEF_WAYPOINTS);
            transformWaypointsForAlliance(Setpoints.STATION_WAYPOINTS);
        }
    }

    /**
     * <ul>
     * <li>Transforms each waypoint in the provided list for the red alliance
     * <li><b><em>This method modifies the points within the list
     * </ul>
     *
     * @param waypoints The list of Pose2d waypoints (defined in blue origin coordinates)
     */
    private void transformWaypointsForAlliance(List<Pose2d> waypoints) {
        final double FIELD_LENGTH = 17.55;
        final double X_OFFSET = 0.0;

        for (int i = 0; i < waypoints.size(); i++) {
            Pose2d bluePose = waypoints.get(i);
            waypoints.set(
                i,
                new Pose2d(
                    FIELD_LENGTH - bluePose.getX() + X_OFFSET,
                    bluePose.getY(),
                    Rotation2d.fromDegrees(180 - bluePose.getRotation().getDegrees())
                )
            );
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

    public Trigger atStation() {
        return atSetpoint(Setpoints.STATION_WAYPOINTS.get(0))
            .or(atSetpoint(Setpoints.STATION_WAYPOINTS.get(1)));
    }

    public Trigger atReef() {
        return atSetpoint(Setpoints.LEFT_REEF_WAYPOINTS.get(0))
            .or(atSetpoint(Setpoints.LEFT_REEF_WAYPOINTS.get(1)))
            .or(atSetpoint(Setpoints.LEFT_REEF_WAYPOINTS.get(2)))
            .or(atSetpoint(Setpoints.LEFT_REEF_WAYPOINTS.get(3)))
            .or(atSetpoint(Setpoints.LEFT_REEF_WAYPOINTS.get(4)))
            .or(atSetpoint(Setpoints.LEFT_REEF_WAYPOINTS.get(5)))

            .or(atSetpoint(Setpoints.RIGHT_REEF_WAYPOINTS.get(0)))
            .or(atSetpoint(Setpoints.RIGHT_REEF_WAYPOINTS.get(1)))
            .or(atSetpoint(Setpoints.RIGHT_REEF_WAYPOINTS.get(2)))
            .or(atSetpoint(Setpoints.RIGHT_REEF_WAYPOINTS.get(3)))
            .or(atSetpoint(Setpoints.RIGHT_REEF_WAYPOINTS.get(4)))
            .or(atSetpoint(Setpoints.RIGHT_REEF_WAYPOINTS.get(5)))
            .or(atL2Algae())
            .or(atL3Algae());
    }

    /**
     * TODO: make this method
     * @return
     */
    public Trigger atL2Algae() {
        return null;
    }

    /**
     * TODO: do this method
     * @return
     */
    public Trigger atL3Algae() {
        return null;
    }

    /**
     * TODO: do this method
     * @return
     */
    public Trigger atProcessor() {
        return null;
    }

    private Trigger atSetpoint(Pose2d setpoint) {
        Pose2d currentPose = this.getState().Pose;
        double distance = currentPose.getTranslation().getDistance(setpoint.getTranslation());
        double rotationError = Math.abs(currentPose.getRotation().minus(setpoint.getRotation()).getDegrees());

        return new Trigger(
            () -> rotationError < Setpoints.ROTATION_ERROR_TOLERANCE.in(Degrees) &&
                distance < Setpoints.SETPOINT_TOLERANCE.in(Meters)
        );
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