// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Rotation;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.drive.Vision.Cameras;
import pabeles.concurrency.ConcurrencyOps.Reset;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.SwerveModule;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

    /**
     * Swerve drive object.
     */
    private final SwerveDrive m_swerveDrive;

    private Vision m_vision;

    private PIDController m_orientPID = new PIDController(DrivebaseConstants.k_rotateP, DrivebaseConstants.k_rotateI, DrivebaseConstants.k_rotateD);

    private PIDController m_xPID = new PIDController(DrivebaseConstants.k_alignP, DrivebaseConstants.k_alignI, DrivebaseConstants.k_alignD);
    private PIDController m_yPID = new PIDController(DrivebaseConstants.k_alignP, DrivebaseConstants.k_alignI, DrivebaseConstants.k_alignD);

    private InterpolatingDoubleTreeMap m_shotTimeInt = new InterpolatingDoubleTreeMap();

    private Pose2d m_curPos = new Pose2d();
    private Pose2d m_sotmPos = new Pose2d();
    
    private boolean spiiningRight = false;
    //ready to shoot if drivetrain is pointing towards aiming target, also requires chassis to be below a specific speed if shooting into hub
    public Trigger isDrivetrainAligned = new Trigger(() -> (Math.abs(m_curPos.getRotation().getDegrees() - getHubAngle()) <= DrivebaseConstants.k_rotateDeadzone) || (Math.abs(m_curPos.getRotation().getDegrees() - getPassAngle()) <= DrivebaseConstants.k_rotateDeadzone));

    private boolean m_autoAlignOn = false;

    /**
     * Initialize {@link SwerveDrive} with the directory provided.
     *
     * @param directory Directory of swerve drive config files.
     */
    public SwerveSubsystem(File directory) {
        Pose2d startingPose = new Pose2d(DrivebaseConstants.k_fieldLengthMeters/2.0, DrivebaseConstants.k_fieldWidthMeters/2.0, new Rotation2d(0));
        // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary
        // objects being created.
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try {
            m_swerveDrive = new SwerveParser(directory).createSwerveDrive(DrivebaseConstants.k_maxSpeed, startingPose);
            // Alternative method if you don't want to supply the conversion factor via JSON
            // files.
            // m_swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed,
            // angleConversionFactor, driveConversionFactor);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        m_swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot
                                                 // via angle.
        m_swerveDrive.setCosineCompensator(false);// !SwerveDriveTelemetry.isSimulation); // Disables cosine compensation
                                                // for simulations since it causes discrepancies not seen in real life.
        m_swerveDrive.setAngularVelocityCompensation(true,
                true,
                0.1); // Correct for skew that gets worse as angular velocity increases. Start with a
                      // coefficient of 0.1.
        m_swerveDrive.setModuleEncoderAutoSynchronize(true,
                1); // Enable if you want to resynchronize your absolute encoders and motor encoders
                    // periodically when they are not moving.
        setupPathPlanner();
        //RobotModeTriggers.autonomous().onTrue(Commands.runOnce(this::zeroGyroWithAlliance));
        m_vision = new Vision(() -> getPose(), m_swerveDrive.field);

        m_orientPID.enableContinuousInput(-180, 180);
        m_orientPID.setIZone(DrivebaseConstants.k_rotateIZone);
        for(double[] values : DrivebaseConstants.k_shotTimes){
            m_shotTimeInt.put(values[0], values[1]);
        }

        SmartDashboard.putData(Commands.run(() -> zeroGyroWithAlliance(), this).withName("reset gyro"));
    }

    /**
     * Construct the swerve drive.
     *
     * @param driveCfg      SwerveDriveConfiguration for the swerve.
     * @param controllerCfg Swerve Controller.
     */
    public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {
        m_swerveDrive = new SwerveDrive(driveCfg,
                controllerCfg,
                DrivebaseConstants.k_maxSpeed,
                new Pose2d(new Translation2d(Meter.of(2), Meter.of(0)),
                        Rotation2d.fromDegrees(0)));
    }

    @Override
    public void periodic() {
        m_vision.updatePoseEstimation(m_swerveDrive);
        SmartDashboard.putBoolean("drivetrain align", isDrivetrainAligned.getAsBoolean());
        SmartDashboard.putNumber("Distance To Hub", getHubDist());
        SmartDashboard.putBoolean("in zone", inZone());

        m_curPos = getPose();
        m_sotmPos = sotmLookAhead(5);
    }

    @Override
    public void simulationPeriodic() {

        }

    /**
     * Setup AutoBuilder for PathPlanner.
     */
    public void setupPathPlanner() {
        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();

            final boolean enableFeedforward = true;
            // Configure AutoBuilder last
            AutoBuilder.configure(
                    this::getPose,
                    // Robot pose supplier
                    this::resetOdometry,
                    // Method to reset odometry (will be called if your auto has a starting pose)
                    this::getRobotVelocity,
                    // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    (speedsRobotRelative, moduleFeedForwards) -> {
                        if (enableFeedforward) {
                            m_swerveDrive.drive(
                                    speedsRobotRelative,
                                    m_swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                                    moduleFeedForwards.linearForces());
                        } else {
                            m_swerveDrive.setChassisSpeeds(speedsRobotRelative);
                        }
                    },
                    // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
                    // optionally outputs individual module feedforwards
                    new PPHolonomicDriveController(
                            // PPHolonomicController is the built in path following controller for holonomic
                            // drive trains
                            new PIDConstants(5.0, 0.0, 0.0),
                            // Translation PID constants
                            new PIDConstants(5.0, 0.0, 0.0)
                    // Rotation PID constants
                    ),
                    config,
                    // The robot configuration
                    () -> {
                        // Boolean supplier that controls when the path will be mirrored for the red
                        // alliance
                        // This will flip the path being followed to the red side of the field.
                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this
            // Reference to this subsystem to set requirements
            );

        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        // Preload PathPlanner Path finding
        // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
        PathfindingCommand.warmupCommand().schedule();
    }

    /**
     * Aim the robot at the target returned by PhotonVision.
     *
     * @return A {@link Command} which will run the alignment.
     */

    /**
     * Get the path follower with events.
     *
     * @param pathName PathPlanner path name.
     * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
     */
    public Command getAutonomousCommand(String pathName) {
        // Create a path following command using AutoBuilder. This will also trigger
        // event markers.
        return new PathPlannerAuto(pathName);
    }

    /**
     * Use PathPlanner Path finding to go to a point on the field.
     *
     * @param pose Target {@link Pose2d} to go to.
     * @return PathFinding command
     */
    public Command driveToPose(Supplier<Pose2d> pose) {
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                m_swerveDrive.getMaximumChassisVelocity(), 4.0,
                m_swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindToPose(
                pose.get(),
                constraints,
                edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
        );
    }
    
    public Command rotateToHub(DoubleSupplier xAxis, DoubleSupplier yAxis) {
        return Commands.run(() -> {
            double x = Math.abs(xAxis.getAsDouble()) > 2*OperatorConstants.k_deadBand ? xAxis.getAsDouble() : 0;
            double y = Math.abs(yAxis.getAsDouble()) > 2*OperatorConstants.k_deadBand ? yAxis.getAsDouble() : 0;
            if(Math.abs(m_curPos.getRotation().getDegrees() - getHubAngle()) < DrivebaseConstants.k_xLockDeadzone && x == 0 && y == 0){
                lock();
            } else {
                if(isRedAlliance()){
                    drive(new Translation2d(2*y, 2*x), computeHubAim(), true);
                } else {
                    drive(new Translation2d(-2*y, -2*x), computeHubAim(), true);
                }
            }    
        }, this);
    }
    
    public Command rotateToPass(DoubleSupplier xAxis, DoubleSupplier yAxis){
        return Commands.run(() -> {
            double x = Math.abs(xAxis.getAsDouble()) > 2*OperatorConstants.k_deadBand ? xAxis.getAsDouble() : 0;
            double y = Math.abs(yAxis.getAsDouble()) > 2*OperatorConstants.k_deadBand ? yAxis.getAsDouble() : 0;
            if(Math.abs(m_curPos.getRotation().getDegrees() - getHubAngle()) < DrivebaseConstants.k_xLockDeadzone && x == 0 && y == 0){
                lock();
            } else {
                if(isRedAlliance()){
                    drive(new Translation2d(2*y, 2*x), computePassAim(), true);
                } else {
                    drive(new Translation2d(-2*y, -2*x), computePassAim(), true);
                }
            }
        }, this);
    }

    public double computeHubAim() {
        return m_orientPID.calculate(m_sotmPos.getRotation().getDegrees(), getHubAngle());
    }

    public double computePassAim() {
        return m_orientPID.calculate(m_curPos.getRotation().getDegrees(), getPassAngle());
    }

    public double getHubAngle() {
        var dx = getHubTarget().getX() - m_sotmPos.getX();
        var dy = getHubTarget().getY() - m_sotmPos.getY();
        var angle = Math.toDegrees(Math.atan2(dy, dx));
        return angle;
    }

    public double getPassAngle() {
        var dx = getPassTarget().getX() - m_curPos.getX();
        var dy = getPassTarget().getY() - m_curPos.getY();
        var angle = Math.toDegrees(Math.atan2(dy, dx));
        return angle;
    }
    // private double normalizeAngle(double angle) {
    //     angle = angle % 360;

    //     if (angle > 180) {
    //         angle -= 360;
    //     } else if (angle < -180) {
    //         angle += 360;
    //     }
    //     return angle;
    // }
    /**
     * Drive with {@link SwerveSetpointGenerator} from 254, implemented by
     * PathPlanner.
     *
     * @param robotRelativeChassisSpeed Robot relative {@link ChassisSpeeds} to
     *                                  achieve.
     * @return {@link Command} to run.
     * @throws IOException    If the PathPlanner GUI settings is invalid
     * @throws ParseException If PathPlanner GUI settings is nonexistent.
     */
    private Command driveWithSetpointGenerator(Supplier<ChassisSpeeds> robotRelativeChassisSpeed)
            throws IOException, ParseException {
        SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(RobotConfig.fromGUISettings(),
                m_swerveDrive.getMaximumChassisAngularVelocity());
        AtomicReference<SwerveSetpoint> prevSetpoint = new AtomicReference<>(
                new SwerveSetpoint(m_swerveDrive.getRobotVelocity(),
                        m_swerveDrive.getStates(),
                        DriveFeedforwards.zeros(m_swerveDrive.getModules().length)));
        AtomicReference<Double> previousTime = new AtomicReference<>();

        return startRun(() -> previousTime.set(Timer.getFPGATimestamp()),
                () -> {
                    double newTime = Timer.getFPGATimestamp();
                    SwerveSetpoint newSetpoint = setpointGenerator.generateSetpoint(prevSetpoint.get(),
                            robotRelativeChassisSpeed.get(),
                            newTime - previousTime.get());
                    m_swerveDrive.drive(newSetpoint.robotRelativeSpeeds(),
                            newSetpoint.moduleStates(),
                            newSetpoint.feedforwards().linearForces());
                    prevSetpoint.set(newSetpoint);
                    previousTime.set(newTime);

                });
    }

    /**
     * Drive with 254's Setpoint generator; port written by PathPlanner.
     *
     * @param fieldRelativeSpeeds Field-Relative {@link ChassisSpeeds}
     * @return Command to drive the robot using the setpoint generator.
     */
    public Command driveWithSetpointGeneratorFieldRelative(Supplier<ChassisSpeeds> fieldRelativeSpeeds) {
        try {
            return driveWithSetpointGenerator(() -> {
                return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds.get(), getHeading());

            });
        } catch (Exception e) {
            DriverStation.reportError(e.toString(), true);
        }
        return Commands.none();

    }

    /**
     * Command to characterize the robot drive motors using SysId
     *
     * @return SysId Drive Command
     */
    public Command sysIdDriveMotorCommand() {
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setDriveSysIdRoutine(
                        new Config(),
                        this, m_swerveDrive, 12, true),
                3.0, 5.0, 3.0);
    }

    /**
     * Command to characterize the robot angle motors using SysId
     *
     * @return SysId Angle Command
     */
    public Command sysIdAngleMotorCommand() {
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setAngleSysIdRoutine(
                        new Config(),
                        this, m_swerveDrive),
                3.0, 5.0, 3.0);
    }

    /**
     * Returns a Command that centers the modules of the SwerveDrive subsystem.
     *
     * @return a Command that centers the modules of the SwerveDrive subsystem
     */
    public Command centerModulesCommand() {
        return run(() -> Arrays.asList(m_swerveDrive.getModules())
                .forEach(it -> it.setAngle(0.0)));
    }

    /**
     * Returns a Command that drives the swerve drive to a specific distance at a
     * given speed.
     *
     * @param distanceInMeters       the distance to drive in meters
     * @param speedInMetersPerSecond the speed at which to drive in meters per
     *                               second
     * @return a Command that drives the swerve drive to a specific distance at a
     *         given speed
     */
    public Command driveToDistanceCommand(double distanceInMeters, double speedInMetersPerSecond) {
        return run(() -> drive(new ChassisSpeeds(speedInMetersPerSecond, 0, 0)))
                .until(() -> m_swerveDrive.getPose().getTranslation()
                        .getDistance(new Translation2d(0, 0)) > distanceInMeters);
    }

    /**
     * Replaces the swerve module feedforward with a new SimpleMotorFeedforward
     * object.
     *
     * @param kS the static gain of the feedforward
     * @param kV the velocity gain of the feedforward
     * @param kA the acceleration gain of the feedforward
     */
    public void replaceSwerveModuleFeedforward(double kS, double kV, double kA) {
        m_swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
    }

    /**
     * Command to drive the robot using translative values and heading as angular
     * velocity.
     *
     * @param translationX     Translation in the X direction. Cubed for smoother
     *                         controls.
     * @param translationY     Translation in the Y direction. Cubed for smoother
     *                         controls.
     * @param angularRotationX Angular velocity of the robot to set. Cubed for
     *                         smoother controls.
     * @return Drive command.
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
            DoubleSupplier angularRotationX) {
        return run(() -> {
            // Make the robot move
            m_swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                    translationX.getAsDouble() * m_swerveDrive.getMaximumChassisVelocity(),
                    translationY.getAsDouble() * m_swerveDrive.getMaximumChassisVelocity()), 0.8),
                    Math.pow(angularRotationX.getAsDouble(), 3) * m_swerveDrive.getMaximumChassisAngularVelocity(),
                    true,
                    false);
        });
    }

    /**
     * Command to drive the robot using translative values and heading as a
     * setpoint.
     *
     * @param translationX Translation in the X direction. Cubed for smoother
     *                     controls.
     * @param translationY Translation in the Y direction. Cubed for smoother
     *                     controls.
     * @param headingX     Heading X to calculate angle of the joystick.
     * @param headingY     Heading Y to calculate angle of the joystick.
     * @return Drive command.
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
            DoubleSupplier headingY) {
        // m_swerveDrive.setHeadingCorrection(true); // Normally you would want heading
        // correction for this kind of control.
        return run(() -> {

            Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                    translationY.getAsDouble()), 0.8);

            // Make the robot move
            driveFieldOriented(m_swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                    headingX.getAsDouble(),
                    headingY.getAsDouble(),
                    m_swerveDrive.getOdometryHeading().getRadians(),
                    m_swerveDrive.getMaximumChassisVelocity()));
        });
    }

    /**
     * The primary method for controlling the drivebase. Takes a
     * {@link Translation2d} and a rotation rate, and
     * calculates and commands module states accordingly. Can use either open-loop
     * or closed-loop velocity control for
     * the wheel velocities. Also has field- and robot-relative modes, which affect
     * how the translation vector is used.
     *
     * @param translation   {@link Translation2d} that is the commanded linear
     *                      velocity of the robot, in meters per
     *                      second. In robot-relative mode, positive x is torwards
     *                      the bow (front) and positive y is
     *                      torwards port (left). In field-relative mode, positive x
     *                      is away from the alliance wall
     *                      (field North) and positive y is torwards the left wall
     *                      when looking through the driver station
     *                      glass (field West).
     * @param rotation      Robot angular rate, in radians per second. CCW positive.
     *                      Unaffected by field/robot
     *                      relativity.
     * @param fieldRelative Drive mode. True for field-relative, false for
     *                      robot-relative.
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        m_swerveDrive.drive(translation,
                rotation,
                fieldRelative,
                false); // Open loop is disabled since it shouldn't be used most of the time.
    }

    /**
     * Drive the robot given a chassis field oriented velocity.
     *
     * @param velocity Velocity according to the field.
     */
    public void driveFieldOriented(ChassisSpeeds velocity) {
        m_swerveDrive.driveFieldOriented(velocity);
    }

    /**
     * Drive the robot given a chassis field oriented velocity.
     *
     * @param velocity Velocity according to the field.
     */
    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
        return run(() -> {
            m_swerveDrive.driveFieldOriented(velocity.get());
        });
    }

    /**
     * Drive according to the chassis robot oriented velocity.
     *
     * @param velocity Robot oriented {@link ChassisSpeeds}
     */
    public void drive(ChassisSpeeds velocity) {
        m_swerveDrive.drive(velocity);
    }

    /**
     * Get the swerve drive kinematics object.
     *
     * @return {@link SwerveDriveKinematics} of the swerve drive.
     */
    public SwerveDriveKinematics getKinematics() {
        return m_swerveDrive.kinematics;
    }

    /**
     * Resets odometry to the given pose. Gyro angle and module positions do not
     * need to be reset when calling this
     * method. However, if either gyro angle or module position is reset, this must
     * be called in order for odometry to
     * keep working.
     *
     * @param initialHolonomicPose The pose to set the odometry to
     */
    public void resetOdometry(Pose2d initialHolonomicPose) {
        m_swerveDrive.resetOdometry(initialHolonomicPose);
    }

    /**
     * Gets the current pose (position and rotation) of the robot, as reported by
     * odometry.
     *
     * @return The robot's pose
     */
    public Pose2d getPose() {
        return m_swerveDrive.getPose();
    }

    /**
     * Set chassis speeds with closed-loop velocity control.
     *
     * @param chassisSpeeds Chassis Speeds to set.
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        m_swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    /**
     * Post the trajectory to the field.
     *
     * @param trajectory The trajectory to post.
     */
    public void postTrajectory(Trajectory trajectory) {
        m_swerveDrive.postTrajectory(trajectory);
    }

    /**
     * Resets the gyro angle to zero and resets odometry to the same position, but
     * facing toward 0.
     */
    public void zeroGyro() {
        m_swerveDrive.zeroGyro();
    }

    public Command resetGyro() {
        return Commands.runOnce(() -> {
            zeroGyroWithAlliance();
        });
    }

    /**
     * Checks if the alliance is red, defaults to false if alliance isn't available.
     *
     * @return true if the red alliance, false if blue. Defaults to false if none is
     *         available.
     */
    private boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }

    /**
     * This will zero (calibrate) the robot to assume the current position is facing
     * forward
     * <p>
     * If red alliance rotate the robot 180 after the drviebase zero command
     */
    public void zeroGyroWithAlliance() {
        if (isRedAlliance()) {
            zeroGyro();
            // Set the pose 180 degrees
            resetOdometry(new Pose2d(m_curPos.getTranslation(), Rotation2d.fromDegrees(180)));
        } else {
            zeroGyro();
        }
    } 

    /**
     * Sets the drive motors to brake/coast mode.
     *
     * @param brake True to set motors to brake mode, false for coast.
     */
    public void setMotorBrake(boolean brake) {
        m_swerveDrive.setMotorIdleMode(brake);
    }

    /**
     * Gets the current yaw angle of the robot, as reported by the swerve pose
     * estimator in the underlying drivebase.
     * Note, this is not the raw gyro reading, this may be corrected from calls to
     * resetOdometry().
     *
     * @return The yaw angle
     */
    public Rotation2d getHeading() {
        return m_curPos.getRotation();
    }

    /**
     * Get the chassis speeds based on controller input of 2 joysticks. One for
     * speeds in which direction. The other for
     * the angle of the robot.
     *
     * @param xInput   X joystick input for the robot to move in the X direction.
     * @param yInput   Y joystick input for the robot to move in the Y direction.
     * @param headingX X joystick which controls the angle of the robot.
     * @param headingY Y joystick which controls the angle of the robot.
     * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
     */
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
        Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
        return m_swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                scaledInputs.getY(),
                headingX,
                headingY,
                getHeading().getRadians(),
                DrivebaseConstants.k_maxSpeed);
    }

    /**
     * Get the chassis speeds based on controller input of 1 joystick and one angle.
     * Control the robot at an offset of
     * 90deg.
     *
     * @param xInput X joystick input for the robot to move in the X direction.
     * @param yInput Y joystick input for the robot to move in the Y direction.
     * @param angle  The angle in as a {@link Rotation2d}.
     * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
     */
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
        Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

        return m_swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                scaledInputs.getY(),
                angle.getRadians(),
                getHeading().getRadians(),
                DrivebaseConstants.k_maxSpeed);
    }

    /**
     * Gets the current field-relative velocity (x, y and omega) of the robot
     *
     * @return A ChassisSpeeds object of the current field-relative velocity
     */
    public ChassisSpeeds getFieldVelocity() {
        return m_swerveDrive.getFieldVelocity();
    }

    /**
     * Gets the current velocity (x, y and omega) of the robot
     *
     * @return A {@link ChassisSpeeds} object of the current velocity
     */
    public ChassisSpeeds getRobotVelocity() {
        return m_swerveDrive.getRobotVelocity();
    }

    /**
     * Get the {@link SwerveController} in the swerve drive.
     *
     * @return {@link SwerveController} from the {@link SwerveDrive}.
     */
    public SwerveController getSwerveController() {
        return m_swerveDrive.swerveController;
    }

    /**
     * Get the {@link SwerveDriveConfiguration} object.
     *
     * @return The {@link SwerveDriveConfiguration} fpr the current drive.
     */
    public SwerveDriveConfiguration getSwerveDriveConfiguration() {
        return m_swerveDrive.swerveDriveConfiguration;
    }

    /**
     * Lock the swerve drive to prevent it from moving.
     */
    public void lock() {
        m_swerveDrive.lockPose();
    }

    /**
     * Gets the current pitch angle of the robot, as reported by the imu.
     *
     * @return The heading as a {@link Rotation2d} angle
     */
    public Rotation2d getPitch() {
        return m_swerveDrive.getPitch();
    }

    /**
     * Add a fake m_vision reading for testing purposes.
     */
    public void addFakeVisionReading() {
        m_swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
    }

    /**
     * Gets the swerve drive object.
     *
     * @return {@link SwerveDrive}
     */
    public SwerveDrive getSwerveDrive() {
        return m_swerveDrive;
    }

    public Translation2d getHubTarget(){
        return isRedAlliance() ?
         new Translation2d(DrivebaseConstants.k_fieldLengthMeters - DrivebaseConstants.k_blueHub.getX(), DrivebaseConstants.k_blueHub.getY()) :
         DrivebaseConstants.k_blueHub;
    }

    public boolean inZone(){
        if(isRedAlliance()){
            return m_curPos.getX() > DrivebaseConstants.k_redZoneX;
        } else {
            return m_curPos.getX() < DrivebaseConstants.k_blueZoneX;
        }
    }

    public Translation2d getPassTarget(){
        Translation2d target = DrivebaseConstants.k_blueOutpost;
        if(isRedAlliance()){
            if(m_curPos.getX() < DrivebaseConstants.k_blueZoneX){
                target = DrivebaseConstants.k_midField;
            }
            target = new Translation2d(DrivebaseConstants.k_fieldLengthMeters - target.getX() ,target.getY());
        } else {
            if(m_curPos.getX() > DrivebaseConstants.k_redZoneX){
                target = DrivebaseConstants.k_midField;
            }
        }
        if(m_curPos.getY() > (DrivebaseConstants.k_fieldWidthMeters/2.0)){
            target = new Translation2d(target.getX(), DrivebaseConstants.k_fieldWidthMeters - target.getY());
        }
        return target;
    }

    public double getHubDist(){
        return getHubTarget().getDistance(m_sotmPos.getTranslation());
    }

    public double getPassDist(){
        return getPassTarget().getDistance(m_curPos.getTranslation());
    }

    public Pose2d getFuturePos(double lookAhead) {
        double xDist = m_curPos.getX() + (getFieldVelocity().vxMetersPerSecond * lookAhead);
        double yDist = m_curPos.getY() + (getFieldVelocity().vyMetersPerSecond * lookAhead);
        return new Pose2d(xDist, yDist, m_curPos.getRotation());
    }

    //iteratively look forward in time to find aiming pose for shoot on the move
    public Pose2d sotmLookAhead(int iterations){
        Pose2d futurePos = getFuturePos(m_shotTimeInt.get(getHubDist()));
        for(int i = 0; i < iterations; i++){
            double futureShotTime = m_shotTimeInt.get(getHubTarget().getDistance(futurePos.getTranslation()));
            futurePos = getFuturePos(futureShotTime);
        }
        return futurePos;
    }

    public boolean isAutoAlignOn() {
        return m_autoAlignOn;
    }

    public Command toggleAutoAlign(){
        return Commands.runOnce(() -> {
            m_autoAlignOn = !m_autoAlignOn;
        }, this);
    }

    public Command runManual(){
        return Commands.run(() -> {
            for (SwerveModule module : m_swerveDrive.getModules()){
                module.getAngleMotor().set(0.1);
            }
        }, this);
    }

    public Supplier<Rotation2d> orientPID(DoubleSupplier targetRotation){
        double setPointDegrees = targetRotation.getAsDouble();
        double heading = getPose().getRotation().getDegrees();
        double rotation = MathUtil.clamp(m_orientPID.calculate(heading, setPointDegrees), -0.8, 0.8);
        return () -> Rotation2d.fromDegrees(rotation);
    }

    public Translation2d getClimbTarget(){
        Translation2d target = DrivebaseConstants.k_leftClimb;
        if(isRedAlliance()){
            if(m_curPos.getY() > DrivebaseConstants.k_climbCenter){
                target = DrivebaseConstants.k_rightClimb;
            }
        } else {
            if(m_curPos.getY() < (DrivebaseConstants.k_fieldWidthMeters - DrivebaseConstants.k_climbCenter)){
                target = DrivebaseConstants.k_rightClimb;
            }
            target = new Translation2d(DrivebaseConstants.k_fieldLengthMeters - target.getX(), DrivebaseConstants.k_fieldWidthMeters - target.getY());
        }
        return target;
    }

    public Command PIDClimb() {
        return Commands.runEnd(() -> {
            double x = MathUtil.clamp(m_xPID.calculate(getPose().getX(), getClimbTarget().getX()), -0.8, 0.8);
            double y = MathUtil.clamp(m_yPID.calculate(getPose().getY(), getClimbTarget().getY()), -0.8, 0.8);
            drive(new Translation2d(x, y), orientPID(() -> isRedAlliance() ? 0 : 180).get().getDegrees(), true);
        }, () -> {
        drive(new ChassisSpeeds(0, 0, 0));
        }
        , this).until(() -> Math.abs(getClimbTarget().getDistance(getPose().getTranslation())) < DrivebaseConstants.k_alignTolerance);
    }  

    public Command funCommand() {
        Timer spiinTimer = new Timer();
        return Commands.run(()->{
            SmartDashboard.putBoolean("right", spiiningRight);
            if (spiinTimer.get() > 0.75){
                spiiningRight = !spiiningRight;
                spiinTimer.reset();
            }
            setChassisSpeeds(new ChassisSpeeds(0, 0, spiiningRight ? -1 : 1));
        }, this);
    }

    public Command wheelRadiusCharacterization(SwerveSubsystem drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(DrivebaseConstants.k_wheelRadiusRampRate);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(DrivebaseConstants.k_wheelRadiusMaxVelocity);
                  drive.drive(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getPose().getRotation();
                  state.gyroDelta = 0.0;
                }),

            //Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getPose().getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius =
                          (state.gyroDelta * DrivebaseConstants.k_driveBaseRadius) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }

  public double[] getWheelRadiusCharacterizationPositions() {
    
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = getModuleCharacterizationPosition(m_swerveDrive.getModules()[i]);
    }
    return values;
  }

  public double getModuleCharacterizationPosition(SwerveModule module){
    return (module.getDriveMotor().getPosition() / DrivebaseConstants.k_wheelRadiusMeters);
  }
}