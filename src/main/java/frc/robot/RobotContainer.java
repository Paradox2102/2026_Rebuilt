// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.indexer.ConveyorSubsystem;
import frc.robot.subsystems.indexer.KickerSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.light.LightSubsystem;
import frc.robot.subsystems.shooter.FuelLaunchSim;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {
  final CommandXboxController m_driverController = new CommandXboxController(0);
  final CommandJoystick m_operatorController = new CommandJoystick(1);

  final LightSubsystem m_lightSubsystem = new LightSubsystem();
  final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve"), m_lightSubsystem);
  final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  final ConveyorSubsystem m_conveyorSubsystem = new ConveyorSubsystem();
  final KickerSubsystem m_kickerSubsystem = new KickerSubsystem();
  final IntakePivotSubsystem m_pivotSubsystem = new IntakePivotSubsystem();
  final IntakeRollerSubsystem m_rollerSubsystem = new IntakeRollerSubsystem();
  final HoodSubsystem m_hoodSubsystem = new HoodSubsystem();
  final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

  final FuelLaunchSim m_fuelLaunchSim = new FuelLaunchSim(m_swerveSubsystem::getPose, m_swerveSubsystem::getFieldVelocity);

  final Trigger m_isReadyToShoot = new Trigger(() -> {
      return m_swerveSubsystem.isDrivetrainAligned.getAsBoolean() && m_shooterSubsystem.isShooterOnTarget.getAsBoolean() && m_hoodSubsystem.isHoodOnTarget.getAsBoolean();
  });

  public Trigger shouldAutoAlign = new Trigger(() -> m_operatorController.getThrottle() < 0);
  
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
      () -> m_driverController.getLeftY() * -1,
      () -> m_driverController.getLeftX() * -1)
      .withControllerRotationAxis(() -> -m_driverController.getRightX())
      .deadband(OperatorConstants.k_deadBand)
      .allianceRelativeControl(true);

  SwerveInputStream driveAngularVelocityAim = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
      () -> m_driverController.getLeftY() * -1,
      () -> m_driverController.getLeftX() * -1)
      .withControllerRotationAxis(() -> -m_swerveSubsystem.rotationPID())
      .deadband(OperatorConstants.k_deadBand)
      .allianceRelativeControl(true);

  public RobotContainer() {
    configureBindings();
    configureFuelSim();
  }

  private void configureBindings() {
    Command driveFieldOrientedAnglularVelocity = m_swerveSubsystem.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedAutoAim = m_swerveSubsystem.driveFieldOriented(driveAngularVelocityAim);

    m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    // m_driverController.a().whileTrue(driveFieldOrientedAutoAim);
    m_swerveSubsystem.isDrivetrainAligned.whileTrue(m_fuelLaunchSim.repeatedlyLaunchFuel(() -> 1, () -> 20));

    m_conveyorSubsystem.setDefaultCommand(m_conveyorSubsystem.runSlow(true));
    m_shooterSubsystem.setDefaultCommand(m_shooterSubsystem.revCommand());
    m_hoodSubsystem.setDefaultCommand(m_hoodSubsystem.returnHood());
    m_kickerSubsystem.setDefaultCommand(m_kickerSubsystem.stop());

    m_driverController.leftTrigger().whileTrue(new SequentialCommandGroup(
      new ConditionalCommand(new Command(){}, m_climberSubsystem.retract(), m_climberSubsystem.isClimberRetracted), 
      m_pivotSubsystem.extend(),
      m_rollerSubsystem.run(true)
    )).onFalse(m_rollerSubsystem.stop());

    m_driverController.rightTrigger().whileTrue(new ConditionalCommand(
      new ParallelCommandGroup(
        //align swerve subsystem
        m_hoodSubsystem.pitchHood(() -> m_swerveSubsystem.getTargetDist()),
        m_shooterSubsystem.shootCommand(() -> m_swerveSubsystem.getTargetDist())
      ),
      new ParallelCommandGroup(
        m_hoodSubsystem.staticPitch(),
        m_shooterSubsystem.staticShootCommand()),

      () -> m_swerveSubsystem.isDrivetrainAligned.getAsBoolean()));
    m_isReadyToShoot.whileTrue(
        new ParallelCommandGroup(
          m_conveyorSubsystem.runNormal(true),
          m_kickerSubsystem.run(true),
          m_swerveSubsystem.rotateToHub(),
          m_hoodSubsystem.pitchHood(() -> m_swerveSubsystem.getTargetDist()),
          m_shooterSubsystem.shootCommand(() -> m_swerveSubsystem.getTargetDist())
        )
    );
    m_driverController.button(0).whileTrue(new ParallelCommandGroup(
          m_conveyorSubsystem.runNormal(false),
          m_kickerSubsystem.run(false)
          )
    );
    m_driverController.button(1).onTrue(new SequentialCommandGroup(
      m_pivotSubsystem.retract(),
      m_rollerSubsystem.stop(),
      m_climberSubsystem.extend())
    );
    m_driverController.button(2).onTrue(m_climberSubsystem.climbingRetract());

    // shouldAutoAlign.onTrue().onFalse(); //toggle auto align on and off.

    m_driverController.button(3).whileTrue(m_climberSubsystem.setPower(-ClimberConstants.k_manualClimbPower));
    m_driverController.button(4).whileTrue(m_climberSubsystem.setPower(ClimberConstants.k_manualClimbPower));

    m_driverController.button(5).whileTrue(
      new SequentialCommandGroup(
        new ParallelCommandGroup(
          m_shooterSubsystem.shootCommand(() -> m_swerveSubsystem.getTargetDist()),
          m_hoodSubsystem.pitchHood(() -> m_swerveSubsystem.getTargetDist())),
          m_conveyorSubsystem.runNormal(false)
      )
   );
   m_driverController.b().whileTrue(m_swerveSubsystem.rotateToHub());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  private void configureFuelSim() {
    FuelSim instance = FuelSim.getInstance();
    instance.spawnStartingFuel();
    instance.registerRobot(0.876, 0.826, 0.152, m_swerveSubsystem::getPose, m_swerveSubsystem::getFieldVelocity);
    instance.registerIntake(-0.47, -0.595, -0.333, 0.333, () -> true, m_fuelLaunchSim::intakeFuel);

    instance.start();
    SmartDashboard.putData(Commands.runOnce(() -> {
                FuelSim.getInstance().clearFuel();
                FuelSim.getInstance().spawnStartingFuel();
            })
            .withName("Reset Fuel")
            .ignoringDisable(true));
  }
}
