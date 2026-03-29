// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
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
import frc.robot.subsystems.shooter.HoodSubsystem.HardCodedShotAngle;
import frc.robot.subsystems.shooter.ShooterSubsystem.HardCodedShotRPM;
import swervelib.SwerveInputStream;

public class RobotContainer {
  private boolean m_autonomous = false;

  final CommandXboxController m_driverController = new CommandXboxController(0);
  final CommandJoystick m_operatorController = new CommandJoystick(1);

  final LightSubsystem m_lightSubsystem = new LightSubsystem();
  final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve"));
  // final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  final ConveyorSubsystem m_conveyorSubsystem = new ConveyorSubsystem();
  final KickerSubsystem m_kickerSubsystem = new KickerSubsystem();
  final IntakePivotSubsystem m_pivotSubsystem = new IntakePivotSubsystem();
  final IntakeRollerSubsystem m_rollerSubsystem = new IntakeRollerSubsystem();
  final HoodSubsystem m_hoodSubsystem = new HoodSubsystem();
  final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  final FuelLaunchSim m_fuelLaunchSim = new FuelLaunchSim(m_swerveSubsystem::getPose, m_swerveSubsystem::getFieldVelocity);

  public Trigger shouldAutoAlign = new Trigger(() -> m_swerveSubsystem.isAutoAlignOn());

  final Trigger m_isReadyToShoot = new Trigger(() -> {
      return (m_swerveSubsystem.isDrivetrainAligned.getAsBoolean() || !shouldAutoAlign.getAsBoolean()) && m_shooterSubsystem.isShooterOnTarget.getAsBoolean() && m_hoodSubsystem.isHoodOnTarget.getAsBoolean() && !m_autonomous;
  });

  final Trigger m_noIntakeShoot = new Trigger(() -> m_isReadyToShoot.getAsBoolean() && !m_driverController.leftTrigger().getAsBoolean());

  final Trigger teleConveyorJammed = new Trigger(() -> m_conveyorSubsystem.isJammed.getAsBoolean() && !m_autonomous);
  final Trigger teleKickerJammed = new Trigger(() -> m_kickerSubsystem.isJammed.getAsBoolean() && !m_autonomous);

  SendableChooser<PathPlannerAuto> m_autoChooser = new SendableChooser<>();
  
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
      () -> m_driverController.getLeftY() * -1 * 
          (m_driverController.leftBumper().getAsBoolean() ? Constants.DrivebaseConstants.k_slowmodeMultiplier : 1),

      () -> m_driverController.getLeftX() * -1 *
          (m_driverController.leftBumper().getAsBoolean() ? Constants.DrivebaseConstants.k_slowmodeMultiplier : 1)
      )
      .withControllerRotationAxis(() -> -m_driverController.getRightX())
      .deadband(OperatorConstants.k_deadBand)
      .allianceRelativeControl(true);

  public RobotContainer() {
    configureBindings();
    configureFuelSim();
    setupAuto();
  }

  private void configureBindings() {
    Command driveFieldOrientedAnglularVelocity = m_swerveSubsystem.driveFieldOriented(driveAngularVelocity);

    m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    // m_driverController.a().whileTrue(m_rollerSubsystem.run(true));
    // m_driverController.b().whileTrue(m_hoodSubsystem.staticPitch());
    // m_driverController.x().whileTrue(m_hoodSubsystem.returnHood());
    // m_driverController.y().onTrue(m_hoodSubsystem.returnHood());
    // m_driverController.y().onTrue(m_shooterSubsystem.staticShootCommand());
    m_operatorController.button(12).whileTrue(m_pivotSubsystem.fullPower());

    m_conveyorSubsystem.setDefaultCommand(m_conveyorSubsystem.runSlow(true));
    m_shooterSubsystem.setDefaultCommand(m_shooterSubsystem.revCommand(() -> m_swerveSubsystem.inZone()));
    m_hoodSubsystem.setDefaultCommand(m_hoodSubsystem.returnHood());
    m_kickerSubsystem.setDefaultCommand(m_kickerSubsystem.runPassiveOut());

    teleConveyorJammed.onTrue(new ParallelDeadlineGroup(new WaitCommand(0.25), m_conveyorSubsystem.runNormal(false)));
    // teleKickerJammed.onTrue(new ParallelDeadlineGroup(new WaitCommand(0.25), m_kickerSubsystem.run(false)));

    m_driverController.leftTrigger().whileTrue(new SequentialCommandGroup(
      // m_climberSubsystem.retract(),
      m_pivotSubsystem.extend(),
      m_rollerSubsystem.run(true)
    )).onFalse(m_rollerSubsystem.stop());

    m_driverController.rightTrigger().whileTrue(
      new ConditionalCommand(
          new ParallelCommandGroup(
            m_swerveSubsystem.rotateToHub(m_driverController::getLeftX, m_driverController::getLeftY),
            m_hoodSubsystem.pitchHood(() -> m_swerveSubsystem.getHubDist()),
            m_shooterSubsystem.shootCommand(() -> m_swerveSubsystem.getHubDist(), false)
          ),
      new ParallelCommandGroup(
        m_hoodSubsystem.staticPitch(),
        m_shooterSubsystem.staticShootCommand()),
        shouldAutoAlign));

    m_driverController.rightBumper().whileTrue(
      new ConditionalCommand(
          new ParallelCommandGroup(
            m_swerveSubsystem.rotateToPass(m_driverController::getLeftX, m_driverController::getLeftY),
            m_hoodSubsystem.pitchHood(() -> m_swerveSubsystem.getPassDist()),
            m_shooterSubsystem.shootCommand(() -> m_swerveSubsystem.getPassDist(), true)
          ),
      new ParallelCommandGroup(
        m_hoodSubsystem.staticPitch(),
        m_shooterSubsystem.staticShootCommand()),
        shouldAutoAlign));

    m_driverController.povDown().onTrue(m_swerveSubsystem.toggleAutoAlign());

    m_isReadyToShoot.whileTrue(
        new ParallelCommandGroup(m_conveyorSubsystem.runNormal(true), m_kickerSubsystem.run(true))
    );

    m_noIntakeShoot.whileTrue(
      new SequentialCommandGroup(
        new WaitCommand(1),
        new ParallelCommandGroup(m_pivotSubsystem.agitate(), m_rollerSubsystem.run(true))));
    
    m_swerveSubsystem.isDrivetrainAligned.whileTrue(m_fuelLaunchSim.repeatedlyLaunchFuel(() -> 10, () -> 30));
    
    m_driverController.povUp().whileTrue(
      m_pivotSubsystem.extend().andThen(new ParallelCommandGroup(
          m_conveyorSubsystem.runNormal(false),
          m_kickerSubsystem.run(false),
          m_rollerSubsystem.run(false)
          ))
    );
    
        m_driverController.y().whileTrue(
      new ParallelCommandGroup(
        m_hoodSubsystem.hardCodedPitch(HardCodedShotAngle.TOWER.angle),
        m_shooterSubsystem.hardCodedShot(HardCodedShotRPM.TOWER.rpm)
    ));
        m_driverController.x().whileTrue(
      new ParallelCommandGroup(
        m_hoodSubsystem.hardCodedPitch(HardCodedShotAngle.TRENCH.angle),
        m_shooterSubsystem.hardCodedShot(HardCodedShotRPM.TRENCH.rpm)
    ));

    // m_driverController.povLeft().onTrue(new ConditionalCommand(
    //   // m_climberSubsystem.retract(),
    //   new SequentialCommandGroup(
    //     m_pivotSubsystem.retract(),
    //     // m_climberSubsystem.extend(),
    //     new ConditionalCommand(
    //       m_swerveSubsystem.PIDClimb(),
    //       new InstantCommand(),
    //        () -> m_swerveSubsystem.isAutoAlignOn())
    //        ),
    //   m_climberSubsystem.isClimberExtended)
    // );

    // m_driverController.povRight().onTrue(m_climberSubsystem.climbingRetract());

    m_operatorController.button(1).whileTrue(
          new ParallelCommandGroup(m_conveyorSubsystem.runNormal(true), m_kickerSubsystem.run(true) ,m_pivotSubsystem.agitate(), m_rollerSubsystem.runInSlow())
        );
    m_operatorController.button(2).onTrue(m_pivotSubsystem.retract());    
    // m_operatorController.button(3).whileTrue(m_climberSubsystem.setPower(-ClimberConstants.k_manualClimbPower)).onFalse(m_climberSubsystem.setPower(0));
    // m_operatorController.button(4).whileTrue(m_climberSubsystem.setPower(ClimberConstants.k_manualClimbPower)).onFalse(m_climberSubsystem.setPower(0));
    m_operatorController.button(5).onTrue(m_hoodSubsystem.trimShooterCommand(shouldAutoAlign, false));
    m_operatorController.button(6).onTrue(m_hoodSubsystem.trimShooterCommand(shouldAutoAlign, true));
    m_operatorController.button(7).onTrue(m_lightSubsystem.overrideWonAuto(false));
    m_operatorController.button(8).onTrue(m_lightSubsystem.overrideWonAuto(true));
    m_operatorController.button(12).toggleOnTrue(m_swerveSubsystem.wheelRadiusCharacterization(m_swerveSubsystem));
    // m_operatorController.button(11).whileTrue(m_swerveSubsystem.funCommand());
    // m_operatorController.button(12).whileTrue(new PathPlannerAuto("zonesweep"));
  }

  private void setupAuto(){
    NamedCommands.registerCommand("Shoot", new ParallelCommandGroup(
            m_swerveSubsystem.rotateToHub(() -> 0, () -> 0),
            m_hoodSubsystem.pitchHood(() -> m_swerveSubsystem.getHubDist()),
            m_shooterSubsystem.shootCommand(() -> m_swerveSubsystem.getHubDist(), false)
          ));
    NamedCommands.registerCommand("Pass", new ParallelCommandGroup(
            m_swerveSubsystem.rotateToPass(() -> 0, () -> 0),
            m_hoodSubsystem.pitchHood(() -> m_swerveSubsystem.getPassDist()),
            m_shooterSubsystem.shootCommand(() -> m_swerveSubsystem.getPassDist(), true)
          )); 
    NamedCommands.registerCommand("Intake", new ParallelCommandGroup(
      m_pivotSubsystem.extend(),
      m_rollerSubsystem.run(true),
      new ConditionalCommand(m_conveyorSubsystem.runNormal(false), m_conveyorSubsystem.runSlow(true), m_conveyorSubsystem.isJammed)));
    // NamedCommands.registerCommand("Climber Out", new SequentialCommandGroup(
    //     m_pivotSubsystem.retract().deadlineFor(m_rollerSubsystem.run(true)),
    //     m_climberSubsystem.extend()));
    // NamedCommands.registerCommand("Climb", m_climberSubsystem.climbingRetract());
    NamedCommands.registerCommand("Feed", 
        new ParallelCommandGroup(
          new ConditionalCommand(m_conveyorSubsystem.runNormal(false), m_conveyorSubsystem.runNormal(true), m_conveyorSubsystem.isJammed), 
          m_kickerSubsystem.run(true), m_pivotSubsystem.agitate(), m_rollerSubsystem.runInSlow()));
    NamedCommands.registerCommand("Wait", Commands.run(() -> {}).until(() -> m_swerveSubsystem.isDrivetrainAligned.getAsBoolean() && m_shooterSubsystem.isShooterOnTarget.getAsBoolean() && m_hoodSubsystem.isHoodOnTarget.getAsBoolean()));
    NamedCommands.registerCommand("Rev Double Swipe", m_shooterSubsystem.shootCommand(() -> ShooterConstants.k_doubleSwipeShotDist, false).alongWith(m_hoodSubsystem.returnHood()));
    NamedCommands.registerCommand("Hood Down", m_hoodSubsystem.returnHood());
    NamedCommands.registerCommand("PID Climb", m_swerveSubsystem.PIDClimb());
    
    m_autoChooser.addOption("depot", new PathPlannerAuto("auto1"));
    m_autoChooser.addOption("sweep", new PathPlannerAuto("auto2"));
    m_autoChooser.addOption("centerL", new PathPlannerAuto("auto3"));
    m_autoChooser.addOption("centerR", new PathPlannerAuto("auto4"));
    m_autoChooser.addOption("double swipe r", new PathPlannerAuto("auto8"));
    m_autoChooser.addOption("double swipe l", new PathPlannerAuto("auto7"));

    SmartDashboard.putData("auto choice", m_autoChooser);
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  public void switchAuto(boolean isAuto){
    m_autonomous = isAuto;
  }

  private void configureFuelSim() {
    FuelSim instance = FuelSim.getInstance();
    instance.spawnStartingFuel();
    instance.registerRobot(0.876, 0.826, 0.4, m_swerveSubsystem::getPose, m_swerveSubsystem::getFieldVelocity);
    instance.registerIntake(-0.595, -0.438, -0.333, 0.333, () -> (m_fuelLaunchSim.canIntake() && m_rollerSubsystem.getVelocity() > 0), m_fuelLaunchSim::intakeFuel);

    instance.start();
    SmartDashboard.putData(Commands.runOnce(() -> {
                FuelSim.getInstance().clearFuel();
                FuelSim.getInstance().spawnStartingFuel();
            })
            .withName("Reset Fuel")
            .ignoringDisable(true));
  }
}
