// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final class DrivebaseConstants
  {
    public static final double k_robotMass = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final Matter k_chassis    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), k_robotMass);
    public static final double k_loopTime  = 0.13; //s, 20ms + 110ms sprk max velocity lag
    public static final double k_maxSpeed  = Units.feetToMeters(14.5);

    // Hold time on motor brakes when disabled
    public static final double k_wheelLockTime = 10; // seconds

    public static final double k_rotateP = 0.05;
    public static final double k_rotateI = 0.1;
    public static final double k_rotateD = 0.0008;

    public static final double k_rotateIZone = 20;
    public static final double k_rotateDeadzone = 0;

    public static final double k_fieldLengthMeters = 16.541;
    public static final double k_fieldWidthMeters = 8.069;

    public static final Translation2d k_blueHub = new Translation2d(4.625, 4.05);
    public static final Translation2d k_blueOutpost = new Translation2d(0.365, 0.675);
    public static final double k_blueZoneX = 4.625;
    public static final double k_redZoneX = 11.925;

    public static final double[][] k_shotTimes = {{0,0},{0,0}};
  }

  public static class IntakeConstants{
    public static double k_rollerMOI = 0;
    public static final double k_rollerReduction = 0;

    public static final double k_pivotMOI = 0;
    public static final double k_pivotReduction = 0;
    public static final double k_pivotLength = 0;
    public static final double k_pivotMaxRotation = 0;

    public static final double k_rollerKV = 0;
    public static final double k_rollerP = 0;

    public static final double k_pivotKCos = 0;
    public static final double k_pivotP = 0;
    public static final double k_pivotI = 0;
    public static final double k_pivotD = 0;

    public static final int k_rollerCurrent = 60;
    public static final double k_rollerInSpeed = 0;
    public static final double k_rollerOutSpeed = 0;
    
    public static final int k_pivotCurrent = 80;

    public static final SparkFlexConfig k_rollerConfig = new SparkFlexConfig();

    public static final SparkFlexConfig k_pivotConfig = new SparkFlexConfig();

    static {
      k_rollerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(k_rollerCurrent);

      k_rollerConfig.closedLoop.p(k_rollerP).feedbackSensor(FeedbackSensor.kPrimaryEncoder).
      feedForward.kV(k_rollerKV);

      k_pivotConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(k_pivotCurrent)
      .absoluteEncoder.positionConversionFactor(360);

      k_pivotConfig.closedLoop.pid(k_pivotP, k_pivotI, k_pivotD).feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .feedForward.kCosRatio(1.0/360.0).kCos(k_pivotKCos);
    }
  }

  public static class ShooterConstants{
    public static final double k_hoodGearRatio = 0.0;
    public static final double k_hoodMomentOfInertia = 0.0;
    public static final double k_hoodArmLengthMeters = 0.0;
    
    public static final double k_shooterMomentOfInertia = 0.0;
    public static final double k_shooterMotorReduction = 0.0;

    public static final int k_hoodCurrentLimit = 60;
    public static final double k_hoodP = 0;
    public static final double k_hoodI = 0;
    public static final double k_hoodD = 0;
    public static final double k_hoodDeadzone = 0;

    public static final int k_shooterCurrent = 40;
    public static final double k_shooterKV = 0;
    public static final double k_shooterP = 0;
    public static final double k_shooterDeadzone = 0;

    public static final double k_staticHoodAngle = 0;

    public static final double k_shooterRevVel = 0;

    public static final SparkFlexConfig k_hoodConfig = new SparkFlexConfig();
    public static final SparkFlexConfig k_leaderConfig = new SparkFlexConfig();
    public static final SparkFlexConfig k_follower1Config = new SparkFlexConfig();
    public static final SparkFlexConfig k_follower23Config = new SparkFlexConfig();

    static {
      k_hoodConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(k_hoodCurrentLimit)
      .absoluteEncoder.positionConversionFactor(360);

      k_hoodConfig.closedLoop.pid(k_hoodP, k_hoodI, k_hoodD)
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

      k_leaderConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(k_shooterCurrent);

      k_leaderConfig.closedLoop.p(k_shooterP).feedForward.kV(k_shooterKV);
      k_follower1Config.apply(k_leaderConfig).follow(CANIDConstants.shooter_1, false);
      k_follower23Config.apply(k_leaderConfig).follow(CANIDConstants.shooter_1, true);
    }
  
  }

  public static class IndexerConstants{
    public static final double k_conveyorMOI = 0;
    public static final double k_conveyorReduction = 0;

    public static final double k_kickerMOI = 0;
    public static final double k_kickerReduction = 0;

    public static final double k_conveyorKV = 0;
    public static final double k_conveyorP = 0;

    public static final double k_kickerKV = 0;
    public static final double k_kickerP = 0;

    public static final int k_conveyorCurrent = 40;
    public static final double k_conveyorInSpeed = 0;
    public static final double k_conveyorOutSpeed = 0;

    public static final int k_kickerCurrent = 40;
    public static final double k_kickerInSpeed = 0;
    public static final double k_kickerOutSpeed = 0;

    public static final SparkFlexConfig k_conveyorConfig = new SparkFlexConfig();

    public static final SparkFlexConfig k_kickerConfig = new SparkFlexConfig();
    static {
      k_conveyorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(k_conveyorCurrent);

      k_kickerConfig.apply(k_conveyorConfig);

      k_conveyorConfig.closedLoop.p(k_conveyorP).feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .feedForward.kV(k_conveyorKV);

      k_conveyorConfig.closedLoop.p(k_kickerP).feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .feedForward.kV(k_kickerKV);
    }
  }

  public static class ClimberConstants{
    public static final double k_climberWeight = 0;
    public static final double k_climberDrumWidth = 0;
    public static final double k_climberReduction = 0;
    public static final double k_climberMaxHeight = 0;
    public static final double k_climberRotationsToMeters = 0;

    public static final double k_climberDeadzone = 0;

    public static final double k_climberP = 0;
    public static final double k_climberI = 0;
    public static final double k_climberD = 0;

    public static final int k_climberCurrent = 80;

    public static final SparkFlexConfig k_leadConfig = new SparkFlexConfig();
    public static final SparkFlexConfig k_followConfig = new SparkFlexConfig();

    static {
      k_leadConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(k_climberCurrent)
      .encoder.positionConversionFactor(k_climberRotationsToMeters).velocityConversionFactor(k_climberRotationsToMeters / 60);

      k_leadConfig.closedLoop.pid(k_climberP, k_climberI, k_climberD);

      k_followConfig.apply(k_leadConfig).follow(CANIDConstants.climber_leader, true);
    }
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double k_deadBand        = 0.1;
    public static final double k_leftYDeadBand = 0.1;
    public static final double k_rightXDeadBand = 0.1;
    public static final double k_turnConstant    = 6;
  }

  public static class CANIDConstants {
    public static final int gyro = 0;
    public static final int fl_drive = 1;
    public static final int fl_turn = 2; 
    public static final int fr_drive = 3; 
    public static final int fr_turn = 4; 
    public static final int bl_drive = 5; 
    public static final int bl_turn = 6; 
    public static final int br_drive = 7; 
    public static final int br_turn = 8;
    public static final int intake_pivot = 10;
    public static final int intake_roller = 11;
    public static final int conveyor = 20;
    public static final int kicker = 21;
    public static final int shooter_1 = 30;
    public static final int shooter_2 = 31;
    public static final int shooter_3 = 32;
    public static final int shooter_4 = 33;
    public static final int shooter_hood = 34;
    public static final int climber_leader = 40;
    public static final int climber_follower = 41;
  }
    public static class LightConstants {
    public static final int k_lightPort = 0; // may be different on actual robot
    public static final int k_lightAmount = 60;

    public static LinearVelocity k_rainbowVelocity = MetersPerSecond.of(0.4);
    public static Distance k_ledSpacing = Meters.of(1 / 120.0);
    
  }
}