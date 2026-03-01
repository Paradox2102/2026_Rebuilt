// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
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

    public static final double k_alignP = 2.5;
    public static final double k_alignI = 0;
    public static final double k_alignD = 0;

    public static final double k_rotateP = 0.1;
    public static final double k_rotateI = 0.1;
    public static final double k_rotateD = 0;

    public static final double k_rotateIZone = 5;
    public static final double k_rotateDeadzone = 5;

    public static final double k_alignTolerance = 0.5125;

    public static final double k_aimTurnDeadzone = 0.1;

    public static final double k_slowmodeMultiplier = 0.5; // half speed
    
    public static final double k_maxDtShootingSpeed = 0;

    public static final double k_fieldLengthMeters = 16.541;
    public static final double k_fieldWidthMeters = 8.069;

    public static final Translation2d k_blueHub = new Translation2d(4.625, 4.05);
    public static final Translation2d k_blueOutpost = new Translation2d(0.7, 1.25);
    public static final Translation2d k_midField = new Translation2d(k_fieldLengthMeters/2.0, 1.5);

    public static final Translation2d k_leftClimb = new Translation2d(15.05, 3.61);
    public static final Translation2d k_rightClimb = new Translation2d(15.05, 4.63);

    public static final double k_blueZoneX = 4.625;
    public static final double k_redZoneX = 11.925;

    public static final double[][] k_shotTimes = {
      {1.25, 1},
      {1.5, 1},
      {1.75, 1},
      {2, 1},
      {2.25, 1},
      {2.5, 1},
      {2.75, 1},
      {3, 1},
      {3.25, 1},
      {3.5, 1},
      {3.75, 1},
      {4, 1},
      {4.25, 1},
      {4.5, 1},
      {4.75, 1},
      {5, 1},
      {5.25, 1}
    };
  }

  public static class VisionConstants{
    public static final Vector<N3> k_visionBaseSDev = VecBuilder.fill(3, 3, 3);
    public static final Vector<N3> k_multiTagSDev = VecBuilder.fill(0.9, 0.9, 0.9);

    public static final Translation3d k_flTranslation = new Translation3d(0.327, 0.180, 0.246);
    public static final Rotation3d k_flRotation = new Rotation3d(0, -Math.toRadians(25), -Math.toRadians(30));
    public static final Translation3d k_frTranslation = new Translation3d(0.327, -0.180, 0.233);
    public static final Rotation3d k_frRotation = new Rotation3d(0, -Math.toRadians(25), Math.toRadians(30));
    public static final Translation3d k_slTranslation = new Translation3d(0.111, 0.343, 0.490);
    public static final Rotation3d k_slRotation = new Rotation3d(0, -Math.toRadians(15), Math.toRadians(90));
    public static final Translation3d k_srTranslation = new Translation3d(0.111, -0.343, 0.490);
    public static final Rotation3d k_srRotation = new Rotation3d(0, -Math.toRadians(15), -Math.toRadians(90));
  }

  public static class IntakeConstants{
    public static double k_rollerMOI = 0.00074;
    public static final double k_rollerReduction = 1.33;

    public static final double k_pivotMOI = 0.5426;
    public static final double k_pivotReduction = 16;
    public static final double k_pivotLength = 0.7;
    public static final double k_pivotMaxRotation = 93.5;
    public static final double k_pivotDeadzone = 2;
    public static final double k_pivotConversionFactor = 93.5/3.94322;

    public static final double k_pivotPullInHeight = 61.0;

    public static final double k_rollerKV = 0.002;

    public static final double k_pivotP = 0.025; //0.25
    public static final double k_pivotI = 0; //0.1
    public static final double k_pivotD = 0;
    public static final double k_pivotCosF = 0.075;

    public static final int k_rollerCurrent = 40;
    public static final double k_rollerInSpeed = 4000;
    public static final double k_rollerOutSpeed = -4000;
    public static final double k_rollerSlowInSpeed = 1000;
    
    public static final int k_pivotCurrent = 60;

    public static final SparkFlexConfig k_rollerConfig = new SparkFlexConfig();

    public static final SparkFlexConfig k_pivotConfig = new SparkFlexConfig();

    static {
      k_rollerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(k_rollerCurrent).inverted(true);

      k_rollerConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).
      feedForward.kV(k_rollerKV);

      k_pivotConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(k_pivotCurrent).inverted(false)
      .encoder.positionConversionFactor(k_pivotConversionFactor);
    }
  }

  public static class ShooterConstants{
    public static final double[][] k_shotAngles = {
      {1.25, 0},
      {1.5, 5},
      {1.75, 10},
      {2, 12.5},
      {2.25, 15},
      {2.5, 19},
      {2.75, 20},
      {3, 22},
      {3.25, 23},
      {3.5, 24},
      {3.75, 25},
      {4, 26},
      {4.25, 27},
      {4.5, 28},
      {4.75, 29},
      {5, 30},
      {5.25, 31}
    };

    public static final double[][] k_shotSpeeds = {
      {1.25, 2750},
      {1.5, 2750},
      {1.75, 2750},
      {2, 2750},
      {2.25, 2750},
      {2.5, 2750},
      {2.75, 2800},
      {3, 2900},
      {3.25, 3000},
      {3.5, 3100},
      {3.75, 3200},
      {4, 3300},
      {4.25, 3400},
      {4.5, 3530},
      {4.75, 3640},
      {5, 3750},
      {5.25, 3800}
    };

    public static final double k_hoodGearRatio = 34.6;
    public static final double k_hoodMomentOfInertia = 0.0642;
    public static final double k_hoodArmLengthMeters = 0.215;
    public static final double k_hoodTicksToDegrees = 35/3.46;
    
    public static final double k_shooterMomentOfInertia = 0.0003156;
    public static final double k_shooterMotorReduction = 1;
    public static final double k_rpmToSurfaceSpeedMperS = (0.0254*Math.PI)/60;

    public static final int k_hoodCurrentLimit = 60;
    public static final double k_hoodP = 0.035;
    public static final double k_hoodI = 0;
    public static final double k_hoodD = 0;
    public static final double k_hoodDeadzone = 1.5;

    public static final int k_shooterCurrent = 40;
    public static final double k_shooterKV = 0.00195;
    public static final double k_shooterP = 0.00005;
    public static final double k_shooterDeadzone = 100;

    public static final double k_staticHoodAngle = 35;//0;
    public static double k_hoodMaxAngle = 35;

    public static final double k_shooterRevVel = 0;

    public static final int k_maxFuelStorage = 45;

    public static final double k_staticShootVel = 2750;

    public static final SparkFlexConfig k_hoodConfig = new SparkFlexConfig();
    public static final SparkFlexConfig k_leaderConfig = new SparkFlexConfig();
    public static final SparkFlexConfig k_follower1Config = new SparkFlexConfig();
    public static final SparkFlexConfig k_leader2Config = new SparkFlexConfig();
    public static final SparkFlexConfig k_follower2Config = new SparkFlexConfig();

    static {
      k_hoodConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(k_hoodCurrentLimit).inverted(true)
      .encoder.positionConversionFactor(k_hoodTicksToDegrees);

      k_hoodConfig.closedLoop.pid(k_hoodP, k_hoodI, k_hoodD)
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

      k_leaderConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(k_shooterCurrent).inverted(true);

      k_leaderConfig.closedLoop.p(k_shooterP).feedbackSensor(FeedbackSensor.kPrimaryEncoder).feedForward.kV(k_shooterKV);
      k_leader2Config.apply(k_leaderConfig);

      k_follower1Config.apply(k_leaderConfig).follow(CANIDConstants.shooter_1, false);
      k_follower2Config.apply(k_leaderConfig).follow(CANIDConstants.shooter_3, false);
    }
  
  }

  public static class IndexerConstants{
    public static final double k_conveyorMOI = 0.000936;
    public static final double k_conveyorReduction = 1;

    public static final double k_kickerMOI = 0.000307;
    public static final double k_kickerReduction = 1;

    public static final double k_conveyorKV = 0.0035; // Needs to be changed since gear ration increased

    public static final double k_kickerKV = 0.00176; // Also needs to be changed

    public static final int k_conveyorCurrent = 40;

    public static final double k_normalConveyorInSpeed = 6000;
    public static final double k_normalConveyorOutSpeed = -4000;
    public static final double k_slowcConveyorInSpeed = 250;
    public static final double k_slowConveyorOutSpeed = -250;

    public static final int k_kickerCurrent = 40;
    public static final double k_kickerInSpeed = 6780;
    public static final double k_kickerOutSpeed = -6000;
    public static final double k_kickerPassiveOutSpeed = -500;

    public static final SparkFlexConfig k_conveyorConfig = new SparkFlexConfig();

    public static final SparkFlexConfig k_kickerConfig = new SparkFlexConfig();
    static {
      k_conveyorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(k_conveyorCurrent).inverted(true);

      k_kickerConfig.apply(k_conveyorConfig).inverted(false);

      k_conveyorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .feedForward.kV(k_conveyorKV);

      k_kickerConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .feedForward.kV(k_kickerKV);
    }
  }

  public static class ClimberConstants{
    public static final double k_climberWeight = 1.667;
    public static final double k_climberDrumWidth = 0.0127;
    public static final double k_climberReduction = 5;
    public static final double k_climberMaxHeight = 0.388;
    public static final double k_climberClimbingStowedHeight = 0.15;
    public static final double k_climberRotationsToMeters = 0.388/76;

    public static final double k_climberDeadzone = 0.05;

    public static final double k_manualClimbPower = 6; //arbitrary number

    public static final double k_climberP = 200;
    public static final double k_climberI = 0;
    public static final double k_climberD = 0;

    public static final int k_climberCurrent = 80;

    
    public static final SparkFlexConfig k_leftConfig = new SparkFlexConfig();
    public static final SparkFlexConfig k_rightConfig = new SparkFlexConfig();

    static {
      k_leftConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(k_climberCurrent).inverted(true);

      k_leftConfig.closedLoop.pid(k_climberP, k_climberI, k_climberD);

      k_leftConfig.encoder.positionConversionFactor(k_climberRotationsToMeters).velocityConversionFactor(k_climberRotationsToMeters / 60);

      k_rightConfig.apply(k_leftConfig);
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
    public static final int climber_right = 40;
    public static final int climber_left = 41;
  }
  public static class LightConstants {
    public static final int k_lightPort = 0; // may be different on actual robot
    public static final int k_lightAmount = 100;

    public static LinearVelocity k_disabledVelocity = MetersPerSecond.of(0.2);
    public static Distance k_ledSpacing = Meters.of(1 / 120.0);
    
  }
}