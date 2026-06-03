// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.light;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.LightConstants;

public class LightSubsystem extends SubsystemBase {
  /** Creates a new LightSubsystem. */
  private AddressableLED m_led = new AddressableLED(LightConstants.k_lightPort);
  public AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(LightConstants.k_lightAmount);
  private LEDPattern m_pattern;
  private Timer m_timer = new Timer();

  private final double k_shift1Time = 10.0;
  private final double k_shift2Time = 35.0;
  private final double k_shift3Time = 60.0;
  private final double k_shift4Time = 85.0;
  private final double k_endShiftTime = 110.0;
  private final double k_matchEndTime = 140.0;

  private boolean m_autoWinOverride = false;
  private boolean m_overrideWon = false;

  private int m_shift = 0;
  private String gameData;
  private static final Time k_blinkMagnitude = Seconds.of(0.4);

  private boolean m_isOn = true;
  private boolean m_isScrolling = true;
  public final class LedPatterns {
      public static LEDPattern m_disabledPattern = LEDPattern.steps(Map.of(0, Color.kRed, 0.16, Color.kBlue, 0.33, new Color(1.0f, 0.35f, 0.0f),0.49, Color.kRed, 0.66, Color.kBlue, 0.83, new Color(1.0f, 0.35f, 0.0f))).scrollAtAbsoluteSpeed(LightConstants.k_disabledVelocity, LightConstants.k_ledSpacing);

      public static LEDPattern m_prideFlag = LEDPattern.steps(Map.of(0, new Color(0.357f, 0.808f, 0.980f), 0.1, new Color(0.961f, 0.663f, 0.722f), 0.2, Color.kWhite, 0.3, new Color(0.961f, 0.663f, 0.722f), 0.4, new Color(0.357f, 0.808f, 0.980f), 0.6, new Color(0.961f, 0.663f, 0.722f), 0.7, Color.kWhite, 0.8, new Color(0.961f, 0.663f, 0.722f), 0.9, new Color(0.357f, 0.808f, 0.980f)));
    }  

  private final class ShiftCommand extends Command {

    private final int m_commandShift;
    private final boolean m_odd;
    private final double m_endTime;
    private Color m_color = Color.kBlack;
    
    public ShiftCommand(boolean odd, double endTime, int shift) {
      m_commandShift = shift;
      m_odd = odd;
      m_endTime = endTime;
      System.out.println(String.format("ShiftCommand (%b, %f, %d)", odd, endTime, shift));
      if (shift != 5){
        new Trigger(() -> m_timer.get() > m_endTime - 3.0).onTrue(new SetPattern(Color.kBlue, true));
      }
    }
    
    @Override
    public void initialize() {
      System.out.println("initialize");
      if (m_commandShift == 0){
        m_timer.reset();
      }
      m_timer.start();
      m_color = getColor();
      // if(getColor() == Color.kPurple){
      //   m_pattern = LedPatterns.m_prideFlag;
      // } else {
        applyPattern(m_color, false);
      // }  
    }
    
    public Color getColor(){
      Color color;
      if ((m_commandShift != 0 && m_commandShift != 5) && gameData != null){
        if (m_odd) {
          color = getWonAuto() ? Color.kRed : Color.kGreen;
        }
        else {
          color = getWonAuto() ? Color.kGreen : Color.kRed;
        }
      }
      else{
        color = Color.kPurple;
      }
      return color;
    }
    @Override
    public void end(boolean interrupted) {
        m_shift ++;
    }
    public void putShiftTime(double shift) {
      SmartDashboard.putNumber("Time Until Shift", round(shift - m_timer.get()));
    }
    public double round(double number) {
      double biigerNumber = Math.floor(number * 10);
      return (biigerNumber) / 10;
    }
    @Override
    public boolean isFinished() {
      switch(m_commandShift) {
        case 0:
          putShiftTime(k_shift1Time);
          return m_timer.get() > k_shift1Time;
          
        case 1:
          putShiftTime(k_shift2Time);
          return m_timer.get() > k_shift2Time;

        case 2:
          putShiftTime(k_shift3Time);
          return m_timer.get() > k_shift3Time;

        case 3:
          putShiftTime(k_shift4Time);
          return m_timer.get() > k_shift4Time;

        case 4:
          putShiftTime(k_endShiftTime);
          return m_timer.get() > k_endShiftTime;

        case 5:
          putShiftTime(k_matchEndTime);
          return m_timer.get() > k_matchEndTime;

        default:
          return true;

      }
    }
  }

  public class SetPattern extends Command {
    private Color m_color;
    private boolean m_blinking;
    public SetPattern(Color color, boolean blinking) {
      m_color = color;
      m_blinking = blinking;
    }
    @Override
    public void initialize() {
      applyPattern(m_color, m_blinking);
    }
  }
  public void applyPattern(Color color, boolean blinking) {
    m_pattern = LEDPattern.solid(color);
      if (blinking) {
        m_pattern = m_pattern.blink(k_blinkMagnitude);
        System.out.println("Setting Pattern to Blinking");
      }
      System.out.println("applying pattern to LED");
      m_pattern.applyTo(m_ledBuffer);
  }
  public LightSubsystem() {
    SmartDashboard.putData(Commands.runOnce(() -> {
        m_isOn = !m_isOn;
      })
      .withName("Toggle Lights")
      .ignoringDisable(true));

      SmartDashboard.putData(Commands.runOnce(() -> {
        m_isScrolling = !m_isScrolling;
        LedPatterns.m_disabledPattern = LEDPattern.steps(Map.of(0, Color.kRed, 0.16, Color.kBlue, 0.33, new Color(1.0f, 0.35f, 0.0f),0.49, Color.kRed, 0.66, Color.kBlue, 0.83, new Color(1.0f, 0.35f, 0.0f))).scrollAtAbsoluteSpeed(m_isScrolling ? LightConstants.k_disabledVelocity : MetersPerSecond.of(0), LightConstants.k_ledSpacing);
      })
      .withName("Toggle Disable Scrolling")
      .ignoringDisable(true));
    m_led.setLength(LightConstants.k_lightAmount);
    m_led.setData(m_ledBuffer);
    m_led.start();
    // new Trigger(() -> DriverStation.isEnabled()).onTrue();
    new Trigger(() -> DriverStation.isTeleop() && !DriverStation.isDisabled()).onTrue(new ShiftCommand(false, k_shift1Time, 0));
    new Trigger(() -> m_timer.get() > k_shift1Time).onTrue(new ShiftCommand(true, k_shift2Time, 1));
    new Trigger(() -> m_timer.get() > k_shift2Time).onTrue(new ShiftCommand(false, k_shift3Time, 2));
    new Trigger(() -> m_timer.get() > k_shift3Time).onTrue(new ShiftCommand(true, k_shift4Time, 3));
    new Trigger(() -> m_timer.get() > k_shift4Time).onTrue(new ShiftCommand(false, k_endShiftTime, 4));
    new Trigger(() -> m_timer.get() > k_endShiftTime).onTrue(new ShiftCommand(false, k_matchEndTime, 5));
  }

  @Override
  public void periodic() {
    if (m_isOn){
      gameData = DriverStation.getGameSpecificMessage();
      if (DriverStation.isDisabled()){
        m_pattern = LedPatterns.m_disabledPattern;
      }
      if (m_pattern != null){
        m_pattern.applyTo(m_ledBuffer);
      }
    }else{
      m_pattern = LEDPattern.solid(Color.kBlack);
      m_pattern.applyTo(m_ledBuffer);
    }
    
    m_led.setData(m_ledBuffer);

    SmartDashboard.putBoolean("Won Auto?", getWonAuto());
  }
  /**
   * Using the current shift, estimated through a timer, It will return if the alliance hub is currently active.
   * @return Boolean returning the current active hub
   */
  public boolean hubIsActive(){
    switch (m_shift) {
      case 0:
        return true;
      case 1:
        return getWonAuto() ? false : true;
      case 2:
        return getWonAuto() ? true : false;
      case 3:
        return getWonAuto() ? false : true;
      case 4:
        return getWonAuto() ? true : false;
      case 5:
        return true;
      default:
        return false;
    }
  }
  /**
   * It gets the game specific message and using that determines what side won auto.
   * Note: This is untested since we don't have a driver station for testing.
   * @return Boolean returning whether your current alliance won Auto
   */
  public boolean getWonAuto() {
    if (m_autoWinOverride){
      return m_overrideWon;
    }
    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
        case 'B':
          if (DriverStation.getAlliance().get() == Alliance.Blue) {
            return true;
          }
          else {
            return false;
          }
        case 'R':
          if (DriverStation.getAlliance().get() == Alliance.Red) {
            return true;
          }
          else {
            return false;
          }
        default:
          return true;
      }
    }
    else {
      return false;
    }
  }

  public Command overrideWonAuto(boolean wonAuto){
    return Commands.runOnce(() -> {
      m_autoWinOverride = true;
      m_overrideWon = wonAuto;
    });
  }
}
