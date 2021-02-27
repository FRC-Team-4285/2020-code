/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4285.CamoSwerve.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import org.usfirst.frc4285.CamoSwerve.RobotMap;

public class ColorWheel extends Subsystem {
/*
 * Color Wheel Subsystem
 *
 * This system is responsible for spinning the color wheel and
 * identify the color currently being viewed.
 */

  private final CANSparkMax colormotor;
  
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_color_sensor = new ColorSensorV3(i2cPort);  
  private final ColorMatch m_color_matcher = new ColorMatch();

  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
  
  private Color detectedColor;


  public ColorWheel() {
    /*
     * Constructor method.
     */

    colormotor = new CANSparkMax(RobotMap.COLOR_WHEEL_MOTOR_ID, MotorType.kBrushless);

    m_color_matcher.addColorMatch(kBlueTarget);
    m_color_matcher.addColorMatch(kGreenTarget);
    m_color_matcher.addColorMatch(kRedTarget);
    m_color_matcher.addColorMatch(kYellowTarget);   
  }

  public void spin() {
    /*
     * Engage spin motors and detect colors while spinning.
     */

    detectedColor = m_color_sensor.getColor();
  
    String colorString;
    final ColorMatchResult match = m_color_matcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

    System.out.println(colorString + " (" + Math.round(match.confidence * 100) + "%)");

    colormotor.set(-1.0);
  }

  public void stop() {
    /*
     * Stop spin motor.
     */

    colormotor.set(0.0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
