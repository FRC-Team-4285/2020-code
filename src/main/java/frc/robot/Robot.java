/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.util.Color;

import com.ctre.phoenix.motorcontrol.ControlMode;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;

import com.playingwithfusion.TimeOfFlight;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_color_sensor = new ColorSensorV3(i2cPort);  
  private final ColorMatch m_color_matcher = new ColorMatch();

  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  Joystick stick = new Joystick(0);
  Joystick stick2 = new Joystick(1);
  Timer RobotTimer = new Timer();  
  TimeOfFlight fly = new TimeOfFlight(10);
  CANSparkMax Motor0 = new CANSparkMax(8, MotorType.kBrushless);
  CANSparkMax Motor1 = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax Motor2 = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax Motor3 = new CANSparkMax(3, MotorType.kBrushless);

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");


  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    Motor2.setInverted(true);
    Motor3.setInverted(true);

    m_color_matcher.addColorMatch(kBlueTarget);
    m_color_matcher.addColorMatch(kGreenTarget);
    m_color_matcher.addColorMatch(kRedTarget);
    m_color_matcher.addColorMatch(kYellowTarget);    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    final double x = tx.getDouble(0.0);
    final double y = ty.getDouble(0.0);
    final double area = ta.getDouble(0.0);

    final Color detectedColor = m_color_sensor.getColor();

    /**
     * Run the color match algorithm on our detected color
     */
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

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // autoSelected = SmartDashboard.getString("Auto Selector",
    // defaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
    case kCustomAuto:
      // Put custom auto code here
      break;
    case kDefaultAuto:
    default:
      // Put default auto code here
      break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    // Time of flight sensor.
    double range = fly.getRange();

    // If the range is invalid, we will ignore
    // the data because the information is useless.
    if (fly.isRangeValid()) {
      // Range can be used within this check.
      System.out.println(range);
    }
    
    /////////////////////////////////
    //        JOYSTICK CODE        //
    /////////////////////////////////

    // Left Motors
    if(stick.getRawAxis(1) > 0.1 || stick.getRawAxis(1) < -0.1)
    {
      // We are receiving joystick input, set the motor speed to the
      // current ranged value of the joystick axis. This gives the robot's
      // driver a variable "gas pedal" of sorts such that if you have
      // the joystick in half position, the motors will be half power.
      Motor0.set(stick.getRawAxis(1));
      Motor1.set(stick.getRawAxis(1));
    }
    else
    {
      // We are not receiving joystick input, so stop the motors.
      Motor0.set(0);
      Motor1.set(0);
    }

    // Right Motors
    if(stick2.getRawAxis(1) > 0.1 || stick2.getRawAxis(1) < -0.1) {
      // We are receiving joystick input, set the motor speed to the
      // current ranged value of the joystick axis. This gives the robot's
      // driver a variable "gas pedal" of sorts such that if you have
      // the joystick in half position, the motors will be half power.
      Motor2.set(1 * stick2.getRawAxis(1));
      Motor3.set(1 * stick2.getRawAxis(1));
    }
    if(stick2.getRawAxis(1) < 0.1 && stick2.getRawAxis(1) > -0.1){
      // We are not receiving joystick input, so stop the motors.
      Motor2.set(0);
      Motor3.set(0);
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
