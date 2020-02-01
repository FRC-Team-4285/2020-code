/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4285.CamoSwerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class RobotMap {
  public static AHRS navX; 

  public static final int LEFT_JOYSTICK = 0;
  public static final int RIGHT_JOYSTICK = 1;
  public static final int CONTROLLER = 2;

  public static final int DRIVE_LEFT_FRONT_ID = 3;
  public static final int DRIVE_LEFT_REAR_ID = 4;
  public static final int DRIVE_RIGHT_FRONT_ID = 2;
  public static final int DRIVE_RIGHT_REAR_ID = 1;

  public static final int STEER_LEFT_FRONT_ID = 7;
  public static final int STEER_LEFT_REAR_ID = 8;
  public static final int STEER_RIGHT_FRONT_ID = 6;
  public static final int STEER_RIGHT_REAR_ID = 5;

  // shooter can ids
  // none set to can id yet

  public static final int TURRET_ID = 0;
  public static final int THROWER_MOTOR_1_ID = 0;
  public static final int THROWER_MOTOR_2_ID = 0;

  public static final int Ball_Pickup_ID = 0;

  public static void init() {
    navX = new AHRS(SPI.Port.kMXP);
  }

}
