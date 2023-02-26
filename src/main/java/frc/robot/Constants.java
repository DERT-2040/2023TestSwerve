// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kFrontRightDriveMotorPort = 10;
    public static final int kFrontLeftDriveMotorPort = 12;
    public static final int kRearLeftDriveMotorPort = 14;
    public static final int kRearRightDriveMotorPort = 16;

    public static final int kFrontRightTurningMotorPort = 11;
    public static final int kFrontLeftTurningMotorPort = 13;
    public static final int kRearLeftTurningMotorPort = 15;
    public static final int kRearRightTurningMotorPort = 17;

    public static final int kFrontRightTurningEncoderPort = 0;
    public static final int kFrontLeftTurningEncoderPort = 1;
    public static final int kRearLeftTurningEncoderPort = 2;
    public static final int kRearRightTurningEncoderPort = 3;

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kRearLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kRearRightTurningEncoderReversed = true;

    /*public static final int kFrontLeftDriveEncoderPort = 4;
    public static final int kRearLeftDriveEncoderPort = 5;
    public static final int kFrontRightDriveEncoderPort = 6;
    public static final int kRearRightDriveEncoderPort = 7;*/

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kRearLeftDriveEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kRearRightDriveEncoderReversed = true;

    public static final double kTrackWidth = 0.37465;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.37465;
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = false;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    public static final double ksVolts = 1;
    public static final double kvVoltSecondsPerMeter = 0.8;
    public static final double kaVoltSecondsSquaredPerMeter = 0.15;

    public static final double kMaxSpeedMetersPerSecond = 3;
  }

  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

    public static final int kEncoderCPR = 42;                               // EWO NEO is 42 counts per rev
    public static final double kGearRatio = 8.14;
    public static final double kWheelDiameterMeters = 0.106;                // EWO 4" * 25.4 mm/inch  /  1000 mm/m  =0.106
    public static final double kDriveEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / ((double)kEncoderCPR * kGearRatio);

    public static final double kTurningEncoderDistancePerPulse = 
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
        // (2 * Math.PI) / (double) kEncoderCPR;  EWO commented out for next line
        (  2 * Math.PI / (double) 1 );                                      // EWO MK4i has 150/7 ratio  150 motor revs = 7 wheel station turns
                                                                            // EWO but we are measuring on the output only (encoder = 1 for each full rev)
 
    public static final double kPModuleTurningController = 1;

    public static final double kPModuleDriveController = 1;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class ControlIndexes {
    //Turntable Control
    public static final int gamePad1LeftTriggerIndex = 2;
    public static final int gamePad1RightTriggerIndex = 3;
    //Arm Rotate
    public static final int gamePad1RightStickYAxisIndex = 5;
    //Arm Extend
    public static final int gamePad1RightStickXAxisIndex = 4;
    //Nude Robot
    public static final int gamePad1LeftStickXAxis = 0;
    public static final int gamePad1LeftStickYAxis = 1;
  }
}
