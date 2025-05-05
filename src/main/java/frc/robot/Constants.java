// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class PIDConstants {
    public static final double Kp = 0.05;
    public static final double Ki = 0;
    public static final double Kd = 0.002;

    public static class PIDConstraints {
      public static double kMaxVelocity = 1.7;
      public static double kMaxAcceleration = 0.75;
    }
  }

  public static class ElevatorConstants {
    public static double kDrumRadius = 1;
    public static final double kElevatorGearing = 12.0;

    public static double kElevatorkS = 0;
    public static double kElevatorkG = 0;
    public static double kElevatorkV = 0;
    public static double kElevatorkA = 0;

  }
}
