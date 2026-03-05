package frc.robot;

public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double DEADBAND = 0.5;
  }

    public static int pigeon2ID = 13;

  public static class ShooterConstants {
    // SHOOTER
    public static int outRightID = 15;
    public static int outLeftID = 14;
    public static int feedID = 16;

    public static double shootSpeed = 0.5;
    public static double revTime = 1;
    public static double shootTime = 3;

    public static double autoShooterSpeed = 0.5;
  }
 
  public static class IntakeConstants{
    //INTAKE
    public static int intakeID = 17;
  }
    
  public static class WristConstants{
    //WRIST
    public static int leftWristID = 18;
    public static int rightWristID = 19;
  }
}