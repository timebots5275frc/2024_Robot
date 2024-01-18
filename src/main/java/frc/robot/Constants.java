package frc.robot;

import frc.robot.CustomTypes.PID_Values;
import frc.robot.CustomTypes.SwerveCanIDs;
import frc.robot.CustomTypes.SwerveModuleLocations;

  public final class Constants {
    public static class OperatorConstants {
      public static final int kDriverControllerPort = 0;
    }

    public static final class ControllerConstants {
      public static final int DRIVER_STICK_CHANNEL = 0;
      public static final int AUX_STICK_CHANNEL = 1;
      public static final int XBOXCONTROLLER_CHANNEL = 3;

      public static final double DEADZONE_DRIVE = 0.1;
      public static final double DEADZONE_STEER = 0.3;
    }

    public static final class DriveConstants {
      // Drivetrain Motor and Encoder IDs

      // #region <Robot 2023 Constants>
        public static final SwerveCanIDs Robot2023SwerveCAN = new SwerveCanIDs(
          1, // LEFT_FRONT_DRIVE_MOTOR_ID 
          2, // LEFT_FRONT_STEER_MOTOR_ID 
          3, // RIGHT_FRONT_DRIVE_MOTOR_ID 
          4, // RIGHT_FRONT_STEER_MOTOR_ID 
          5, // LEFT_REAR_DRIVE_MOTOR_ID 
          6, // LEFT_REAR_STEER_MOTOR_ID 
          7, // RIGHT_REAR_DRIVE_MOTOR_ID 
          8, // RIGHT_REAR_STEER_MOTOR_ID 
          10, // LEFT_FRONT_STEER_ENCODER_ID 
          11, // RIGHT_FRONT_STEER_ENCODER_ID 
          12, // LEFT_REAR_STEER_ENCODER_ID 
          13); // RIGHT_REAR_STEER_ENCODER_ID 

        public static final SwerveModuleLocations Robot2023SwerveLocations = new SwerveModuleLocations(
          (11 - 2.25) * 0.0254, // LEFT_FRONT_WHEEL_X
          (14 - 2.25) * 0.0254, // LEFT_FRONT_WHEEL_Y
          (11 - 2.25) * 0.0254, // RIGHT_FRONT_WHEEL_X
          (-14 + 2.25) * 0.0254, // RIGHT_FRONT_WHEEL_Y
          (-11 + 2.25) * 0.0254, // RIGHT_REAR_WHEEL_X
          (-14 + 2.25) * 0.0254, // RIGHT_REAR_WHEEL_Y
          (-11 + 2.25) * 0.0254, // LEFT_REAR_WHEEL_X
          (14 - 2.25) * 0.0254); //LEFT_REAR_WHEEL_Y
      // #endregion

      // #region <CaidBot Constants>
          public static final SwerveCanIDs CaidBotSwerveCAN = new SwerveCanIDs(
            1, // LEFT_FRONT_DRIVE_MOTOR_ID 
            2, // LEFT_FRONT_STEER_MOTOR_ID 
            5, // RIGHT_FRONT_DRIVE_MOTOR_ID 
            6, // RIGHT_FRONT_STEER_MOTOR_ID 
            3, // LEFT_REAR_DRIVE_MOTOR_ID 
            4, // LEFT_REAR_STEER_MOTOR_ID 
            7, // RIGHT_REAR_DRIVE_MOTOR_ID
            8, // RIGHT_REAR_STEER_MOTOR_ID
            10, // LEFT_FRONT_STEER_ENCODER_ID
            12, // RIGHT_FRONT_STEER_ENCODER_ID 
            11, // LEFT_REAR_STEER_ENCODER_ID
           13); // // RIGHT_REAR_STEER_ENCODER_ID

        public static final SwerveModuleLocations CaidBotSwerveLocations = new SwerveModuleLocations(
          11.75 * 0.0254, // LEFT_FRONT_WHEEL_X
          11.75 * 0.0254, // LEFT_FRONT_WHEEL_Y
          11.75 * 0.0254, // RIGHT_FRONT_WHEEL_X
          -11.75 * 0.0254, // RIGHT_FRONT_WHEEL_Y
          -11.75 * 0.0254, // RIGHT_REAR_WHEEL_X
          -11.75 * 0.0254, // RIGHT_REAR_WHEEL_Y
          -11.75 * 0.0254, // LEFT_REAR_WHEEL_X
          11.75 * 0.0254); //LEFT_REAR_WHEEL_Y
      // #endregion

      // CHANGE TO SET CURRENT ROBOT INFO //
      public static final SwerveCanIDs ROBOT_SWERVE_CAN = CaidBotSwerveCAN;
      public static final SwerveModuleLocations ROBOT_SWERVE_LOCATIONS = CaidBotSwerveLocations;
      //////////////////////////////////////

      public static final double WHEEL_RADIUS = 2.0 * 0.0254; // meters * 0.98
      public static final double WHEEL_CIRCUMFERENCE = 2.0 * Math.PI * WHEEL_RADIUS; // meters/revolution

      public static final double MAX_DRIVE_SPEED = 3.5; // meters/second
      public static final double MAX_STEER_RATE = .5; // rotations/second of a wheel for steer.
      public static final double MAX_TWIST_RATE = .6 * 2.0 * Math.PI; // radians/second of the robot rotation.

      // #region <Misc CAN IDs>
        public static final int PIGEON_IMU_ID = 9;
        public static final int PIGEON_2_ID = 9;
      // #endregion

      // #region <Gear Ratios>
      // Drive motor gear ratio.
      // | Driving Gear | Driven Gear |
      // First Stage | 14 | 50 |
      // Second Stage | 28 | 16 |
      // Third Stage | 15 | 60 |
      //
      // Overall Gear Ratio = 0.1225
      // One rotation of the motor gives 0.1225 rotations of the wheel.
      // 8.163 rotations of the motor gives one rotation of the wheel.
      public static final double DRIVE_GEAR_RATIO = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 60.0);

      // Steer motor gear ratio
      // | Driving Gear | Driven Gear |
      // First Stage | 15 | 32 |
      // Second Stage | 10 | 40 |
      //
      // Overall Gear Ration = 0.1171875
      // One rotation of the motor gives 0.1171875 rotations of the wheel.
      // 8.533 rotations of the motor gives one rotation of the wheel.
      public static final double STEER_GEAR_RATIO = (15.0 / 32) * (10 / 40);
      // #endregion

      // #region <PID Values>
      public static final PID_Values PID_SparkMax_Steer = new PID_Values(0.0001, 0, 0, 0, 0.00005);
      public static final PID_Values PID_Encoder_Steer = new PID_Values(20, 10, 0);
      public static final PID_Values PID_SparkMax_Drive = new PID_Values(0.0003, 0, 0, 0, 0.00016);
      // #endregion
  }

  public static final class VisionConstants {
    public static final int VALUES_TO_AVERAGE = 3;

    public static enum AprilTag
    {
      ba_source_left(1, "BA Source left"),
      ba_source_right(2, "BA Source right"),
      ra_speaker_aux(3, "RA Speaker auxillary"),
      ra_speaker_main(4, "RA Speaker main"),
      ra_amplifier(5, "RA Amplifier"),
      ba_amplifier(6, "BA Amplifier"),
      ba_speaker_main(7, "BA Speaker main"),
      ba_speaker_aux(8, "BA Speaker auxillary"),
      ra_source_right(9, "RA Source right"),
      ra_source_left(10, "RA Source left"),
      ra_core_scoring_table(11, "RA Core scoring table side"),
      ra_core_opp_scoring_table(12, "RA Core opposite scoring table side"),
      ra_core_mid(13, "RA Core middle side"),
      ba_core_mid(14, "BA Core middle side"),
      ba_core_opp_scoring_table(15, "BA Core opposite scoring table side"),
      ba_core_scoring_table(16, "BA Core scoring table side");

      int id;
      String name;
      private AprilTag(int id, String name)
      {
        this.id = id;
        this.name = name;
      }
    }
  }
}