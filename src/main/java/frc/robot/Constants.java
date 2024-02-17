package frc.robot;

import javax.xml.namespace.QName;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.CustomTypes.PID_Values;
import frc.robot.CustomTypes.SwerveCanIDs;
import frc.robot.CustomTypes.SwerveModuleLocations;
import frc.robot.CustomTypes.Math.Vector2;
import frc.robot.CustomTypes.Math.Vector3;

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

      // #region <Robot 2024 Constants>
      public static final SwerveCanIDs Robot2024Can = new SwerveCanIDs(
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

      public static final SwerveModuleLocations Robot2024SwerveLocations = new SwerveModuleLocations(
          ((29 / 2) - 3.25) * MathConstants.INCH_TO_METER, // LEFT_FRONT_WHEEL_X
          ((30 / 2) - 3.25) * MathConstants.INCH_TO_METER, // LEFT_FRONT_WHEEL_Y
          ((29 / 2) - 3.25) * MathConstants.INCH_TO_METER, // RIGHT_FRONT_WHEEL_X
          (-(30 / 2) + 3.25) * MathConstants.INCH_TO_METER, // RIGHT_FRONT_WHEEL_Y
          (-(29 / 2) + 3.25) * MathConstants.INCH_TO_METER, // RIGHT_REAR_WHEEL_X
          (-(30 / 2) + 3.25) * MathConstants.INCH_TO_METER, // RIGHT_REAR_WHEEL_Y
          (-(29 / 2) + 3.25) * MathConstants.INCH_TO_METER, // LEFT_REAR_WHEEL_X
          ((30 / 2) - 3.25) * MathConstants.INCH_TO_METER); //LEFT_REAR_WHEEL_Y
      // #endregion

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
          (11 - 2.25) * MathConstants.INCH_TO_METER, // LEFT_FRONT_WHEEL_X
          (14 - 2.25) * MathConstants.INCH_TO_METER, // LEFT_FRONT_WHEEL_Y
          (11 - 2.25) * MathConstants.INCH_TO_METER, // RIGHT_FRONT_WHEEL_X
          (-14 + 2.25) * MathConstants.INCH_TO_METER, // RIGHT_FRONT_WHEEL_Y
          (-11 + 2.25) * MathConstants.INCH_TO_METER, // RIGHT_REAR_WHEEL_X
          (-14 + 2.25) * MathConstants.INCH_TO_METER, // RIGHT_REAR_WHEEL_Y
          (-11 + 2.25) * MathConstants.INCH_TO_METER, // LEFT_REAR_WHEEL_X
          (14 - 2.25) * MathConstants.INCH_TO_METER); //LEFT_REAR_WHEEL_Y
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
          11.75 * MathConstants.INCH_TO_METER, // LEFT_FRONT_WHEEL_X
          11.75 * MathConstants.INCH_TO_METER, // LEFT_FRONT_WHEEL_Y
          11.75 * MathConstants.INCH_TO_METER, // RIGHT_FRONT_WHEEL_X
          -11.75 * MathConstants.INCH_TO_METER, // RIGHT_FRONT_WHEEL_Y
          -11.75 * MathConstants.INCH_TO_METER, // RIGHT_REAR_WHEEL_X
          -11.75 * MathConstants.INCH_TO_METER, // RIGHT_REAR_WHEEL_Y
          -11.75 * MathConstants.INCH_TO_METER, // LEFT_REAR_WHEEL_X
          11.75 * MathConstants.INCH_TO_METER); //LEFT_REAR_WHEEL_Y
      // #endregion

      // CHANGE TO SET CURRENT ROBOT INFO //
      public static final SwerveCanIDs ROBOT_SWERVE_CAN = Robot2024Can;
      public static final SwerveModuleLocations ROBOT_SWERVE_LOCATIONS = Robot2024SwerveLocations;
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

      public static final double AUTO_ODOMETRY_DRIVE_MIN_SPEED = .1;
      public static final double AUTO_ODOMETRY_DRIVE_MAX_SPEED = 2;
      public static final double AUTO_ODOMETRY_DRIVE_TARGET_ALLOWED_ERROR = .1; // in meters
      public static final double AUTO_ODOMETRY_DRIVE_SLOWDOWN_DISTANCE = .6; // in meters
  }

  public static final class VisionConstants {
    public static final boolean ENABLE_LIMELIGHT_LIGHT_ON_ENABLE = true;
    public static final int VALUES_TO_AVERAGE = 3;
    public static final double TARGET_POSITION_ALLOWED_ERROR = .1;

    public static final double MAX_AMP_TARGET_DISTANCE = 3;
    public static final Vector2 AMP_VISION_DRIVE_TARGET = new Vector2(0, .5);

    public static enum AprilTagData
    {
      ba_source_left(1, "Source left", DriverStation.Alliance.Blue),
      ba_source_right(2, "Source right", DriverStation.Alliance.Blue),
      ra_speaker_aux(3, "Speaker auxillary", DriverStation.Alliance.Red),
      ra_speaker_main(4, "Speaker main", DriverStation.Alliance.Red),
      ra_amplifier(5, "Amplifier", DriverStation.Alliance.Red),
      ba_amplifier(6, "Amplifier", DriverStation.Alliance.Blue),
      ba_speaker_main(7, "Speaker main", DriverStation.Alliance.Blue),
      ba_speaker_aux(8, "Speaker auxillary", DriverStation.Alliance.Blue),
      ra_source_right(9, "Source right", DriverStation.Alliance.Red),
      ra_source_left(10, "Source left", DriverStation.Alliance.Red),
      ra_core_scoring_table(11, "Core scoring table side", DriverStation.Alliance.Red),
      ra_core_opp_scoring_table(12, "Core opposite scoring table side", DriverStation.Alliance.Red),
      ra_core_mid(13, "Core middle side", DriverStation.Alliance.Red),
      ba_core_mid(14, "Core middle side", DriverStation.Alliance.Blue),
      ba_core_opp_scoring_table(15, "Core opposite scoring table side", DriverStation.Alliance.Blue),
      ba_core_scoring_table(16, "Core scoring table side", DriverStation.Alliance.Blue);

      public final int id;
      public final String name;
      public final DriverStation.Alliance alliance;
      private AprilTagData(int id, String name, DriverStation.Alliance alliance)
      {
        this.id = id;
        this.name = name;
        this.alliance = alliance;
      }

      @Override
      public String toString()
      {
        return (alliance == DriverStation.Alliance.Blue ? "Blue " : "Red ") + name + "[" + id + "]";
      }

      public static AprilTagData getTag(int id) {
        switch(id){
          case 1: return ba_source_left;
          case 2: return ba_source_right;
          case 3: return ra_speaker_aux;
          case 4: return ra_speaker_main;
          case 5: return ra_amplifier;
          case 6: return ba_amplifier;
          case 7: return ba_speaker_main;
          case 8: return ba_speaker_aux;
          case 9: return ra_source_right;
          case 10: return ra_source_left;
          case 11: return ra_core_scoring_table;
          case 12: return ra_core_opp_scoring_table;
          case 13: return ra_core_mid;
          case 14: return ba_core_mid;
          case 15: return ba_core_opp_scoring_table;
          case 16: return ba_core_scoring_table;
          default: return null;
        }
      }

      public static boolean isSpeakerTag(AprilTagData tag) { return tag == AprilTagData.ra_speaker_main || tag == AprilTagData.ba_speaker_main; }
      public static boolean isAmpTag(AprilTagData tag) { return tag == AprilTagData.ra_amplifier || tag == AprilTagData.ba_amplifier; }
    }
  }

  public static class ClimberConstants {
    public static final int CLIMBER_MOTOR_L_ID  = 40;
    public static final int CLIMBER_MOTOR_R_ID  = 41;
    public static final double CLIMBER_MAX_POS = 230 * 2.25;
    public static final double CLIMBER_SPEED = 9000;
    public static final PID_Values ClimberMotorPIDs = new PID_Values(
      0.00016, 
    0.0,
    0.0,
    0.0,
    0.0
    );
  }
  public static class ShooterConstants {
    public static final PID_Values ShooterRunPIDs = new PID_Values(
    0.00005,
    0.0,
    0.0,
    0.0,
    0.0002);

    public static final PID_Values ShooterPivotPIDs = new PID_Values(
    0.00003,
    0.00000003,
    0.000064, 
    0.0,
    0.0001);
    
    public static final double SHOOTER_PIVOT_MAX_VEL = 1035 * 1.5;
    public static final double SHOOTER_PIVOT_MAX_ACCEL = SHOOTER_PIVOT_MAX_VEL * 2;

    public static final double SHOOTER_PIVOT_GEAR_RATIO = 621;
    public static final double SHOOTER_PIVOT_ROTATIONS_PER_DEGREE = SHOOTER_PIVOT_GEAR_RATIO / 360.0;


    public final static int LEFT_SHOOTER_ID = 32;
    public final static int RIGHT_SHOOTER_ID = 31;
    public final static int PIVOT_SHOOTER_ID = 30;
    public final static int PIVOT_SHOOTER_CANCODER_ID = 33;

    public final static double LEFT_SHOOTER_SPEED = 4000;
    public final static double RIGHT_SHOOTER_SPEED = 0.5 * LEFT_SHOOTER_SPEED;

    public final static double LEFT_AMP_SPEED = 731;
    public final static double RIGHT_AMP_SPEED = LEFT_AMP_SPEED;

    public final static double SHOOTER_RESET_POS = 65 * SHOOTER_PIVOT_ROTATIONS_PER_DEGREE;
    public final static double SHOOTER_START_POS = 65 * SHOOTER_PIVOT_ROTATIONS_PER_DEGREE;
    public final static double SHOOTER_DEFAULT_SHOOTING_POS = 58 * SHOOTER_PIVOT_ROTATIONS_PER_DEGREE;
    public final static double SHOOTER_DEFAULT_AMP_POS = 56 * SHOOTER_PIVOT_ROTATIONS_PER_DEGREE;
    public final static double SHOOTER_NONE_POS = 55 * SHOOTER_PIVOT_ROTATIONS_PER_DEGREE;
    public final static double SHOOTER_STUPID_POS = 78 * SHOOTER_PIVOT_ROTATIONS_PER_DEGREE;

    public final static double SHOOTER_PIVOT_ALLOWED_OFFSET = 1;
    public final static double LEFT_SHOOTER_ALLOWED_DIFFERENTIAL = 0;
    public final static double RIGHT_SHOOTER_ALLOWED_DIFFERENTIAL = 0;

    public final static double SPEAKER_MIN_SHOT_DISTANCE = .8; // Meters
    public final static double SPEAKER_MAX_SHOT_DISTANCE = 5; // Meters

    public final static double SHOOTER_TARGET_HEIGHT = 80; /*inches */
    public final static double LIMELIGHT_HEIGHT_WITH_OFFSET = 0;
    public final static double SHOOTER_LENGTH = 0;

    public final static double SHOOTER_OFFSET_FROM_GROUND = 0;

    public final static double SHOOTER_MAX_INTAKE_ANGLE = 59 * SHOOTER_PIVOT_ROTATIONS_PER_DEGREE;
  }

  public static class IntakeConstants {
    public static final PID_Values IntakeRunPIDs = new PID_Values(
    0.0, 
    0.0,
    0.0,
    0.0,
    0.003);

    public static final PID_Values IntakePivotPIDs = new PID_Values(
    0.000085, 
    0.0,
    0.001,
    0.0,
    0.000162);
    public static final double INTAKE_PIVOT_MAX_VEL = 2500;
    public static final double INTAKE_PIVOT_MAX_ACCEL = 4000;
    public static final double INTAKE_PIVOT_MIN_VEL = 5;
    // Spark Max Motor ID's5
    public static final int INTAKE_RUN_MOTOR_ID = 21;
    public static final int INTAKE_PIVOT_MOTOR_ID = 20;
    public static final int INTAKE_ANGLE_ENCODER_ID = 22;
    public static final double INTAKE_PIVOT_GEAR_RATIO = 96;
    public static final double INTAKE_PIVOT_ROTATIONS_PER_DEGREE = INTAKE_PIVOT_GEAR_RATIO / 360;
    // Positions of intake in motor rotations

    public static final double INTAKE_IN_POS = 224.5 * INTAKE_PIVOT_ROTATIONS_PER_DEGREE;
    public static final double INTAKE_OUT_POS = 5.4 * INTAKE_PIVOT_ROTATIONS_PER_DEGREE;
    public static final double INTAKE_UP_POS = 32 * INTAKE_PIVOT_ROTATIONS_PER_DEGREE;
    public static final double INTAKE_PIVOT_ALLOWED_OFFSET = 5 * INTAKE_PIVOT_ROTATIONS_PER_DEGREE;

    public static final double INTAKE_RUN_SPEED = 400;

  }

  public static final class MathConstants
  {
    public static final double INCH_TO_METER = 0.0254;
  }
}