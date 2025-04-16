package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class LimelightConstants{
        public static final Pose2d TAG_6_L_POSE2D = new Pose2d(13.592, 2.816, Rotation2d.fromDegrees(120));
        public static final Pose2d TAG_6_R_POSE2D = new Pose2d(13.592, 2.816, Rotation2d.fromDegrees(120));
        public static final Pose2d TAG_17_L_POSE2D = new Pose2d(3.978, 5.253, Rotation2d.fromDegrees(-60));
        public static final Pose2d TAG_17_R_POSE2D = new Pose2d(3.705, 5.078, Rotation2d.fromDegrees(-60));


        public static final Matrix<N3,N1> VISION_STD_DEV = VecBuilder.fill(1, 1, 9999);
        public static final Matrix<N3,N1> STATE_STD_DEV = VecBuilder.fill(0.1, 0.1, 0.0001);

        public static final double translationStdDevCoefficient = 5;
        public static final double rotationStdDevCoefficient = 0.9;
    }


    public static final class ArmConstants{
        public static final int ARM_LEAD_MOTR_ID = 41;
        public static final int ARM_FOLLOW_MOTOR_ID = 42;

        //Infeeds
        public static final double COMP = 0.428;

        public static final double CORAL_INFEED = 0.345;//.345 before 3/8/25 //0.347 // 3/8/25 3.49 **good 0.345**
        public static final double AUTO_CORAL_INFEED = 0.348;
        // public static final double CORAL_SOURCE_INFEED = 0.533;
        public static final double CORAL_SOURCE_INFEED = 0.527;


        public static final double ALGAE_INFEED = 0.425;//.425
        public static final double ALGAE_INFEED_L1_Front = .53;
        public static final double ALGAE_INFEED_L1 = .586;
        public static final double ALGAE_INFEED_L2_Front = .531;//.531
        public static final double ALGAE_INFEED_L2 = .586;
        public static final double ALGAE_INFEED_L2_COMP = .46;

        //Reef
        public static final double L1 = 0.58;
        // public static final double L1_Front = 0.422;
        public static final double L1_Front = 0.40;

        public static final double L2 = 0.571;
        public static final double L2_Front = 0.447;

        public static final double L3 = 0.5831;
        public static final double L3_Front = 0.497;

        // public static final double L4 = 0.583;
        public static final double L4 = 0.58;//.583 //0.577
        public static final double L4_Front = 0.535;//.53
        // public static final double L4_Front = 0.539;

        //Barge
        public static final double BARGE = 0.59;

        //Processor
        public static final double PREPROCESSOR = 0.427;
        public static final double PROCESSOR = 0.425; 

        public static final double PID_TEST_POINT = .5;
        public static final double PID_TEST_POINT2 = .37;



        // THIS ONE CARSON!!!!
        public static final double CLIMB_READY = 0.518; //0.530 


        public static final double CLIMBED = 0.43; 


        public static final int ARM_CANCONDER_ID = 4;

        public static final double MAX_PID_OUTPUT = 1;
        public static final double ALGAE_INFEED_PID_OUTPUT = 0.5;
        public static final double ALGAE_BARGE_PID_OUTPUT = .75;
        // public static final double MAX_PID_OUTPUT = 0.2;

    }

    public static final class WristConstants{
        public static final int WRIST_MOTOR_ID = 60;
        public static final int WRIST_CANCODER_ID = 6; 

        //Comp
        public static final double COMP = 0.776;
        public static final double ALGAE_INFEED_L1_COMP = .58;

        //Infeed
        public static final double CORAL_INFEED = 0.43; // 0.415 //0.42 *good one 0.43*
        // public static final double CORAL_SOURCE_INFEED = 0.35;
        public static final double CORAL_SOURCE_INFEED = 0.343;//.336


        public static final double ALGAE_INFEED = 0.285;//.4
        public static final double ALGAE_INFEED_L1 = 0.73;//0.72
        public static final double ALGAE_INFEED_L1_Front = 0.215;
        public static final double ALGAE_INFEED_L2 = .72;
        public static final double ALGAE_INFEED_L2_Front = .23;

        //Reef
        public static final double L1 = 0.78;
        // public static final double L1_Front = 0.426;
        public static final double L1_Front = 0.779;

        public static final double L2 = 0.773;
        public static final double L2_Front = 0.72;

        public static final double L3 = 0.773;
        public static final double L3_Front = 0.666;

        public static final double L4 = 0.79;//.77
        public static final double L4_FLIP_BACK = 0.6;
        public static final double L4_Front = 0.558;//.581
        // public static final double L4_Front = 0.569;

        //Barge
        public static final double BARGE = 0.495;// 0.48 .5

        //Processor
        public static final double PROCESSOR = 0.38; 

        public static final double PID_TEST_POINT = 0.7;
        public static final double PID_TEST_POINT2 = 0.46;

        public static final double MAX_PID_OUTPUT = 0.5;
        public static final double ALGAE_INFEED_PID_OUTPUT = 0.25;
        public static final double BARGE_PID_OUTPUT = 0.15;
        public static final double PROCESSOR_PID_OUTPUT = 0.15;
        // public static final double MAX_PID_OUTPUT = 0.2;
    }

    public static final class ElevatorConstants{
        public static final int Elevator_Lead_Motor_ID = 51;
        
        public static final double COMP = 0.1;
        // public static final double CORAL_SOURCE_INFEED = 1.48;
        public static final double CORAL_SOURCE_INFEED = 2.55;//2.8

        //Infeed
        public static final double ALGAE_INFEED_Ground = 0.863;
        public static final double ALGAE_INFEED_L1 = 1.25;//1.55
        public static final double ALGAE_INFEED_L2 = 5.5;//5.6
        public static final double ALGAE_INFEED_L1_Front = 4.425;//4.5 high 4.3 low
        public static final double AUTO_ALGAE_INFEED_L1_Front = 4.3;//4.5 high 4.3 low
        public static final double ALGAE_INFEED_L2_Front = 8.4;//8.3
        public static final double AUTO_ALGAE_INFEED_L2_Front = 8.1;//8.3

        //Reef
        public static final double L1 = 0.1;
        public static final double L1_Front = 0.5;

        public static final double L2 = 0.15; //0.25;
        public static final double L2_Front = 1.26;

        public static final double L3 = 4.45; //0.45
        public static final double L3_Front = 4.65;

        public static final double L4 = 10.55; // 10.55
        public static final double L4_Front = 10.4; // 10.4S

        //Barge
        public static final double BARGE = 10.4;

        //Processor
        public static final double PROCESSOR = 0.5;
        
        public static final double ELEVATOR_TEST_POINT = 9;
        public static final double ELEVATOR_TEST_POINT_2 = 3;
        
        // public static final double MAX_PID_OUTPUT = .2; 
        // public static final double MAX_PID_OUTPUT = 1; 
        public static final double MAX_PID_OUTPUT = .95; 
        public static final double Elevator_Test_PID_OUTPUT = .85;
    }

    public static final class ClimberConstants {
    
        public static final double MANUAL_SPEED = 1;
        
    }

    public static final class InfeedConstants{
        public static final int Infeed_Left_Motor_ID = 61;
        public static final int Infeed_Right_Motor_ID = 62;

        public static final double ALGAE_INFEED = 1;
        public static final double LEFT_CORAL_INFEED = 1;
        public static final double RIGHT_CORAL_INFEED = 0.6;  //0.6

        public static final double CORAL_SOURCE_INFEED = .25; //0.5

        public static final double IDLE_ALGAE_VOLTAGE = 1;

        public static final double CORAL_SHOT = 0.65;
        public static final double L3_SHOT = 0.4;
        public static final double PROCESSOR_SHOT = 0.15;
        public static final double BARGE_SHOT = .2; //0.1

        public static final String ALGAE_INFEED_GROUND = "Ground Infeed";
        public static final String ALGAE_INFEED_L1 = "L1 Infeed";
        public static final String ALGAE_INFEED_L2 = "L2 Infeed";


    }

    public static final class AlignmentConstants{
        // public static final PathConstraints PATHFINDING_CONSTRAINTS = new PathConstraints(4, 10, 10, 10);
    }

    public static final class Swerve {
        public static final int pigeonID = 5;

        public static final COTSTalonFXSwerveConstants chosenModule = 
        COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(22.75);
        public static final double wheelBase = Units.inchesToMeters(22.75);
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));


        public static final Translation2d mod0Offset = new Translation2d(wheelBase / 2.0, trackWidth / 2.0);
        public static final Translation2d mod1Offset = new Translation2d(wheelBase / 2.0, -trackWidth / 2.0);
        public static final Translation2d mod2Offset = new Translation2d(-wheelBase / 2.0, trackWidth / 2.0);
        public static final Translation2d mod3Offset = new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0);

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 60;
        public static final int driveCurrentThreshold = 60;
        // public static final double driveCurrentThresholdTime = 0.1;
        public static final double driveCurrentThresholdTime = 1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; //TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.15; 
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Brake;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 0;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-26.455); //26.279
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 12;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-129.462); //-130.253
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 21;
            public static final int angleMotorID = 22;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(13.886); //14.238
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 31;
            public static final int angleMotorID = 32;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-156.972);//-60.029, -146.777
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

}
