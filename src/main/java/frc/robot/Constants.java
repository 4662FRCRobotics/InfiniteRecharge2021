/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class Common {
        public static final int kPCM_PORT = 1;
    }

    public static final class ButtonMappings {

        // joystick buttons
        public static final int kSHOOTER = 1;
        public static final int kLOADER = 2;
        public static final int kCLOSELOADER = 3;
        public static final int kLOADERSPIT = 5;
        public static final int kCLIMB_UP = 6;
        public static final int kCLIMB_DOWN = 4;
        public static final int kVISION_ON = 8;
        public static final int kVISION_DOWN = 7;
        public static final int kHARVESTER_REVERSE = 10;

        // console buttons
        public static final int kCLIMB_SWITCH = 1;
    }

    public static final class DriveConstants {
        public static final int kLeftMotor1Port = 2;
        public static final int kLeftMotor2Port = 3;
        public static final int kRightMotor1Port = 4;
        public static final int kRightMotor2Port = 5;
        public static final boolean kIS_DRIVE_INVERTED = false;

        public static final double kRAMP_RATE = 1.0;
        public static final int kCURRENT_LIMT = 40;

        public static final double kGEARBOX_REDUCTION = (50.0/12.0) * (60.0/14.0);
        public static final double kTIRE_SIZE_IN = 7.9;
        public static final double kTIRE_SIZE_M = Units.inchesToMeters(kTIRE_SIZE_IN);
        public static final int kPULSE_PER_ROTATION = 1;
        public static final double kENCODER_DISTANCE_PER_PULSE_M = ((double) kPULSE_PER_ROTATION / kGEARBOX_REDUCTION) * (kTIRE_SIZE_M * Math.PI);
        public static final double kTRACK_WIDTH_M = 0.64;

        public static final DifferentialDriveKinematics K_DRIVE_KINEMATICS = new DifferentialDriveKinematics(kTRACK_WIDTH_M);
        

        public static final double kDRIVE_P = 0.35;
        public static final double kDRIVE_I = 0.0;
        public static final double kDRIVE_D = 0.0;
        public static final double kDRIVE_TOLERANCE = 2;
        //public static final double kTURN_ANGLE_TOLERANCE_DEG_PER_S = 10;
        /*
        public static final double kKEEP_HEADING_P =  0.2;
		public static final double kKEEP_HEADING_I =  0.0;
		public static final double kKEEP_HEADING_D = 0.4;
        public static final double kKEEP_HEADING_TOLERANCE =  1;
        */
    }

    public static final class ClimberConstants{
        public static final int kCLIMBER_FWD_PORT = 7;
        public static final int kCLIMBER_INV_PORT = 12;
        public static final int kCLIMBER_BRAKE_PORT = 1;
        public static final double kCLIMB_SPEED = 0.8;
        public static final double kCLIMB_UP_DIRECTION = 1;
        public static final double kCLIMB_UP_SPEED = kCLIMB_SPEED * kCLIMB_UP_DIRECTION;
        public static final double kCLIMB_DOWN_SPEED = kCLIMB_SPEED * -kCLIMB_UP_DIRECTION;
        public static final double kCLIMB_STOP = 0;
        public static final double kCLIMB_BRAKE_CLOSE_ANGLE = 1;
        public static final double kCLIMB_BRAKE_OPEN_ANGLE = 0;
    }

    public static final class HopperConstants {
        public static final int kHOPPER_MOTOR_PORT = 8;
        public static final int kSHOOTER_SENSOR_PORT = 0;
        public static final int kINTAKE_SENSOR_PORT = 1;
        public static final int kBELT_FRAME_OUT = 4;
        public static final int kBELT_FRAME_IN = 1;

        public static final double kHOPPER_SPEED = 0.55;
        public static final double kHOPPER_INTAKE_SPEED = 0.6;
        public static final double kHOPPER_LAUNCH_SPEED = 0.3;
        public static final double kHOPPER_ZERO_SPEED = 0;
        public static final double kDISTANCE_THRESHOLD = 10.0;
    }

    public static final class ShooterConstants {
        public static final int kSHOOTER_MOTOR0_PORT = 9;
        public static final int kSHOOTER_MOTOR1_PORT = 10;

        public static final int kSHOOTER_LIMIT_AMPS = 40;
        public static final double kSHOOTER_RAMP_SEC = 0.5;

        public static final double kSHOOTER_MAX_VOLTS = 9;
        public static final double kSHOOTER_MIN_VOLTS = 5;
        public static final double kSHOOTER_RANGE = kSHOOTER_MAX_VOLTS - kSHOOTER_MIN_VOLTS;
        // check the Launcher Wheel Variables spreadsheet to see what these numbers do
        // low offset is for formula 1
       // public static final double kSHOOTER_LOW_OFFSET = 0.90;
        // constants for formula 2 
        //public static final double kUPPER_WHEEL_K2 = 14;
        //public static final double kUPPER_WHEEL_K1 = 6;
        // contstans for formula 3
        //public static final double kUPPER_WHEEL_K3 = 3;
        //public static final double kUPPER_WHEEL_K4 = 1;

        public static final int kSHOOTER_DIRECTION = -1;
    }

    public static final class VisionConstants {
        public static final int kSERVO_SHOOTER_ANGLE = 0;
        public static final int kSERVO_DOWN_ANGLE = 0;
        public static final int kSERVO_UP_ANGLE = 90;
        public static final int kLIGHT_RELAY_PORT = 0;

        public static final String kVISION_TABLE_KEY = "Vision";
        public static final String kIS_LOADING_STATION_ALIGNED_KEY = "isLoadingStationAligned";
        public static final String kIS_HIGH_GOAL_ALIGNED_KEY = "isHighGoalAligned";
        public static final String kIS_VISION_ON_KEY = "isVisionOn";
        public static final String kVISION_OFFSET_KEY = "highGoalOffset";
        public static final String kVISION_DISTANCE_KEY = "highGoalDistance";

        public static final String kVISION_TAB_KEY = "Vision Table";

    }
    public static final class IntakeConstants{
        public static final int kBELT_MOTOR_PORT = 11;
        public static final double kBELT_MOTOR_SPEED = -0.25;   //was 3 9/26/2021
        public static final int kINTAKE_DOWN = 2;
        public static final int kINTAKE_UP = 3;
    } 

    public static final class ConsoleCommandConstants{
        public static final int kPOV_INTERVAL = 45;
    
        public static final int kPOSITION_LEFT_I = 0;
        public static final int kPOSITION_MIDDLE_I = 1;
        public static final int kPOSITION_RIGHT_I = 2;
        
        //public static final String kPOS_PATTERN_NAME[][] = {{"Left.Crossline", "Left.ShootCross", "Left.ShootGatther"},
        //                                                    {"Mid.Crossline", "Mid.ShootCross", "Mid.ShootGather"},
        //                                                    {"Right.Crossline", "Right.ShootCross", "Right.ShootGather"},
        //                                                    {"Home.Funky", "Home.Pat2", "Home.Pat3", "Home.Pat4", "Home.Pat5"}};
        public static final String kPOS_PATTERN_NAME[] = {"Left", "Middle", "Right"};
    }

    public static final class AutoConstants{
        public static final double kDISTANCE_1_LEFT = -0.4;
        public static final double kDISTANCE_1_MIDDLE = -0.5;
        public static final double kDISTANCE_1_RIGHT = -0.4;
        public static final double kDISTANCE_2_LEFT = -1.0;
        public static final double kDISTANCE_2_MIDDLE = -0.0;
        public static final double kDISTANCE_2_RIGHT = -.75;
        public static final double kTURN_2_LEFT = .4;
        public static final double kTURN_2_MIDDLE = -0.0;
        public static final double kTURN_2_RIGHT = .5;
        public static final double kLAUNCH_LOWER_VOLTAGE = 7.75;
        public static final double kLAUNCH_UPPER_VOLTAGE = 5.75;
        public static final double kLAUNCH_TIMER = 4;
    }
}
