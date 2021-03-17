/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class ButtonMappings {
        public static final int kROTATION_CONTROL = 4;
        public static final int kPOSITION_CONTROL = 6;
        public static final int kWHEEL_OF_FORTUNE_CW = 12;
        public static final int kWHEEL_OF_FORTUNE_CCW = 11;
        public static final int kSHOOTER = 1;

        public static final int kLOADER = 2;
        public static final int kCLIMB_UP = 5;
        public static final int kCLIMB_DOWN = 3;
        public static final int kVISION_ON = 8;
        public static final int kVISION_DOWN = 7;
        
        public static final int kCLIMB_SWITCH = 9;

        public static final int kHARVESTER_REVERSE = 10;
    }

    public static final class ContestantConstants {
        public static final String kBLUE_STRING = "B";
        public static final String kRED_STRING = "R";
        public static final String kGREEN_STRING = "G";
        public static final String kYELLOW_STRING = "Y";

        public static final int kCHANGES_PER_ROT = 24;
        public static final int kMOTOR_ID = 6;
        public static final double kPOSITION_MOTOR_SPEED = 0.25;
        public static final double kROTATION_MOTOR_SPEED = 0.5;
        public static final double kOVERRIDE_SPEED = 1.0;
        public static final double kZERO_SPEED = 0;

        public static final double kCOLOR_CONFIDENCE_THRESHOLD = 0.95;  // .85

        //public static final int CONTESTANT_MOTOR = 

        public enum Color {
            RED(0.531006, 0.337158, 0.128906, kRED_STRING),
            BLUE(0.119873, 0.414551, 0.465820, kBLUE_STRING),
            YELLOW(0.321289, 0.555908, 0.123291, kYELLOW_STRING),
            GREEN(0.165527, 0.573975, 0.260010, kGREEN_STRING);

            private double m_dRed;
            private double m_dGreen;
            private double m_dBlue;

            private String m_name;

            Color(double red, double green, double blue, String name) {
                this.m_dRed = red;
                this.m_dBlue = blue;
                this.m_dGreen = green;
                this.m_name = name;
            }
            public double getRed(){
                return m_dRed;
            }

            public double getGreen(){
                return m_dGreen;
            }

            public double getBlue(){
                return m_dBlue;
            }

            public final String getName(){
                return m_name;
            }
        }
        
        public enum Direction {
            CW, CCW
        }
    }
    
    public static final class DriveConstants {
        public static final int kLeftMotor1Port = 2;
        public static final int kLeftMotor2Port = 3;
        public static final int kRightMotor1Port = 4;
        public static final int kRightMotor2Port = 5;

        public static final double kRAMP_RATE = 1.0;
        public static final int kCURRENT_LIMT = 40;

        public static final double kGEARBOX_REDUCTION = (50.0/12.0) * (60.0/14.0);
        public static final double kTIRE_SIZE = 7.9;
        public static final double kPULSE_PER_ROTATION = 1;

        public static final double kTURN_ANGLE_P = 0.2;
        public static final double kTURN_ANGLE_I = 0.0;
        public static final double kTURN_ANGLE_D = 0.4;
        public static final double kTURN_ANGLE_TOLERANCE = 2;
        public static final double kTURN_ANGLE_TOLERANCE_DEG_PER_S = 10;

        public static final double kKEEP_HEADING_P =  0.2;
		public static final double kKEEP_HEADING_I =  0.0;
		public static final double kKEEP_HEADING_D = 0.4;
        public static final double kKEEP_HEADING_TOLERANCE =  1;
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
        public static final int kHOPPER_MOTOR_PORT = 8; // Arbitrary number for now
        public static final int kSHOOTER_SENSOR_PORT = 0;
        public static final int kINTAKE_SENSOR_PORT = 1;
        public static final int kHOPPER_ENCODER_PORT = 3;
        public static final double kDISTANCE_PER_ROTATION = 6.0;
        public static final double kHOPPER_ENCODER_TOLERANCE = 0.05;
        //public static final int kHOPPER_AT_INTAKE_PORT = 2;
        public static final double kHOPPER_SPEED = -0.5;
        public static final double kHOPPER_ZERO_SPEED = 0;
        public static final double kDISTANCE_THRESHOLD = 10.0;
    }

    public static final class ShooterConstants {
        public static final int kSHOOTER_MOTOR0_PORT = 9;
        public static final int kSHOOTER_MOTOR1_PORT = 10;

        public static final double kSHOOTER_SPEED = -.6;
        public static final double kSHOOTER_ZERO_SPEED = 0;

        public static final int kSHOOTER_DIRECTION = 1;
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
        public static final double kBELT_MOTOR_SPEED = -0.5;
    } 
}
