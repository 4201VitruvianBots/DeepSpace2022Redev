package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

public final class Constants {
    public static final class USB { 
        public static final int leftJoystick = 0;
        public static final int rightJoystick = 1;
        public static final int xBoxController = 2;

    }

    public static final class Pneumatics { 
        public static final int pcmOne = 11;
        public static final PneumaticsModuleType pcmType = PneumaticsModuleType.CTREPCM;

        public static final int climbPistonForward = 6;
        public static final int climbPistonReverse = 7;
        
    }

    public static final class Climber { 
        public static final int climbMotor = 50; 

    }
    public static final class Elevator { 
        public static int leftElevatorA = 30;
        public static int leftElevatorB = 31;
        public static int rightElevatorA = 32;
        public static int rightElevatorB = 33;

        public static int elevatorBottom = 2;
        public static int elevatorTop = 3;
        public static int elevatorMid = 4;

        public static double elevatorSetPoint = 0;
    }
    public static final class Wrist {
        public static int wristMotor = 40;
        public static int wristBottom = 5;
        public static int wristTop = 6;
    }
    public static final class CAN {
        public static int mechanismPositions;
        // Constants
    
        public static int leftJoystick = 0;
        public static int rightJoystick = 1;
        public static int xBoxController = 2;
    
        // Pneumatics
        public static int pdp = 0;
        public static int PCMOne = 11;
            public static int driveTrainShifterForward = 0;
            public static int driveTrainShifterReverse = 1;
            public static int hatchIntakeExtendForward = 2;
            public static int hatchIntakeExtendReverse = 3;
            public static int hatchIntakeSecureForward = 4;
            public static int hatchIntakeSecureReverse = 5;
            public static int climbPistonForward = 6;
            public static int climbPistonReverse = 7;
    
        // Drivetrain
        public static int leftFrontDriveMotor = 20;
        public static int leftRearDriveMotor = 21;
        public static int rightFrontDriveMotor = 22;
        public static int rightRearDriveMotor = 23;
        public static int climbDriveMotor = 24;
    
        public static int leftElevatorA = 30;
        public static int leftElevatorB = 31;
        public static int rightElevatorA = 32;
        public static int rightElevatorB = 33;
    
        public static int wristMotor = 40;
    
        public static int climbMotor = 50;
    
        public static int cargoIntakeMotor = 60;
        public static int hatchIntakeMotor = 61;
    
        // DIOs
        public static int bannerIR = 0;
        public static int hatchSensor = 1;
        public static int elevatorBottom = 2;
        public static int elevatorTop = 3;
        public static int elevatorMid = 4;
        public static int wristBottom = 5;
        public static int wristTop = 6;
        public static int robotSwitch = 9;
    
        public static int ledCh0 = 23;  //LED info channels start from 9 on NavX    Arduino port 12
        public static int ledCh1 = 22;  //and go down                               Arduino port 11
        public static int ledCh2 = 21;  //                                          Arduino port 10
        public static int ledCh3 = 20;  //                                          Arduino port 9
    
        public static int LED_BLUE = 1;
        public static int LED_RED = 2;
        public static int LED_YELLOW = 3;
        public static int LED_GREEN = 4;
    
        // PDP Channels
        public static int pdpChannelDriveTrainLeftForward = 14;
        public static int pdpChannelDriveTrainLeftReverse = 15;
        public static int pdpChannelDriveTrainRightForward = 1;
        public static int pdpChannelDriveTrainRightReverse = 0;
        public static int pdpChannelElevatorLeft = 13;
        public static int pdpChannelElevatorRight = 2;
    
        // Setpoints (Units in inches or degrees)
        public static double WRIST_RETRACTED_ANGLE = 165;
        public static double WRIST_RETRACTED_CARGO_ANGLE = 135;
        public static double WRIST_EXTENDED_ANGLE = 0;
        public static double WRIST_CARGO_ANGLE = 45;
        public static double WRIST_CARGO_SHIP_ANGLE = 0;
        public static double WRIST_CARGO_INTAKE_STATION_ANGLE = 90;
        public static double WRIST_CARGO_HIGH_ANGLE = 40;
        public static double WRIST_HATCH_LOW_ANGLE = 45;
        public static double WRIST_HATCH_ANGLE = 85;
        public static double ELEVATOR_HOME_POSITION = 0;
    //    public static double ELEVATOR_HATCH_INTAKE_POSITION = 10.5;
        //public static double ELEVATOR_HATCH_LOW_POSITION = 10;
        public static double ELEVATOR_HATCH_LOW_POSITION = 7;
        public static double ELEVATOR_HATCH_MID_POSITION = 37;
        public static double ELEVATOR_HATCH_HIGH_POSITION = 60;
        public static double ELEVATOR_HATCH_GROUND_LOW_POSITION = 0;
        public static double ELEVATOR_HATCH_GROUND_MID_POSITION = 20;
        public static double ELEVATOR_HATCH_GROUND_HIGH_POSITION = 47;
        public static double ELEVATOR_CARGO_INTAKE_DEPOT_POSITION = 1; // 2
        public static double ELEVATOR_CARGO_INTAKE_STATION_POSITION = 10.5;
        public static double ELEVATOR_CARGO_SHIP_POSITION = 35;
        public static double ELEVATOR_CARGO_LOW_POSITION = 10;
        public static double ELEVATOR_CARGO_MID_POSITION = 35;
        public static double ELEVATOR_CARGO_HIGH_POSITION = 62;
        public static double ELEVATOR_CLIMB_POSITION = 30;
    
        //
        public static double CARGO_INTAKE_SPEED = -0.7;
        public static double CARGO_OUTTAKE_SPEED = 0.7;
        public static double CARGO_HOLD_SPEED = -0.2;
        public static double HATCH_INTAKE_SPEED = -0.8;
        public static double HATCH_HOLD_SPEED = 0;
        public static double HATCH_OUTTAKE_SPEED = 0.8;
        public static double HATCH_GROUND_INTAKE_SPEED = -0.8;
        public static double HATCH_GROUND_HOLD_SPEED = -0.1;
        public static double HATCH_GROUND_OUTTAKE_SPEED = 1;
    
    
        // Pathfinder (Units in feet)
        public static double wheel_diameter = 0.5104167;
        public static double wheelbase = 2.20833;
        public static double max_vel_high = 0.5104167;
        public static double max_vel_low = 1.6;
    }
}