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
    }
    public static final class Wrist {
        public static int wristMotor = 40;
        public static int wristBottom = 5;
        public static int wristTop = 6;

    }
}
