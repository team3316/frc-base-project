package frc.robot;

import java.nio.file.Path;
import java.nio.file.Paths;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.team3316.kit.control.PIDFGains;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.autonomous.AutoPlan;
import frc.robot.autonomous.StartingConfig;

/**
 * Constants
 */
public final class Constants {
//TODO: CHANGE ALL CONSTANTS TO ACTUAL VALUES
    
    public static final class Debug {
        public static final boolean brakeOnDisable = false;
    }

    public static final class Sensors {
        public static final class Encoders {
            public static final class Quad {
                public static final int upr = 1024;
            }
    
            public static final class SparkMAX {
                public static final int upr = 42;
            }
        }
        
        public static final class PigeonIMU {
            public static final double unitsPerDegree = 16;
        }
    }
    
    public static final class Autonomous {
        // Set autonomous plan and starting config
        public static final StartingConfig startingConfig = StartingConfig.exampleStartingConfig;
        public static final AutoPlan autoPlan = AutoPlan.BottomPortPlan;

        // How long to brake after the trajectory has ended
        public static final double brakeDelay = 1.0;

        // Trajectory generation consts
        public static final double ksVolts = 1.61;
        public static final double kvVoltSecondsPerMeter = 0.0234;
        public static final double kaVoltSecondsSquaredPerMeter = 0.00788;

        public static final double kMaxSpeedMetersPerSecond = 2.6; // 1.0
        public static final double kMaxAccelerationMetersPerSecondSquared = 3.0; // 2.0

        // Trajectory following consts
        public static final double kPDriveVel = 0.715 * 16; // 0.715 * 1
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.2; // 0.7

        public static final class Directories {
            // paths to trajectory folder when running both localy and on robot
            public static final Path robotDeployDir = Paths.get("home/lvuser/deploy");
            public static final Path localDeployDir = Paths.get("src/main/deploy");
            // The name of the folder inside the deploy folder which stores the trajectories
            public static final Path trajectoriesFolderName = Paths.get("trajectories");

            /**
             * Returns the path to the trajectories folder on the robot
             * @return robotDeployDir combined with trajectoriesFolderName
             */
            public static Path robotTrajectoriesDir() {
                return robotDeployDir.resolve(trajectoriesFolderName);
            }

            /**
             * Returns the path to the trajectories folder on the robot
             * @return localDeployDir combined with trajectoriesFolderName
             */
            public static Path localTrajectoriesDir() {
                return localDeployDir.resolve(trajectoriesFolderName);
            }
        }
    }

    public static final class Intake {
        public static final double rollerVoltageCompensation = 12;
        public static final double armVoltageCompensation = 6;

        public static final class IntakePosition {
            public static final double tolerance = 1;

            public static final class IN {
                public static final double percent = 1.0;
            }

            public static final class KEEPIN {
                public static final double percent = 0.1;
            }

            public static final class OUT {
                public static final double percent = -0.5;
            }

            public static final class KEEPOUT {
                public static final double percent = -0.1;
            }

            public static final class DEFAULT {
                public static final double percent = 0;
            }

        }

        public static final class Roller {
            public static final double tolerance = 0.5;

            public static final class RollerState {
                public static final class IN {
                    public static final double voltage = 0.75;
                }

                public static final class OUT {
                    public static final double voltage = -0.2;
                }

                public static final class DEFAULT {
                    public static final double voltage = 0;
                }
            }
        }

        public static final class MotorControllers {
            public static final class RollerVictor {
                public static final boolean inverted = false;
            }
            public static final class ArmTalon {
                public static final boolean inverted = false;
            }
        }
    }

    public static final class Shooter {
        public static final int peakCurrent = 60;

        public static final class State {
            public static final double tolerance = 100;
            public static final class Shoot {
                public static final double velocityLeft = 4000;
                public static final double velocityRight = 4000;
                public static final PIDFGains gainsLeft = new PIDFGains(0.0005, 0.0, 0.0, 0.00215, tolerance);
                public static final PIDFGains gainsRight = new PIDFGains(0.0005, 0.0, 0.0, 0.00215, tolerance);

            }

            public static final class ShootBottom {
                public static final double velocityLeft = 1150;
                public static final double velocityRight = 1150;
                public static final PIDFGains gainsLeft = new PIDFGains(0.0002, 0.0, 0.03, 0.000183, tolerance);
                public static final PIDFGains gainsRight = new PIDFGains(0.0001, 0.0, 0.03, 0.0001815, tolerance);
            }

            public static final class None {
                public static final double velocityLeft = 0;
                public static final double velocityRight = 0;
            }
        }
        
        public static final class MotorControllers {
            public static final class SparkMaxLeft {
                public static final boolean inverted = false;
                public static final double dpr = 1; // In RPM
            }

            public static final class SparkMaxRight {
                public static final boolean inverted = true; // In RPM
                public static final double dpr = 1;
            }
        }
    }

    public static final class Pizza {
        public static final double radius = 280;

        public static final class MotorControllers {
            public static final class PizzaMotor {
                public static final boolean inverted = false;
                public static final boolean sensorPhase = false;
                public static final double voltageCompensation = 12.0;
                public static final double dpr = 360 * 500; //TODO: update when we will know the real gear ratio.

                public static final class PIDF {
                    public static final double kP = 0;
                    public static final double kI = 0;
                    public static final double kD = 0;
                    public static final double kF = 0;
                }
            }
        }

        public static final class PizzaState {
            public static final double tolerance = 0.2;
            public static final int toleranceRawUnits = 18;
            public static final class CLOCKWISE {
                public static final double percent = 0.4;
            }
            
            public static final class COUNTERCLOCKWISE {
                public static final double percent = -1.0;
            }

            public static final class CLOCKWISE_BOTTOM {
                public static final double percent = 1.0;
            }

            public static final class NONE {
                public static final double percent = 0.0;
            }
        }
    }
        

    public static final class Extractor {
        public static final class RollerVictor {
            public static final boolean inverted = true;
            public static final boolean sensorPhase = true;
            public static final double complientWheelRadius = 1.0;
            public static final double complientWheelGearRatio = 5;
            // Tolerance set to zero because we want the PID to run as long as the command is running
            public static final int tolerance = 0;
            public static final double dpr = 2 * Math.PI * RollerVictor.complientWheelRadius / RollerVictor.complientWheelGearRatio;
            public static final double voltageCompensation = 13;

            public static final class PIDF {
                public static final double kP = 0.0292;
                public static final double kI = 0.0;
                public static final double kD = 0.0;
                public static final double kF = 0.0644;
            }
        }
        public static final class RollerState {
            public static final double tolerance = 0.0; //Does not matter

            public static final class IN {
                public static final double percentage = 1.0;
            }

            public static final class OUT {
                public static final double percentage = -1.0;
            }

            public static final class NONE {
                public static final double percentage = 0.0;
            }
        }
    }

    public static final class Drivetrain {
        public static final double wheelDistance = 0.6;
        public static final double wheelDiameter = 6 * 0.0254;
        public static final double DPR = wheelDiameter * Math.PI;

        public static final double straightDriveMaxVelocityInRawUnits = 9000;

        public static final boolean gyroReversed = true;

        public static final class MotorControllers {
            public static final class TalonR {
                public static final boolean inverted = false;
                public static final boolean sensorPhase = false;
                public static final PIDFGains gainsDefault = new PIDFGains(0.1, 0.0, 0.0, 0.0, 1.0);
                public static final PIDFGains gainsTurn = new PIDFGains(0.03, 0.06, 0.0045, 0.0, 0.45);
            }
            public static final class VictorRF {
                public static final boolean inverted = false;
            }
            public static final class VictorRB {
                public static final boolean inverted = false;
            }
            public static final class TalonL {
                public static final boolean inverted = true;
                public static final boolean sensorPhase = false;
                public static final PIDFGains gainsDefault = new PIDFGains(0.1, 0.0, 0.0, 0.0, 1.0);
                public static final PIDFGains gainsTurn = new PIDFGains(0.027, 0.027, 0.0, 0.0, 0.45);
            }
            public static final class VictorLF {
                public static final boolean inverted = true;
            }
            public static final class VictorLB {
                public static final boolean inverted = true;
            }
        }
    }

    public static final class Climber {
        public static final double climberVoltageCompensation = 10;

        public static final class MotorControllers {
            public static final class ClimberTalon {
                public static final double voltageCompensation = 8;
                public static final double deadband = 0.15;
                public static final InvertType inverted = InvertType.None;
            }

            public static final class ClimberSpark {
                public static final double voltageCompensation = 7;
                public static final boolean inverted = true;
            }
        }


        public static final class ClimberState {
            public static final double tolerance = 0.2;

            public static final class PULL {
                public static final double percent = 1.0;
            }

            public static final class NONE {
                public static final double percent = 0.0;
            }
        }
    }

    public static final class Joysticks {
        public static final class RightJoystick {
            public static final int port = 1;
            public static final class Buttons {
                public static final int straightDrive = 1;
                public static final int goToYaw = 2;
            }
        }
        
        public static final class LeftJoystick {
            public static final int port = 0;
            public static final class Buttons {
                public static final int gearShift = 1;
            }
        }

        public static final class OperatorJoystick {
            public static final int port = 2;
            public static final double triggerThreshold = 0.3;
            
            public static final class Buttons {
                public static final int pizzaCounterClockwise = XboxController.Button.kA.value;
                public static final int pizzaClockwise = XboxController.Button.kX.value;
                public static final int climb = XboxController.Button.kY.value;
                public static final int collection = XboxController.Button.kBumperLeft.value;
                public static final int bottomShoot = XboxController.Button.kBack.value;
                public static final int openClimber = XboxController.Button.kB.value;
                public static final int visionShoot = XboxController.Button.kBumperRight.value;
            }
            
            public static final class Rumble {
                public static final double on = 0.5;
                public static final double off = 0.0;
            }
        }
    }
}
