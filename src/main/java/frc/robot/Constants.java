package frc.robot;

import java.nio.file.Path;
import java.nio.file.Paths;

/**
 * Constants
 */
public final class Constants {
    public static final class Sensors {
        public static final class Encoders {
            public static final class Quad {
                public static final int upr = 1024;
            }
    
            public static final class SparkMAX {
                public static final int upr = 42;
            }
        }
    }
    
    public static final class Autonomous {
        /**
         * How long to brake after the trajectory has ended
         */
        public static final double brakeDelay = 1.0;

        // Trajectory generation consts
        public static final double ksVolts = 0;
        public static final double kvVoltSecondsPerMeter = 0;
        public static final double kaVoltSecondsSquaredPerMeter = 0;

        public static final double kMaxSpeedMetersPerSecond = 0;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0;

        // Trajectory following consts
        public static final double kPDriveVel = 0;
        public static final double kDDriveVel = 0;
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.2;

        public static final class Directories {
            /**
             * Path to deploy folder when running on robot
             */
            private static final Path robotDeployDir = Paths.get("home/lvuser/deploy");
            /**
             * Path to deploy folder when running on locally
             */
            private static final Path localDeployDir = Paths.get("src/main/deploy");
            /**
             * Name of the deploy subfolder in which trajectories are stored
             */
            private static final Path trajectoriesFolder = Paths.get("trajectories");

            /**
             * Path to trajectories folder when running on robot
             */
            public static Path robotTrajectoriesDir = robotDeployDir.resolve(trajectoriesFolder);

            /**
             * Path to trajectories folder when running locally
             */
            public static Path localTrajectoriesDir = localDeployDir.resolve(trajectoriesFolder);
        }
    }

    public static final class Drivetrain {
        /**
         * Distance between sides of drivetrain [M]
         */
        public static final double wheelDistance = 0.6;
        /**
         * Wheel's diameter [M]
         */
        public static final double wheelDiameter = 6 * 0.0254;
        /**
         * Distance Per Revolution [M]
         */
        public static final double DPR = wheelDiameter * Math.PI;

        /**
         * Reverses gyro output
         */
        public static final boolean gyroReversed = true;

        /**
         * Motor Controllers configuration
         */
        public static final class MotorControllers {
            // Remeber to only put master motor controllers here, use InvertType.FollowMaster for slave motors
        }
    }

    /**
     * Store joystick and button ports here
     */
    public static final class Joysticks {
        public static final class RightJoystick {
            public static final int port = 1;

            public static final class Buttons {
                // Nothing
            }
        }
        
        public static final class LeftJoystick {
            public static final int port = 0;

            public static final class Buttons {
                // Nothing
            }
        }

        public static final class OperatorJoystick {
            public static final int port = 2;
            public static final double triggerThreshold = 0.3;
            
            public static final class Buttons {
                // Remember to use XboxController.Button enum here
            }
            
            public static final class Rumble {
                public static final double on = 0.5;
                public static final double off = 0.0;
            }
        }
    }
}
