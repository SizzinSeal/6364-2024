package frc.robot.subsystems;

// wpilib imports
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

// phoenix 6 imports
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;

// local imports
import frc.robot.Constants.DrivetrainConstants;

public class SwerveModule {
        private final TalonFX m_driveMotor;
        private final TalonFX m_steerMotor;
        private final CANcoder m_encoder;

        // Gains are for example purposes only - must be determined for your own robot!
        private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

        // Gains are for example purposes only - must be determined for your own robot!
        private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
                        1,
                        0,
                        0,
                        new TrapezoidProfile.Constraints(
                                        DrivetrainConstants.MAX_ANGULAR_SPEED,
                                        DrivetrainConstants.MAX_ANGULAR_ACCELERATION));

        // Gains are for example purposes only - must be determined for your own robot!
        private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
        private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

        /**
         * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
         * and turning encoder.
         *
         * @param driveMotorID  CAN ID for the drive motor
         * @param driveMotorBus CAN bus name for the drive motor
         * @param steerMotorID  CAN ID for the turn motor
         * @param steerMotorBus CAN bus name for the turn motor
         * @param encoderID     CAN ID for the encoder
         * @param encoderBus    CAN bus name for the encoder
         */
        public SwerveModule(
                        int driveMotorID,
                        String driveMotorBus,
                        int steerMotorID,
                        String steerMotorBus,
                        int encoderID,
                        String encoderBus) {
                m_driveMotor = new TalonFX(driveMotorID, driveMotorBus);
                m_steerMotor = new TalonFX(steerMotorID, steerMotorBus);

                m_encoder = new CANcoder(encoderID, encoderBus);

                m_encoder.getAbsolutePosition().setUpdateFrequency(100);

                // Limit the PID Controller's input range between -pi and pi and set the input
                // to be continuous.
                m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
        }

        /**
         * Returns the current state of the module.
         *
         * @return The current state of the module.
         */
        public SwerveModuleState getState() {
                return new SwerveModuleState(
                                m_driveMotor.getVelocity().getValueAsDouble(),
                                new Rotation2d(m_encoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI));
        }

        /**
         * Returns the current position of the module.
         *
         * @return The current position of the module.
         */
        public SwerveModulePosition getPosition() {
                return new SwerveModulePosition(
                                m_driveMotor.getPosition().getValueAsDouble(),
                                new Rotation2d(m_encoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI));
        }

        /**
         * Sets the desired state for the module.
         *
         * @param desiredState Desired state with speed and angle.
         */
        public void setDesiredState(SwerveModuleState desiredState) {
                var encoderRotation = new Rotation2d(m_encoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI);

                // Optimize the reference state to avoid spinning further than 90 degrees
                SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);

                // Scale speed by cosine of angle error. This scales down movement perpendicular
                // to the desired
                // direction of travel that can occur when modules change directions. This
                // results in smoother
                // driving.
                state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

                // Calculate the drive output from the drive PID controller.
                final double driveOutput = m_drivePIDController.calculate(m_driveMotor.getVelocity().getValueAsDouble(),
                                state.speedMetersPerSecond);

                final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

                // Calculate the turning motor output from the turning PID controller.
                final double turnOutput = m_turningPIDController.calculate(
                                m_encoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI,
                                state.angle.getRadians());

                final double turnFeedforward = m_turnFeedforward
                                .calculate(m_turningPIDController.getSetpoint().velocity);

                m_driveMotor.setVoltage(driveOutput + driveFeedforward);
                m_steerMotor.setVoltage(turnOutput + turnFeedforward);
        }
}
