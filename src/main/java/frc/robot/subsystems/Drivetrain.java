package frc.robot.subsystems;

// wpilib imports
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants.CANIDs;
import frc.robot.Constants.DrivetrainConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// navx
import com.kauailabs.navx.frc.AHRS;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
    public final SwerveModule m_frontLeft = new SwerveModule(CANIDs.FL_DRIVE_ID,
            CANIDs.CANBUS_NAME, CANIDs.FL_STEER_ID, CANIDs.CANBUS_NAME,
            CANIDs.FL_ENCODER_ID, CANIDs.CANBUS_NAME);
    private final SwerveModule m_frontRight = new SwerveModule(CANIDs.FR_DRIVE_ID,
            CANIDs.CANBUS_NAME, CANIDs.FR_STEER_ID, CANIDs.CANBUS_NAME,
            CANIDs.FR_ENCODER_ID, CANIDs.CANBUS_NAME);
    private final SwerveModule m_backLeft = new SwerveModule(CANIDs.BL_DRIVE_ID,
            CANIDs.CANBUS_NAME, CANIDs.BL_STEER_ID, CANIDs.CANBUS_NAME,
            CANIDs.BL_ENCODER_ID, CANIDs.CANBUS_NAME);
    private final SwerveModule m_backRight = new SwerveModule(CANIDs.BR_DRIVE_ID,
            CANIDs.CANBUS_NAME, CANIDs.BR_STEER_ID, CANIDs.CANBUS_NAME,
            CANIDs.BR_ENCODER_ID, CANIDs.CANBUS_NAME);

    private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            DrivetrainConstants.FL_SWERVE_LOCATION, DrivetrainConstants.FR_SWERVE_LOCATION,
            DrivetrainConstants.BL_SWERVE_LOCATION,
            DrivetrainConstants.BL_SWERVE_LOCATION);

    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
            m_kinematics,
            m_gyro.getRotation2d(),
            new SwerveModulePosition[] {
                    m_frontLeft.getPosition(),
                    m_frontRight.getPosition(),
                    m_backLeft.getPosition(),
                    m_backRight.getPosition()
            });

    public Drivetrain() {
        // reset the gyro
        m_gyro.reset();
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    public void drive(
            double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
        var swerveModuleStates = m_kinematics.toSwerveModuleStates(
                ChassisSpeeds.discretize(
                        fieldRelative
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                        xSpeed, ySpeed, rot,
                                        m_gyro.getRotation2d())
                                : new ChassisSpeeds(xSpeed, ySpeed, rot),
                        periodSeconds));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
                DrivetrainConstants.MAX_LINEAR_SPEED);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
    }

    /** Updates the field relative position of the robot. */
    public void updateOdometry() {
        SwerveModulePosition[] pos = new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
        };
        SmartDashboard.putNumber("FL Angle", pos[0].angle.getDegrees());
        SmartDashboard.putNumber("FR Angle", pos[1].angle.getDegrees());
        SmartDashboard.putNumber("BL Angle", pos[2].angle.getDegrees());
        SmartDashboard.putNumber("BR Angle", pos[3].angle.getDegrees());
        m_odometry.update(
                m_gyro.getRotation2d(), pos);
    }
}