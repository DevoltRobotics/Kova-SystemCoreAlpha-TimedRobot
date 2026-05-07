// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Ported from the FTC GoBILDA Pinpoint driver (MIT License, Base 10 Assets, LLC, 2025)

package first.robot.driver;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import org.wpilib.hardware.bus.I2C;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.SimDevice;
import org.wpilib.hardware.hal.SimDouble;
import org.wpilib.hardware.hal.SimEnum;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.DoubleTopic;
import org.wpilib.networktables.NTSendable;
import org.wpilib.networktables.NTSendableBuilder;
import org.wpilib.util.sendable.SendableRegistry;

/**
 * Driver for the goBILDA® Pinpoint Odometry Computer.
 *
 * <p>The Pinpoint is an I2C device that performs sensor fusion between two odometry pods and an
 * internal IMU to provide real-time position (X, Y, heading) and velocity tracking.
 *
 * <p>Call {@link #update()} once per loop to refresh all cached values before reading them.
 */
public class GoBildaPinpointDriver implements NTSendable, AutoCloseable {

  /** Default I2C device address (7-bit). */
  public static final byte DEFAULT_ADDRESS = 0x31;

  // Ticks-per-mm constants for goBILDA odometry pods
  private static final float TICKS_PER_MM_SWINGARM = 13.26291192f;
  private static final float TICKS_PER_MM_4_BAR    = 19.89436789f;

  // -------------------------------------------------------------------------
  // Register map (each register is a 4-byte little-endian int or float)
  // -------------------------------------------------------------------------
  private static final int REG_DEVICE_ID      = 1;
  private static final int REG_DEVICE_VERSION = 2;
  private static final int REG_DEVICE_STATUS  = 3;
  private static final int REG_DEVICE_CONTROL = 4;
  private static final int REG_LOOP_TIME      = 5;
  private static final int REG_X_ENCODER      = 6;
  private static final int REG_Y_ENCODER      = 7;
  private static final int REG_X_POSITION     = 8;
  private static final int REG_Y_POSITION     = 9;
  private static final int REG_H_ORIENTATION  = 10;
  private static final int REG_X_VELOCITY     = 11;
  private static final int REG_Y_VELOCITY     = 12;
  private static final int REG_H_VELOCITY     = 13;
  private static final int REG_TICKS_PER_MM   = 14;
  private static final int REG_X_POD_OFFSET   = 15;
  private static final int REG_Y_POD_OFFSET   = 16;
  private static final int REG_YAW_SCALAR     = 17;
  private static final int REG_BULK_READ      = 18;

  // -------------------------------------------------------------------------
  // Device Control command bit-fields
  // -------------------------------------------------------------------------
  private static final int CTRL_RECALIBRATE_IMU      = 1 << 0;
  private static final int CTRL_RESET_POS_AND_IMU    = 1 << 1;
  private static final int CTRL_SET_Y_ENCODER_REV    = 1 << 2;
  private static final int CTRL_SET_Y_ENCODER_FWD    = 1 << 3;
  private static final int CTRL_SET_X_ENCODER_REV    = 1 << 4;
  private static final int CTRL_SET_X_ENCODER_FWD    = 1 << 5;

  // -------------------------------------------------------------------------
  // Device Status bit-fields
  // -------------------------------------------------------------------------
  private static final int STATUS_READY                  = 1 << 0;
  private static final int STATUS_CALIBRATING            = 1 << 1;
  private static final int STATUS_FAULT_X_NOT_DETECTED  = 1 << 2;
  private static final int STATUS_FAULT_Y_NOT_DETECTED  = 1 << 3;
  private static final int STATUS_FAULT_IMU_RUNAWAY     = 1 << 4;
  private static final int STATUS_FAULT_BAD_READ        = 1 << 5;

  // -------------------------------------------------------------------------
  // Public enums
  // -------------------------------------------------------------------------

  /** Current operating status of the Pinpoint device. */
  public enum DeviceStatus {
    /** Device is powering up and has not yet initialized. */
    NOT_READY,
    /** Device is functioning normally. */
    READY,
    /** Device is recalibrating its internal gyro. Robot must be stationary. */
    CALIBRATING,
    /** Neither odometry pod is detected. */
    FAULT_NO_PODS_DETECTED,
    /** The X (forward) odometry pod is not detected. */
    FAULT_X_POD_NOT_DETECTED,
    /** The Y (strafe) odometry pod is not detected. */
    FAULT_Y_POD_NOT_DETECTED,
    /** The IMU heading is diverging unexpectedly. */
    FAULT_IMU_RUNAWAY,
    /** A bad I2C read was detected; the last good values are being reported. */
    FAULT_BAD_READ
  }

  /** Direction of an odometry pod encoder. */
  public enum EncoderDirection {
    /** Encoder counts increase when the robot moves in the positive direction. */
    FORWARD,
    /** Encoder counts are inverted relative to robot motion. */
    REVERSED
  }

  /** Pre-defined ticks-per-mm constants for goBILDA odometry pods. */
  public enum GoBildaOdometryPods {
    /** goBILDA Swingarm Pod (~13.26 ticks/mm). */
    SWINGARM_POD,
    /** goBILDA 4-Bar Pod (~19.89 ticks/mm). */
    FOUR_BAR_POD
  }

  /** Selects a reduced update scope for {@link #update(ReadData)}. */
  public enum ReadData {
    /** Only refresh the heading (H orientation) register. Faster than a full bulk read. */
    ONLY_UPDATE_HEADING
  }

  // -------------------------------------------------------------------------
  // Hardware / sim fields
  // -------------------------------------------------------------------------

  private I2C m_i2c;

  private SimDevice m_simDevice;
  private SimDouble m_simX;
  private SimDouble m_simY;
  private SimDouble m_simH;
  private SimDouble m_simVelX;
  private SimDouble m_simVelY;
  private SimDouble m_simVelH;
  private SimEnum   m_simStatus;

  // -------------------------------------------------------------------------
  // Cached values (refreshed each call to update())
  // -------------------------------------------------------------------------

  private int   m_deviceStatusRaw = 0;
  private int   m_loopTime        = 0;
  private int   m_xEncoderValue   = 0;
  private int   m_yEncoderValue   = 0;
  private float m_xPosition       = 0;
  private float m_yPosition       = 0;
  private float m_hOrientation    = 0;
  private float m_xVelocity       = 0;
  private float m_yVelocity       = 0;
  private float m_hVelocity       = 0;

  // -------------------------------------------------------------------------
  // Constructor
  // -------------------------------------------------------------------------

  /**
   * Constructs the GoBILDA Pinpoint driver using the default I2C address (0x31).
   *
   * @param port The I2C port the device is attached to.
   */
  public GoBildaPinpointDriver(I2C.Port port) {
    this(port, DEFAULT_ADDRESS);
  }

  /**
   * Constructs the GoBILDA Pinpoint driver.
   *
   * @param port          The I2C port the device is attached to.
   * @param deviceAddress 7-bit I2C address of the device (default 0x31).
   */
  public GoBildaPinpointDriver(I2C.Port port, int deviceAddress) {
    m_i2c = new I2C(port, deviceAddress);

    // Simulation device
    m_simDevice = SimDevice.create("Odometry:GoBildaPinpoint", port.value, deviceAddress);
    if (m_simDevice != null) {
      m_simStatus = m_simDevice.createEnumDouble(
          "status",
          SimDevice.Direction.OUTPUT,
          new String[] {"NOT_READY", "READY", "CALIBRATING", "FAULT"},
          new double[] {0, 1, 2, 3},
          1); // default READY
      m_simX    = m_simDevice.createDouble("x_mm",    SimDevice.Direction.INPUT,  0.0);
      m_simY    = m_simDevice.createDouble("y_mm",    SimDevice.Direction.INPUT,  0.0);
      m_simH    = m_simDevice.createDouble("h_rad",   SimDevice.Direction.INPUT,  0.0);
      m_simVelX = m_simDevice.createDouble("velX_mm", SimDevice.Direction.INPUT,  0.0);
      m_simVelY = m_simDevice.createDouble("velY_mm", SimDevice.Direction.INPUT,  0.0);
      m_simVelH = m_simDevice.createDouble("velH_rad",SimDevice.Direction.INPUT,  0.0);
    }

    HAL.reportUsage("I2C[" + port.value + "][" + deviceAddress + "]", "GoBildaPinpoint");
    SendableRegistry.add(this, "GoBildaPinpoint", port.value);
  }

  // -------------------------------------------------------------------------
  // AutoCloseable
  // -------------------------------------------------------------------------

  @Override
  public void close() {
    SendableRegistry.remove(this);
    if (m_i2c != null) {
      m_i2c.close();
      m_i2c = null;
    }
    if (m_simDevice != null) {
      m_simDevice.close();
      m_simDevice = null;
    }
  }

  // -------------------------------------------------------------------------
  // Low-level I2C helpers
  // -------------------------------------------------------------------------

  /**
   * Reads a 4-byte little-endian signed integer from the given register.
   *
   * @param register Register number.
   * @return The integer value stored in that register.
   */
  private int readInt(int register) {
    ByteBuffer buf = ByteBuffer.allocate(4);
    m_i2c.read(register, 4, buf);
    buf.order(ByteOrder.LITTLE_ENDIAN);
    return buf.getInt(0);
  }

  /**
   * Reads a 4-byte little-endian float from the given register.
   *
   * @param register Register number.
   * @return The float value stored in that register.
   */
  private float readFloat(int register) {
    ByteBuffer buf = ByteBuffer.allocate(4);
    m_i2c.read(register, 4, buf);
    buf.order(ByteOrder.LITTLE_ENDIAN);
    return buf.getFloat(0);
  }

  /**
   * Writes a 4-byte little-endian integer to the given register.
   *
   * @param register Register number.
   * @param value    Integer to write.
   */
  private void writeInt(int register, int value) {
    ByteBuffer buf = ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN);
    buf.putInt(value);
    m_i2c.writeBulk(buf.array());
    // WPILib I2C writeBulk sends starting at the previously addressed register;
    // we address the register first with a zero-length write then bulk-write the payload.
    m_i2c.write(register, 0); // address the register
    m_i2c.writeBulk(ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN).putInt(value).array());
  }

  /**
   * Writes a 4-byte little-endian float to the given register.
   *
   * @param register Register number.
   * @param value    Float to write.
   */
  private void writeFloat(int register, float value) {
    m_i2c.write(register, 0); // address the register
    m_i2c.writeBulk(ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN).putFloat(value).array());
  }

  // -------------------------------------------------------------------------
  // Corruption guards (mirrors FTC driver logic)
  // -------------------------------------------------------------------------

  private float guardPosition(float oldVal, float newVal, int threshold, boolean checkLoopTime) {
    boolean noData  = checkLoopTime && (m_loopTime < 1);
    boolean corrupt = noData || Float.isNaN(newVal) || Math.abs(newVal - oldVal) > threshold;
    if (!corrupt) return newVal;
    m_deviceStatusRaw = STATUS_FAULT_BAD_READ;
    return oldVal;
  }

  private float guardVelocity(float oldVal, float newVal, int threshold) {
    boolean noData  = (m_loopTime <= 1);
    boolean corrupt = noData || Float.isNaN(newVal) || Math.abs(newVal) > threshold;
    if (!corrupt) return newVal;
    m_deviceStatusRaw = STATUS_FAULT_BAD_READ;
    return oldVal;
  }

  // -------------------------------------------------------------------------
  // Status decoder
  // -------------------------------------------------------------------------

  private DeviceStatus decodeStatus(int s) {
    if ((s & STATUS_CALIBRATING) != 0)          return DeviceStatus.CALIBRATING;
    boolean xOk = (s & STATUS_FAULT_X_NOT_DETECTED) == 0;
    boolean yOk = (s & STATUS_FAULT_Y_NOT_DETECTED) == 0;
    if (!xOk && !yOk)                           return DeviceStatus.FAULT_NO_PODS_DETECTED;
    if (!xOk)                                   return DeviceStatus.FAULT_X_POD_NOT_DETECTED;
    if (!yOk)                                   return DeviceStatus.FAULT_Y_POD_NOT_DETECTED;
    if ((s & STATUS_FAULT_IMU_RUNAWAY) != 0)    return DeviceStatus.FAULT_IMU_RUNAWAY;
    if ((s & STATUS_READY) != 0)                return DeviceStatus.READY;
    if ((s & STATUS_FAULT_BAD_READ) != 0)       return DeviceStatus.FAULT_BAD_READ;
    return DeviceStatus.NOT_READY;
  }

  // -------------------------------------------------------------------------
  // Update methods
  // -------------------------------------------------------------------------

  /**
   * Performs a bulk I2C read (40 bytes) to refresh all cached values.
   *
   * <p>Call this exactly once per robot loop before reading any position or velocity data.
   */
  public void update() {
    // Sim shortcut
    if (m_simX != null) {
      m_xPosition    = (float) m_simX.get();
      m_yPosition    = (float) m_simY.get();
      m_hOrientation = (float) m_simH.get();
      m_xVelocity    = (float) m_simVelX.get();
      m_yVelocity    = (float) m_simVelY.get();
      m_hVelocity    = (float) m_simVelH.get();
      return;
    }

    final int POS_THRESHOLD  = 5000;  // > one FTC field in mm
    final int HEAD_THRESHOLD = 120;   // ~20 full rotations in radians
    final int VEL_THRESHOLD  = 10000; // 10 000 mm/s — faster than any FTC robot
    final int HVEL_THRESHOLD = 120;   // ~20 rotations/second

    float oldX  = m_xPosition;
    float oldY  = m_yPosition;
    float oldH  = m_hOrientation;
    float oldVX = m_xVelocity;
    float oldVY = m_yVelocity;
    float oldVH = m_hVelocity;

    // Single 40-byte bulk read starting at REG_BULK_READ
    ByteBuffer raw = ByteBuffer.allocate(40);
    m_i2c.read(REG_BULK_READ, 40, raw);
    raw.order(ByteOrder.LITTLE_ENDIAN);

    m_deviceStatusRaw = raw.getInt(0);
    m_loopTime        = raw.getInt(4);
    m_xEncoderValue   = raw.getInt(8);
    m_yEncoderValue   = raw.getInt(12);
    float newX  = raw.getFloat(16);
    float newY  = raw.getFloat(20);
    float newH  = raw.getFloat(24);
    float newVX = raw.getFloat(28);
    float newVY = raw.getFloat(32);
    float newVH = raw.getFloat(36);

    m_xPosition    = guardPosition(oldX,  newX,  POS_THRESHOLD,  true);
    m_yPosition    = guardPosition(oldY,  newY,  POS_THRESHOLD,  true);
    m_hOrientation = guardPosition(oldH,  newH,  HEAD_THRESHOLD, true);
    m_xVelocity    = guardVelocity(oldVX, newVX, VEL_THRESHOLD);
    m_yVelocity    = guardVelocity(oldVY, newVY, VEL_THRESHOLD);
    m_hVelocity    = guardVelocity(oldVH, newVH, HVEL_THRESHOLD);
  }

  /**
   * Performs a narrow I2C read to refresh only a subset of data.
   *
   * <p>Currently only {@link ReadData#ONLY_UPDATE_HEADING} is supported. This reads a single
   * register instead of all 40 bytes, which can improve loop timing when full odometry updates
   * are not needed every cycle.
   *
   * @param data The subset of data to update.
   */
  public void update(ReadData data) {
    if (data == ReadData.ONLY_UPDATE_HEADING) {
      final int HEAD_THRESHOLD = 120;
      float oldH = m_hOrientation;

      if (m_simH != null) {
        m_hOrientation = (float) m_simH.get();
        return;
      }

      float newH = readFloat(REG_H_ORIENTATION);
      m_hOrientation = guardPosition(oldH, newH, HEAD_THRESHOLD, false);

      // Clear a stale bad-read fault caused by this heading-only path
      if (m_deviceStatusRaw == STATUS_FAULT_BAD_READ) {
        m_deviceStatusRaw = STATUS_READY;
      }
    }
  }

  // -------------------------------------------------------------------------
  // Configuration writes
  // -------------------------------------------------------------------------

  /**
   * Sets the physical offset of each odometry pod from the robot's tracking center.
   *
   * <p>The X offset is how far sideways (left is positive) the X (forward) pod sits from center.
   * The Y offset is how far forward (forward is positive) the Y (strafe) pod sits from center.
   *
   * @param xOffsetMm X pod lateral offset in millimeters.
   * @param yOffsetMm Y pod longitudinal offset in millimeters.
   */
  public void setOffsets(double xOffsetMm, double yOffsetMm) {
    writeFloat(REG_X_POD_OFFSET, (float) xOffsetMm);
    writeFloat(REG_Y_POD_OFFSET, (float) yOffsetMm);
  }

  /**
   * Recalibrates the internal IMU gyroscope zero-offset.
   *
   * <p><strong>The robot must be completely stationary.</strong> Calibration takes ~0.25 seconds.
   */
  public void recalibrateIMU() {
    writeInt(REG_DEVICE_CONTROL, CTRL_RECALIBRATE_IMU);
  }

  /**
   * Resets the tracked position to (0, 0, 0) and recalibrates the internal IMU.
   *
   * <p><strong>The robot must be completely stationary.</strong> Calibration takes ~0.25 seconds.
   */
  public void resetPosAndIMU() {
    writeInt(REG_DEVICE_CONTROL, CTRL_RESET_POS_AND_IMU);
  }

  /**
   * Sets the direction of each encoder.
   *
   * <p>The X (forward) pod should count upward when the robot moves forward.
   * The Y (strafe) pod should count upward when the robot strafes left.
   *
   * @param xEncoder Direction for the X pod.
   * @param yEncoder Direction for the Y pod.
   */
  public void setEncoderDirections(EncoderDirection xEncoder, EncoderDirection yEncoder) {
    writeInt(REG_DEVICE_CONTROL,
        xEncoder == EncoderDirection.FORWARD ? CTRL_SET_X_ENCODER_FWD : CTRL_SET_X_ENCODER_REV);
    writeInt(REG_DEVICE_CONTROL,
        yEncoder == EncoderDirection.FORWARD ? CTRL_SET_Y_ENCODER_FWD : CTRL_SET_Y_ENCODER_REV);
  }

  /**
   * Sets encoder resolution using a goBILDA pod preset.
   *
   * @param pods The type of goBILDA odometry pod in use.
   */
  public void setEncoderResolution(GoBildaOdometryPods pods) {
    float ticksPerMm = (pods == GoBildaOdometryPods.SWINGARM_POD)
        ? TICKS_PER_MM_SWINGARM : TICKS_PER_MM_4_BAR;
    writeFloat(REG_TICKS_PER_MM, ticksPerMm);
  }

  /**
   * Sets a custom encoder resolution in ticks per millimeter.
   *
   * <p>Compute this as: {@code countsPerRevolution / wheelCircumferenceMm}.
   * Typical values are 10–100 ticks/mm.
   *
   * @param ticksPerMm Encoder resolution in ticks per millimeter.
   */
  public void setEncoderResolution(double ticksPerMm) {
    writeFloat(REG_TICKS_PER_MM, (float) ticksPerMm);
  }

  /**
   * Sets the yaw scalar applied to the IMU heading measurement.
   *
   * <p>Tuning is usually unnecessary because each device is factory-calibrated. If you need to
   * adjust, rotate the robot exactly 10 full turns and compare the measured to actual rotation.
   * Divide actual by measured and apply here. Values outside [0.95, 1.05] may indicate a
   * defective unit.
   *
   * @param yawScalar Scalar multiplied against the gyro heading output.
   */
  public void setYawScalar(double yawScalar) {
    writeFloat(REG_YAW_SCALAR, (float) yawScalar);
  }

  /**
   * Overrides the device's current estimated position.
   *
   * <p>Use this to initialize field-relative coordinates at match start, or to correct odometry
   * drift when a more accurate external measurement (e.g., AprilTags) is available.
   *
   * @param xMm      New X position in millimeters.
   * @param yMm      New Y position in millimeters.
   * @param headingRad New heading in radians.
   */
  public void setPosition(double xMm, double yMm, double headingRad) {
    writeFloat(REG_X_POSITION,    (float) xMm);
    writeFloat(REG_Y_POSITION,    (float) yMm);
    writeFloat(REG_H_ORIENTATION, (float) headingRad);
  }

  // -------------------------------------------------------------------------
  // One-shot reads (avoid calling every loop)
  // -------------------------------------------------------------------------

  /**
   * Reads the device ID register. Should return 1 for a functioning device.
   *
   * <p><em>This performs its own I2C transaction — avoid calling in a hot loop.</em>
   *
   * @return The device ID (expected: 1).
   */
  public int getDeviceID() {
    return readInt(REG_DEVICE_ID);
  }

  /**
   * Reads the firmware version from the device.
   *
   * <p><em>This performs its own I2C transaction — avoid calling in a hot loop.</em>
   *
   * @return Firmware version integer.
   */
  public int getDeviceVersion() {
    return readInt(REG_DEVICE_VERSION);
  }

  /**
   * Reads the current yaw scalar from the device.
   *
   * <p><em>This performs its own I2C transaction — avoid calling in a hot loop.</em>
   *
   * @return The yaw scalar currently stored on the device.
   */
  public float getYawScalar() {
    return readFloat(REG_YAW_SCALAR);
  }

  /**
   * Reads the configured X pod offset from the device.
   *
   * <p><em>This performs its own I2C transaction — avoid calling in a hot loop.</em>
   *
   * @return X pod offset in millimeters.
   */
  public float getXOffsetMm() {
    return readFloat(REG_X_POD_OFFSET);
  }

  /**
   * Reads the configured Y pod offset from the device.
   *
   * <p><em>This performs its own I2C transaction — avoid calling in a hot loop.</em>
   *
   * @return Y pod offset in millimeters.
   */
  public float getYOffsetMm() {
    return readFloat(REG_Y_POD_OFFSET);
  }

  // -------------------------------------------------------------------------
  // Cached getters (fast — read from memory, not I2C)
  // -------------------------------------------------------------------------

  /**
   * Returns the device status decoded from the most recent {@link #update()} call.
   *
   * @return Current {@link DeviceStatus}.
   */
  public DeviceStatus getDeviceStatus() {
    return decodeStatus(m_deviceStatusRaw);
  }

  /**
   * Returns the device's internal loop time from the most recent {@link #update()} call.
   *
   * <p>Normal range is 500–1100 µs. Values outside this range may indicate a hardware issue.
   *
   * @return Loop time in microseconds.
   */
  public int getLoopTime() {
    return m_loopTime;
  }

  /**
   * Returns the device's loop frequency derived from {@link #getLoopTime()}.
   *
   * <p>Normal range is 900–2000 Hz.
   *
   * @return Frequency in Hz, or 0 if loop time is unavailable.
   */
  public double getFrequencyHz() {
    return (m_loopTime != 0) ? 1_000_000.0 / m_loopTime : 0.0;
  }

  /**
   * Returns the raw tick count of the X (forward) encoder from the most recent update.
   *
   * @return Raw encoder ticks.
   */
  public int getEncoderX() {
    return m_xEncoderValue;
  }

  /**
   * Returns the raw tick count of the Y (strafe) encoder from the most recent update.
   *
   * @return Raw encoder ticks.
   */
  public int getEncoderY() {
    return m_yEncoderValue;
  }

  /**
   * Returns the estimated X (forward) position from the most recent update.
   *
   * @return X position in millimeters.
   */
  public double getPosXMm() {
    return m_xPosition;
  }

  /**
   * Returns the estimated Y (strafe) position from the most recent update.
   *
   * @return Y position in millimeters.
   */
  public double getPosYMm() {
    return m_yPosition;
  }

  /**
   * Returns the estimated heading (H orientation) from the most recent update, normalized to
   * [-π, π].
   *
   * @return Heading in radians.
   */
  public double getHeadingRad() {
    // Normalize to [-π, π]
    double h = m_hOrientation % (2 * Math.PI);
    if (h > Math.PI)  h -= 2 * Math.PI;
    if (h < -Math.PI) h += 2 * Math.PI;
    return h;
  }

  /**
   * Returns the raw (unnormalized, cumulative) heading from the most recent update.
   *
   * <p>This value accumulates across multiple rotations and is not wrapped to [-π, π].
   *
   * @return Heading in radians (unbounded).
   */
  public double getHeadingRadUnnormalized() {
    return m_hOrientation;
  }

  /**
   * Returns the estimated X (forward) velocity from the most recent update.
   *
   * @return X velocity in millimeters per second.
   */
  public double getVelXMmPerSec() {
    return m_xVelocity;
  }

  /**
   * Returns the estimated Y (strafe) velocity from the most recent update.
   *
   * @return Y velocity in millimeters per second.
   */
  public double getVelYMmPerSec() {
    return m_yVelocity;
  }

  /**
   * Returns the estimated heading velocity from the most recent update.
   *
   * @return Heading velocity in radians per second.
   */
  public double getHeadingVelocityRadPerSec() {
    return m_hVelocity;
  }

  // -------------------------------------------------------------------------
  // NTSendable (SmartDashboard / Shuffleboard)
  // -------------------------------------------------------------------------

  @Override
  public void initSendable(NTSendableBuilder builder) {
    builder.setSmartDashboardType("GoBildaPinpoint");

    DoublePublisher pubX   = new DoubleTopic(builder.getTopic("X_mm")).publish();
    DoublePublisher pubY   = new DoubleTopic(builder.getTopic("Y_mm")).publish();
    DoublePublisher pubH   = new DoubleTopic(builder.getTopic("Heading_rad")).publish();
    DoublePublisher pubVX  = new DoubleTopic(builder.getTopic("VelX_mm_s")).publish();
    DoublePublisher pubVY  = new DoubleTopic(builder.getTopic("VelY_mm_s")).publish();
    DoublePublisher pubVH  = new DoubleTopic(builder.getTopic("VelH_rad_s")).publish();
    DoublePublisher pubHz  = new DoubleTopic(builder.getTopic("Frequency_Hz")).publish();

    builder.addCloseable(pubX);
    builder.addCloseable(pubY);
    builder.addCloseable(pubH);
    builder.addCloseable(pubVX);
    builder.addCloseable(pubVY);
    builder.addCloseable(pubVH);
    builder.addCloseable(pubHz);

    builder.setUpdateTable(() -> {
      pubX.set(m_xPosition);
      pubY.set(m_yPosition);
      pubH.set(getHeadingRad());
      pubVX.set(m_xVelocity);
      pubVY.set(m_yVelocity);
      pubVH.set(m_hVelocity);
      pubHz.set(getFrequencyHz());
    });
  }
}