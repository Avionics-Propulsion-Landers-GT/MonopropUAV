/**
 * Stores the data from the IMU at a particular time stamp.
 *
 * @author Anyi Lin
 * @version 1.0
 */
public class IMUData {
    private double timestamp;
    private double accelX;
    private double accelY;
    private double accelZ;
    private double gyroX;
    private double gyroY;
    private double gyroZ;
    private double magX;
    private double magY;
    private double magZ;

    /**
     * Creates a new IMUData instance with the defined variables.
     *
     * @param timestamp the time stamp of the data.
     * @param accelX the x value of the accelerometer.
     * @param accelY the y value of the accelerometer.
     * @param accelZ the y value of the accelerometer.
     * @param gyroX the x value of the gyroscope.
     * @param gyroY the y value of the gyroscope.
     * @param gyroZ the z value of the gyroscope.
     * @param magX the x value of the magnetometer.
     * @param magY the y value of the magnetometer.
     * @param magZ the z value of the magnetometer.
     */
    public IMUData(double timestamp, double accelX, double accelY, double accelZ, double gyroX,
                   double gyroY, double gyroZ, double magX, double magY, double magZ) {
        this.timestamp = timestamp;
        this.accelX = accelX;
        this.accelY = accelY;
        this.accelZ = accelZ;
        this.gyroX = gyroX;
        this.gyroY = gyroY;
        this.gyroZ = gyroZ;
        this.magX = magX;
        this.magY = magY;
        this.magZ = magZ;
    }

    public double getTimestamp() {
        return timestamp;
    }

    public void setTimestamp(double timestamp) {
        this.timestamp = timestamp;
    }

    public double getAccelX() {
        return accelX;
    }

    public void setAccelX(double accelX) {
        this.accelX = accelX;
    }

    public double getAccelY() {
        return accelY;
    }

    public void setAccelY(double accelY) {
        this.accelY = accelY;
    }

    public double getAccelZ() {
        return accelZ;
    }

    public void setAccelZ(double accelZ) {
        this.accelZ = accelZ;
    }

    public double getGyroX() {
        return gyroX;
    }

    public void setGyroX(double gyroX) {
        this.gyroX = gyroX;
    }

    public double getGyroY() {
        return gyroY;
    }

    public void setGyroY(double gyroY) {
        this.gyroY = gyroY;
    }

    public double getGyroZ() {
        return gyroZ;
    }

    public void setGyroZ(double gyroZ) {
        this.gyroZ = gyroZ;
    }

    public double getMagX() {
        return magX;
    }

    public void setMagX(double magX) {
        this.magX = magX;
    }

    public double getMagY() {
        return magY;
    }

    public void setMagY(double magY) {
        this.magY = magY;
    }

    public double getMagZ() {
        return magZ;
    }

    public void setMagZ(double magZ) {
        this.magZ = magZ;
    }
}
