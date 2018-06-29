package edu.illinois.mitra.starl.objects;

public class Rotation {
    public Rotation() {
        setYaw(0.0);
        setPitch(0.0);
        setRoll(0.0);
    }

    public Rotation(double yaw, double pitch, double roll) {
        setYaw(yaw);
        setPitch(pitch);
        setRoll(roll);
    }

    public double getYaw() {
        return yaw;
    }

    public void setYaw(double yaw) {
        this.yaw = yaw;
    }

    public double getPitch() {
        return pitch;
    }

    public void setPitch(double pitch) {
        this.pitch = pitch;
    }

    public double getRoll() {
        return roll;
    }

    public void setRoll(double roll) {
        this.roll = roll;
    }

    public void rotateBy(Rotation other) {
        setYaw(getYaw() + other.getYaw());
        setPitch(getPitch() + other.getPitch());
        setRoll(getRoll() + other.getRoll());
    }

    // Private fields
    private double yaw;
    private double pitch;
    private double roll;
}
