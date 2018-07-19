package edu.illinois.mitra.starl.modelinterfaces;

public interface DeviceControllerListener
{
    public void onDisconnect();
    public void onUpdateBattery(final byte percent);
}
