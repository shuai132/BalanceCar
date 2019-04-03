//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by Fernflower decompiler)
//

package com.juma.sdk;

import android.annotation.SuppressLint;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCallback;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattDescriptor;
import android.bluetooth.BluetoothGattService;
import android.bluetooth.BluetoothManager;
import android.content.Context;
import android.util.Log;
import java.io.InputStream;
import java.net.URL;
import java.net.URLConnection;
import java.security.SecureRandom;
import java.util.UUID;
import javax.crypto.Cipher;
import javax.crypto.KeyGenerator;
import javax.crypto.SecretKey;
import javax.crypto.spec.SecretKeySpec;
import org.apache.http.util.ByteArrayBuffer;

public class JumaDevice {
    private BluetoothManager bluetoothManager = null;
    private BluetoothAdapter bluetoothAdapter = null;
    private String name = null;
    private UUID uuid = null;
    private Context context = null;
    private JumaDeviceCallback callback = null;
    private ScanHelper scanHelper = null;
    private boolean isConnected = false;
    private boolean isConnecting = false;
    private boolean isDisconnecting = false;
    private BluetoothGatt bluetoothGatt = null;
    private BluetoothGattService bluetoothGattService = null;
    private BluetoothGattCharacteristic bluetoothGattCharacteristicCommand = null;
    private BluetoothGattCharacteristic bluetoothGattCharacteristicEvent = null;
    private BluetoothGattCharacteristic bluetoothGattCharacteristicBulkOut = null;
    private BluetoothGattCharacteristic bluetoothGattCharacteristicBulkIn = null;
    private BluetoothGattDescriptor bluetoothGattDescriptorEvent = null;
    private BluetoothGattDescriptor bluetoothGattDescriptorBulkIn = null;
    private static final UUID SERVICE_UUID = UUID.fromString("00008000-60b2-21f8-bce3-94eea697f98c");
    private static final UUID CHARACTERISTIC_UUID_COMMAND = UUID.fromString("00008001-60b2-21f8-bce3-94eea697f98c");
    private static final UUID CHARACTERISTIC_UUID_EVENT = UUID.fromString("00008002-60b2-21f8-bce3-94eea697f98c");
    private static final UUID CHARACTERISTIC_UUID_BULK_OUT = UUID.fromString("00008003-60b2-21f8-bce3-94eea697f98c");
    private static final UUID CHARACTERISTIC_UUID_BULK_IN = UUID.fromString("00008004-60b2-21f8-bce3-94eea697f98c");
    private static final UUID DESCRIPTOR_UUID = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb");
    private static final byte MESSAGE_TYPE_OTA_DATA = -127;
    private static final byte MESSAGE_TYPE_OTA_SET = -126;
    private static final byte OTA_HEADER_BEGIN = 0;
    private static final byte OTA_HEADER_END = 1;
    private static final byte OTA_HEADER_DATA = 2;
    private static final int MESSAGE_MAX_LENGTH = 198;
    private byte[] readyMessage = null;
    private byte[] firmwareData = null;
    private int index = 0;
    private boolean isUpdating = false;
    private static final String SDK_VERSION = "02.00.00.01.151203";
    public static final int SUCCESS = 0;
    public static final int ERROR = 1;
    public static final int STATE_CONNECTED = 0;
    public static final int STATE_DISCONNECTED = 1;
    private BluetoothGattCallback gattCallback = new BluetoothGattCallback() {
        public void onConnectionStateChange(BluetoothGatt gatt, int status, int newState) {
            if (status == 0) {
                if (newState == 2) {
                    if (!gatt.discoverServices()) {
                        gatt.disconnect();
                    }
                } else if (newState == 0) {
                    gatt.close();
                    if (JumaDevice.this.isConnecting) {
                        JumaDevice.this.isConnecting = false;
                        if (JumaDevice.this.callback != null) {
                            JumaDevice.this.callback.onConnectionStateChange(1, 0);
                        }

                        return;
                    }

                    if (JumaDevice.this.isDisconnecting) {
                        JumaDevice.this.isDisconnecting = false;
                    }

                    if (JumaDevice.this.isConnected) {
                        JumaDevice.this.isConnected = false;
                    }

                    (new Thread(new Runnable() {
                        public void run() {
                            try {
                                Thread.sleep(100L);
                            } catch (InterruptedException var2) {
                                var2.printStackTrace();
                            }

                            if (JumaDevice.this.callback != null) {
                                JumaDevice.this.callback.onConnectionStateChange(0, 1);
                            }

                        }
                    })).start();
                }
            } else {
                if (JumaDevice.this.isConnected) {
                    JumaDevice.this.isConnected = false;
                }

                if (JumaDevice.this.callback != null && JumaDevice.this.isConnecting) {
                    JumaDevice.this.isConnecting = false;
                    JumaDevice.this.callback.onConnectionStateChange(1, 0);
                }

                if (JumaDevice.this.isDisconnecting) {
                    JumaDevice.this.isDisconnecting = false;
                    JumaDevice.this.callback.onConnectionStateChange(1, 1);
                }

                gatt.close();
                JumaDevice.this.clear();
            }

        }

        public void onServicesDiscovered(BluetoothGatt gatt, int status) {
            if (status == 0) {
                JumaDevice.this.bluetoothGattService = gatt.getService(JumaDevice.SERVICE_UUID);
                if (JumaDevice.this.bluetoothGattService != null) {
                    JumaDevice.this.bluetoothGattCharacteristicCommand = JumaDevice.this.bluetoothGattService.getCharacteristic(JumaDevice.CHARACTERISTIC_UUID_COMMAND);
                    JumaDevice.this.bluetoothGattCharacteristicEvent = JumaDevice.this.bluetoothGattService.getCharacteristic(JumaDevice.CHARACTERISTIC_UUID_EVENT);
                    JumaDevice.this.bluetoothGattCharacteristicBulkOut = JumaDevice.this.bluetoothGattService.getCharacteristic(JumaDevice.CHARACTERISTIC_UUID_BULK_OUT);
                    JumaDevice.this.bluetoothGattCharacteristicBulkIn = JumaDevice.this.bluetoothGattService.getCharacteristic(JumaDevice.CHARACTERISTIC_UUID_BULK_IN);
                }

                if (JumaDevice.this.bluetoothGattCharacteristicEvent != null) {
                    JumaDevice.this.bluetoothGattDescriptorEvent = JumaDevice.this.bluetoothGattCharacteristicEvent.getDescriptor(JumaDevice.DESCRIPTOR_UUID);
                }

                if (JumaDevice.this.bluetoothGattCharacteristicBulkIn != null) {
                    JumaDevice.this.bluetoothGattDescriptorBulkIn = JumaDevice.this.bluetoothGattCharacteristicBulkIn.getDescriptor(JumaDevice.DESCRIPTOR_UUID);
                }

                if (JumaDevice.this.bluetoothGattService != null && JumaDevice.this.bluetoothGattCharacteristicCommand != null && JumaDevice.this.bluetoothGattCharacteristicEvent != null && JumaDevice.this.bluetoothGattCharacteristicBulkOut != null && JumaDevice.this.bluetoothGattCharacteristicBulkIn != null && JumaDevice.this.bluetoothGattDescriptorEvent != null && JumaDevice.this.bluetoothGattDescriptorBulkIn != null) {
                    JumaDevice.this.bluetoothGattDescriptorEvent.setValue(BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE);
                    gatt.setCharacteristicNotification(JumaDevice.this.bluetoothGattCharacteristicEvent, true);
                    if (!gatt.writeDescriptor(JumaDevice.this.bluetoothGattDescriptorEvent)) {
                        gatt.disconnect();
                    }
                } else {
                    Log.e(JumaDevice.class.getName(), "Does not support the device = " + JumaDevice.this.uuid.toString());
                    gatt.disconnect();
                }
            } else if (status == 133) {
                JumaDevice.this.isConnecting = false;
                if (JumaDevice.this.callback != null) {
                    JumaDevice.this.callback.onConnectionStateChange(1, 0);
                }

                gatt.close();
                JumaDevice.this.clear();
            } else {
                gatt.disconnect();
            }

        }

        public void onDescriptorWrite(BluetoothGatt gatt, BluetoothGattDescriptor descriptor, int status) {
            if (status == 0) {
                if (descriptor.getCharacteristic().getUuid().equals(JumaDevice.CHARACTERISTIC_UUID_EVENT)) {
                    JumaDevice.this.bluetoothGattDescriptorBulkIn.setValue(BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE);
                    gatt.setCharacteristicNotification(JumaDevice.this.bluetoothGattCharacteristicBulkIn, true);
                    if (!gatt.writeDescriptor(JumaDevice.this.bluetoothGattDescriptorBulkIn)) {
                        gatt.disconnect();
                    }
                } else if (descriptor.getCharacteristic().getUuid().equals(JumaDevice.CHARACTERISTIC_UUID_BULK_IN)) {
                    JumaDevice.this.isConnecting = false;
                    JumaDevice.this.isConnected = true;
                    JumaDevice.this.bluetoothGatt = gatt;
                    if (JumaDevice.this.callback != null) {
                        JumaDevice.this.callback.onConnectionStateChange(0, 0);
                    }
                }
            } else if (status == 133) {
                JumaDevice.this.isConnecting = false;
                if (JumaDevice.this.callback != null) {
                    JumaDevice.this.callback.onConnectionStateChange(1, 0);
                }

                gatt.close();
                JumaDevice.this.clear();
            } else {
                gatt.disconnect();
            }

        }

        public void onCharacteristicWrite(BluetoothGatt gatt, BluetoothGattCharacteristic characteristic, int status) {
            if (status == 0) {
                if (characteristic.getUuid().equals(JumaDevice.CHARACTERISTIC_UUID_COMMAND)) {
                    if (JumaDevice.this.readyMessage != null && JumaDevice.this.readyMessage.length > 0) {
                        while(JumaDevice.this.readyMessage != null) {
                            JumaDevice.this.bluetoothGattCharacteristicBulkOut.setValue(JumaDevice.this.readPacket(JumaDevice.this.readyMessage));
                            if (!JumaDevice.this.bluetoothGatt.writeCharacteristic(JumaDevice.this.bluetoothGattCharacteristicBulkOut)) {
                                if (JumaDevice.this.callback != null) {
                                    JumaDevice.this.callback.onSend(1);
                                }

                                return;
                            }

                            JumaDevice.this.readyMessage = JumaDevice.this.updateMessage(JumaDevice.this.readyMessage);
                        }
                    }

                    if (JumaDevice.this.readyMessage == null && !JumaDevice.this.isUpdating && JumaDevice.this.callback != null) {
                        JumaDevice.this.callback.onSend(0);
                    }
                }
            } else if (!JumaDevice.this.isUpdating) {
                if (JumaDevice.this.callback != null) {
                    JumaDevice.this.callback.onSend(1);
                }
            } else {
                JumaDevice.this.isUpdating = false;
                if (JumaDevice.this.callback != null) {
                    JumaDevice.this.callback.onUpdateFirmware(1);
                }
            }

        }

        public void onCharacteristicChanged(BluetoothGatt gatt, BluetoothGattCharacteristic characteristic) {
            byte type;
            if (characteristic.getUuid().equals(JumaDevice.CHARACTERISTIC_UUID_EVENT)) {
                byte msgType = characteristic.getValue()[0];
                if (msgType == -127 && JumaDevice.this.isUpdating) {
                    type = characteristic.getValue()[2];
                    switch(type) {
                    case 0:
                        if (JumaDevice.this.firmwareData != null) {
                            if (!JumaDevice.this.readySend((byte)-127, JumaDevice.this.readFirmwarePacket(JumaDevice.this.firmwareData))) {
                                JumaDevice.this.isUpdating = false;
                                JumaDevice.this.firmwareData = null;
                                if (JumaDevice.this.callback != null) {
                                    JumaDevice.this.callback.onUpdateFirmware(1);
                                }

                                return;
                            }

                            JumaDevice.this.index = JumaDevice.this.index + 1;
                            JumaDevice.this.firmwareData = JumaDevice.this.updateFirmwareData(JumaDevice.this.firmwareData);
                        } else {
                            JumaDevice.this.isUpdating = false;
                            JumaDevice.this.firmwareData = null;
                            if (JumaDevice.this.callback != null) {
                                JumaDevice.this.callback.onUpdateFirmware(1);
                            }
                        }
                    case 1:
                    default:
                        break;
                    case 2:
                        if (JumaDevice.this.firmwareData == null) {
                            if (JumaDevice.this.readySend((byte)-127, new byte[]{1})) {
                                JumaDevice.this.isUpdating = false;
                                if (JumaDevice.this.callback != null) {
                                    JumaDevice.this.callback.onUpdateFirmware(0);
                                }
                            } else {
                                JumaDevice.this.isUpdating = false;
                                JumaDevice.this.firmwareData = null;
                                if (JumaDevice.this.callback != null) {
                                    JumaDevice.this.callback.onUpdateFirmware(1);
                                }
                            }
                        } else {
                            if (!JumaDevice.this.readySend((byte)-127, JumaDevice.this.readFirmwarePacket(JumaDevice.this.firmwareData))) {
                                JumaDevice.this.isUpdating = false;
                                JumaDevice.this.firmwareData = null;
                                if (JumaDevice.this.callback != null) {
                                    JumaDevice.this.callback.onUpdateFirmware(1);
                                }
                            }

                            JumaDevice.this.index = JumaDevice.this.index + 1;
                            JumaDevice.this.firmwareData = JumaDevice.this.updateFirmwareData(JumaDevice.this.firmwareData);
                        }
                    }

                    return;
                }
            }

            byte[] message = characteristic.getValue();
            type = message[0];
            byte[] pureMessage = new byte[message.length - 2];

            for(int i = 0; i < pureMessage.length; ++i) {
                pureMessage[i] = message[i + 2];
            }

            if (Integer.parseInt(JumaDevice.toHex(new byte[]{type})) < 128 && JumaDevice.this.callback != null) {
                JumaDevice.this.callback.onReceive(type, pureMessage);
            }

        }

        public void onReadRemoteRssi(BluetoothGatt gatt, int rssi, int status) {
            if (status == 0) {
                if (JumaDevice.this.callback != null) {
                    JumaDevice.this.callback.onRemoteRssi(0, rssi);
                }
            } else if (status == 133) {
                if (JumaDevice.this.callback != null) {
                    JumaDevice.this.callback.onRemoteRssi(1, rssi);
                }

                JumaDevice.this.callback.onConnectionStateChange(1, 0);
                gatt.close();
                JumaDevice.this.clear();
            }

        }
    };

    JumaDevice(Context context, ScanHelper scanHelper, String name, UUID uuid) {
        this.name = name;
        this.uuid = uuid;
        this.context = context;
        this.scanHelper = scanHelper;
    }

    public String getName() {
        return this.name;
    }

    public UUID getUuid() {
        return this.uuid;
    }

    public synchronized boolean connect(JumaDeviceCallback callback) {
        if (this.isConnecting) {
            return false;
        } else {
            if (this.bluetoothManager == null) {
                this.bluetoothManager = (BluetoothManager)this.context.getSystemService("bluetooth");
            }

            if (this.bluetoothAdapter == null) {
                this.bluetoothAdapter = this.bluetoothManager.getAdapter();
            }

            if (!this.checkBluetoothState()) {
                return false;
            } else if (this.scanHelper.isScanning() && !this.scanHelper.stopScan()) {
                return false;
            } else {
                this.callback = callback;
                if (this.isConnected) {
                    if (callback != null) {
                        callback.onConnectionStateChange(0, 0);
                    }

                    return true;
                } else {
                    this.isConnecting = true;

                    try {
                        if (this.bluetoothGatt != null) {
                            this.bluetoothGatt.close();
                        }

                        this.bluetoothGatt = this.bluetoothAdapter.getRemoteDevice(this.IDDecrypt(this.uuid.toString(), this.context)).connectGatt(this.context, false, this.gattCallback);
                        return true;
                    } catch (Exception var3) {
                        this.isConnecting = false;
                        return false;
                    }
                }
            }
        }
    }

    public boolean disconnect() {
        if (this.isDisconnecting) {
            return false;
        } else if (!this.checkBluetoothState()) {
            return false;
        } else if (this.isConnected && this.bluetoothGatt != null) {
            this.isDisconnecting = true;
            this.bluetoothGatt.disconnect();
            return true;
        } else {
            return false;
        }
    }

    public boolean getRemoteRssi() {
        if (!this.checkBluetoothState()) {
            return false;
        } else if (this.isConnected && this.bluetoothGatt != null) {
            this.bluetoothGatt.readRemoteRssi();
            return true;
        } else {
            return false;
        }
    }

    public boolean send(byte type, byte[] message) {
        return Integer.parseInt(toHex(new byte[]{type}), 16) > 128 ? false : this.readySend(type, message);
    }

    private boolean readySend(byte type, byte[] message) {
        if (!this.checkBluetoothState()) {
            return false;
        } else if (this.readyMessage != null) {
            return false;
        } else if (message.length > 198) {
            return false;
        } else if (this.isConnected && this.bluetoothGatt != null) {
            this.readyMessage = this.addMessageHead(type, message);
            this.bluetoothGattCharacteristicCommand.setValue(this.readPacket(this.readyMessage));
            if (!this.bluetoothGatt.writeCharacteristic(this.bluetoothGattCharacteristicCommand)) {
                this.readyMessage = null;
                return false;
            } else {
                this.readyMessage = this.updateMessage(this.readyMessage);
                return true;
            }
        } else {
            return false;
        }
    }

    public static String getVersion() {
        return "02.00.00.01.151203";
    }

    public boolean setOtaMode() {
        return this.readySend((byte)-126, new byte[]{79, 84, 65, 95, 77, 79, 68, 69, 0});
    }

    public boolean updateFirmware(String url) {
        if (this.isUpdating) {
            return false;
        } else {
            this.isUpdating = true;
            (new JumaDevice.DownloadThread(url, new JumaDevice.DownloadCallback() {
                public void onDownload(boolean state, byte[] data) {
                    if (!state) {
                        if (JumaDevice.this.callback != null) {
                            JumaDevice.this.callback.onUpdateFirmware(1);
                        }

                        JumaDevice.this.isUpdating = false;
                        JumaDevice.this.firmwareData = null;
                    } else {
                        JumaDevice.this.firmwareData = data;
                        JumaDevice.this.index = 0;
                        if (!JumaDevice.this.readySend((byte)-127, new byte[1])) {
                            JumaDevice.this.isUpdating = false;
                            JumaDevice.this.firmwareData = null;
                            if (JumaDevice.this.callback != null) {
                                JumaDevice.this.callback.onUpdateFirmware(1);
                            }
                        }
                    }

                }
            })).start();
            return true;
        }
    }

    public boolean isFirmwareUpdating() {
        return this.isUpdating;
    }

    public boolean isConnected() {
        return this.isConnected;
    }

    private boolean checkBluetoothState() {
        return this.bluetoothAdapter != null && this.bluetoothAdapter.isEnabled();
    }

    private void clear() {
        this.bluetoothGatt = null;
        this.bluetoothGattService = null;
        this.bluetoothGattCharacteristicCommand = null;
        this.bluetoothGattCharacteristicEvent = null;
        this.bluetoothGattCharacteristicBulkIn = null;
        this.bluetoothGattCharacteristicBulkOut = null;
        this.bluetoothGattDescriptorEvent = null;
        this.bluetoothGattDescriptorBulkIn = null;
    }

    private byte[] addMessageHead(byte type, byte[] message) {
        if (message == null) {
            message = new byte[0];
        }

        ByteArrayBuffer buffer = new ByteArrayBuffer(200);
        buffer.append(type);
        buffer.append(message.length);

        for(int i = 0; i < message.length; ++i) {
            buffer.append(message[i]);
        }

        return buffer.toByteArray();
    }

    private byte[] readPacket(byte[] readyMessage) {
        byte[] packet = null;
        if (readyMessage.length <= 20) {
            packet = new byte[readyMessage.length];
        } else {
            packet = new byte[20];
        }

        for(int i = 0; i < packet.length; ++i) {
            packet[i] = readyMessage[i];
        }

        return packet;
    }

    private byte[] updateMessage(byte[] readyMessage) {
        if (readyMessage.length <= 20) {
            return null;
        } else {
            ByteArrayBuffer buffer = new ByteArrayBuffer(200);

            for(int i = 20; i < readyMessage.length; ++i) {
                buffer.append(readyMessage[i]);
            }

            return buffer.toByteArray();
        }
    }

    private byte[] updateFirmwareData(byte[] firmwateData) {
        if (firmwateData.length <= 196) {
            return null;
        } else {
            ByteArrayBuffer buffer = new ByteArrayBuffer(firmwateData.length - 196);

            for(int i = 196; i < firmwateData.length; ++i) {
                buffer.append(firmwateData[i]);
            }

            return buffer.toByteArray();
        }
    }

    private byte[] readFirmwarePacket(byte[] firmwateData) {
        byte[] buffer = null;
        if (firmwateData == null) {
            return (byte[])buffer;
        } else {
            if (firmwateData.length <= 196) {
                buffer = new byte[firmwateData.length + 2];
            } else {
                buffer = new byte[198];
            }

            buffer[0] = 2;
            buffer[1] = (byte)this.index;

            for(int i = 0; i < buffer.length - 2; ++i) {
                buffer[i + 2] = firmwateData[i];
            }

            return buffer;
        }
    }

    @SuppressLint({"DefaultLocale"})
    private String IDDecrypt(String encryptedId, Context context) throws Exception {
        StringBuffer sb = new StringBuffer(encryptedId.toUpperCase());

        for(int i = 0; i < 4; ++i) {
            sb.deleteCharAt(8 + i * 4);
        }

        StringBuffer sb2 = null;
        sb2 = new StringBuffer(this.decrypt(this.getLocalBluetoothMAC(context), sb.toString()));

        for(int i = 0; i < 5; ++i) {
            sb2.insert(2 + i * 3, ":");
        }

        return sb2.toString();
    }

    private String decrypt(String seed, String encrypted) throws Exception {
        byte[] rawKey = this.getRawKey(seed.getBytes());
        byte[] enc = this.toByte(encrypted);
        byte[] result = this.decrypt(rawKey, enc);
        return new String(result);
    }

    private byte[] getRawKey(byte[] seed) throws Exception {
        KeyGenerator kgen = KeyGenerator.getInstance("AES");
        SecureRandom sr = SecureRandom.getInstance("SHA1PRNG", "Crypto");
        sr.setSeed(seed);
        kgen.init(128, sr);
        SecretKey skey = kgen.generateKey();
        byte[] raw = skey.getEncoded();
        return raw;
    }

    private byte[] decrypt(byte[] raw, byte[] encrypted) throws Exception {
        SecretKeySpec skeySpec = new SecretKeySpec(raw, "AES");
        Cipher cipher = Cipher.getInstance("AES");
        cipher.init(2, skeySpec);
        byte[] decrypted = cipher.doFinal(encrypted);
        return decrypted;
    }

    private byte[] toByte(String hexString) {
        int len = hexString.length() / 2;
        byte[] result = new byte[len];

        for(int i = 0; i < len; ++i) {
            result[i] = Integer.valueOf(hexString.substring(2 * i, 2 * i + 2), 16).byteValue();
        }

        return result;
    }

    private static String toHex(byte[] buf) {
        if (buf == null) {
            return "";
        } else {
            StringBuffer result = new StringBuffer(2 * buf.length);
            String HEX = "0123456789ABCDEF";

            for(int i = 0; i < buf.length; ++i) {
                result.append(HEX.charAt(buf[i] >> 4 & 15)).append(HEX.charAt(buf[i] & 15));
            }

            return result.toString();
        }
    }

    private String getLocalBluetoothMAC(Context context) {
        BluetoothManager manager = (BluetoothManager)context.getSystemService("bluetooth");
        BluetoothAdapter adapter = manager.getAdapter();
        String mac = adapter.getAddress();
        return mac;
    }

    private interface DownloadCallback {
        void onDownload(boolean var1, byte[] var2);
    }

    private class DownloadThread extends Thread {
        private JumaDevice.DownloadCallback downloadCallback = null;
        private String url = null;

        public DownloadThread(String url, JumaDevice.DownloadCallback callback) {
            this.url = url;
            this.downloadCallback = callback;
        }

        public void run() {
            try {
                URLConnection connection = (new URL(this.url)).openConnection();
                byte[] buffer = new byte[connection.getContentLength()];
                InputStream is = connection.getInputStream();
                is.read(buffer);
                is.close();
                this.downloadCallback.onDownload(true, buffer);
            } catch (Exception var4) {
                this.downloadCallback.onDownload(false, (byte[])null);
            }

        }
    }
}
