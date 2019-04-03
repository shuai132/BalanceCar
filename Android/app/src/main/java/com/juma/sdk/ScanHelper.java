//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by Fernflower decompiler)
//

package com.juma.sdk;

import android.annotation.SuppressLint;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothManager;
import android.bluetooth.BluetoothAdapter.LeScanCallback;
import android.content.Context;
import java.security.SecureRandom;
import java.util.ArrayList;
import java.util.List;
import java.util.UUID;
import javax.crypto.Cipher;
import javax.crypto.KeyGenerator;
import javax.crypto.SecretKey;
import javax.crypto.spec.SecretKeySpec;

public class ScanHelper {
    private Context context = null;
    private BluetoothAdapter bluetoothAdapter = null;
    private boolean isScanning = false;
    private String name = null;
    private ScanHelper.ScanCallback callback = null;
    private static final String JUMA_SERVICE_UUID = "90FE";
    public static final int STATE_START_SCAN = 0;
    public static final int STATE_STOP_SCAN = 1;
    private static final String SDK_VERSION = "02.00.00.01.151203";
    private LeScanCallback leScanCallback = new LeScanCallback() {
        public void onLeScan(BluetoothDevice device, int rssi, byte[] scanRecord) {
            if (ScanHelper.this.getServiceUuids(scanRecord).contains("90FE")) {
                if (ScanHelper.this.name == null || ScanHelper.this.name.equals("") || ScanHelper.this.name.equals(device.getName())) {
                    ScanHelper.this.callback.onDiscover(new JumaDevice(ScanHelper.this.context, ScanHelper.this, device.getName(), UUID.fromString(ScanHelper.this.IDEncrypt(device.getAddress(), ScanHelper.this.context))), rssi);
                }

            }
        }
    };

    public ScanHelper(Context context, ScanHelper.ScanCallback callback) {
        this.context = context;
        this.callback = callback;
        BluetoothManager bluetoothManager = (BluetoothManager)context.getSystemService("bluetooth");
        this.bluetoothAdapter = bluetoothManager.getAdapter();
    }

    public boolean startScan(String name) {
        if (!this.checkBluetoothState()) {
            return false;
        } else {
            this.name = name;
            if (!this.isScanning) {
                if (this.bluetoothAdapter.startLeScan(this.leScanCallback)) {
                    this.isScanning = true;
                    this.updateScanState(0);
                    return true;
                } else {
                    return false;
                }
            } else {
                this.updateScanState(0);
                return true;
            }
        }
    }

    public boolean stopScan() {
        if (!this.checkBluetoothState()) {
            return false;
        } else {
            if (this.isScanning) {
                this.isScanning = false;
                this.bluetoothAdapter.stopLeScan(this.leScanCallback);
                (new Thread(new Runnable() {
                    public void run() {
                        try {
                            Thread.sleep(100L);
                        } catch (InterruptedException var2) {
                            var2.printStackTrace();
                        }

                        ScanHelper.this.updateScanState(1);
                    }
                })).start();
            } else {
                this.updateScanState(1);
            }

            return true;
        }
    }

    public boolean isScanning() {
        return this.isScanning;
    }

    public boolean isEnabled() {
        return this.bluetoothAdapter == null ? false : this.bluetoothAdapter.isEnabled();
    }

    public boolean enable() {
        return this.bluetoothAdapter == null ? false : this.bluetoothAdapter.enable();
    }

    public boolean disable() {
        return this.bluetoothAdapter == null ? false : this.bluetoothAdapter.disable();
    }

    public static String getVersion() {
        return "02.00.00.01.151203";
    }

    private void updateScanState(int status) {
        if (this.callback != null) {
            this.callback.onScanStateChange(status);
        }

    }

    private boolean checkBluetoothState() {
        return this.bluetoothAdapter != null && this.bluetoothAdapter.isEnabled();
    }

    private List<String> getServiceUuids(byte[] advData) {
        List<String> uuids = new ArrayList();

        byte[] mByte;
        for(boolean isOver = true; isOver; advData = mByte) {
            int dataLen = advData[0];
            if (dataLen == 0) {
                isOver = false;
                break;
            }

            byte[] allData = new byte[dataLen];

            for(int i = 0; i < allData.length; ++i) {
                allData[i] = advData[i + 1];
            }

            byte[] type = new byte[]{allData[0]};
            byte[] data = new byte[allData.length - 1];

            int number;
            for(number = 0; number < data.length; ++number) {
                data[number] = allData[number + 1];
            }

            int i;
            if ((255 & type[0]) == 2) {
                mByte = new byte[data.length];

                for(i = 0; i < mByte.length; ++i) {
                    mByte[i] = data[data.length - i - 1];
                }

                uuids.add(this.toHex(mByte));
            } else {
                if ((255 & type[0]) == 3) {
                    number = data.length / 2;

                    for(i = 0; i < number; ++i) {
                        mByte = new byte[]{data[i * 2], data[i * 2 + 1]};
                        uuids.add(this.toHex(mByte));
                    }
                } else if ((255 & type[0]) == 4) {
                    mByte = new byte[data.length];

                    for(i = 0; i < mByte.length; ++i) {
                        mByte[i] = data[data.length - i - 1];
                    }

                    uuids.add(this.toHex(mByte));
                } else if ((255 & type[0]) == 5) {
                    number = data.length / 4;

                    for(i = 0; i < number; ++i) {
                        mByte = new byte[]{data[i * 4], data[i * 4 + 1], data[i * 4 + 2], data[i * 4 + 3]};
                        uuids.add(this.toHex(mByte));
                    }
                } else if ((255 & type[0]) == 6) {
                    mByte = new byte[data.length];

                    for(i = 0; i < mByte.length; ++i) {
                        mByte[i] = data[data.length - i - 1];
                    }

                    uuids.add(this.toHex(mByte));
                } else if ((255 & type[0]) == 7) {
                    number = data.length / 16;

                    for(i = 0; i < number; ++i) {
                        mByte = new byte[]{data[i * 16], data[i * 16 + 1], data[i * 16 + 2], data[i * 16 + 3], data[i * 16 + 4], data[i * 16 + 5], data[i * 16 + 6], data[i * 16 + 7], data[i * 16 + 8], data[i * 16 + 9], data[i * 16 + 10], data[i * 16 + 11], data[i * 16 + 12], data[i * 16 + 13], data[i * 16 + 14], data[i * 16 + 15]};
                        uuids.add(this.toHex(mByte));
                    }
                }
            }

            mByte = new byte[advData.length - dataLen - 1];

            for(i = 0; i < mByte.length; ++i) {
                mByte[i] = advData[i + 1 + dataLen];
            }
        }

        return uuids;
    }

    private void appendHex(StringBuffer sb, byte b) {
        String HEX = "0123456789ABCDEF";
        sb.append(HEX.charAt(b >> 4 & 15)).append(HEX.charAt(b & 15));
    }

    @SuppressLint({"DefaultLocale"})
    private String IDEncrypt(String cleartextId, Context context) {
        StringBuffer sb = new StringBuffer(cleartextId);

        for(int i = 2; i < sb.length(); i += 2) {
            sb.deleteCharAt(i);
        }

        StringBuffer sb2 = null;

        try {
            sb2 = new StringBuffer(this.encrypt(this.getLocalBluetoothMAC(context), sb.toString()));
        } catch (Exception var6) {
            var6.printStackTrace();
        }

        for(int i = 0; i < 4; ++i) {
            sb2.insert(8 + i * 5, "-");
        }

        return sb2.toString().toLowerCase();
    }

    private String encrypt(String seed, String cleartext) throws Exception {
        byte[] rawKey = this.getRawKey(seed.getBytes());
        byte[] result = this.encrypt(rawKey, cleartext.getBytes());
        return this.toHex(result);
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

    private byte[] encrypt(byte[] raw, byte[] clear) throws Exception {
        SecretKeySpec skeySpec = new SecretKeySpec(raw, "AES");
        Cipher cipher = Cipher.getInstance("AES");
        cipher.init(1, skeySpec);
        byte[] encrypted = cipher.doFinal(clear);
        return encrypted;
    }

    private String toHex(byte[] buf) {
        if (buf == null) {
            return "";
        } else {
            StringBuffer result = new StringBuffer(2 * buf.length);

            for(int i = 0; i < buf.length; ++i) {
                this.appendHex(result, buf[i]);
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

    public interface ScanCallback {
        void onDiscover(JumaDevice var1, int var2);

        void onScanStateChange(int var1);
    }
}
