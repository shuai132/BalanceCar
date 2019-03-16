package com.juma.stsensor;

import android.app.Activity;
import android.content.Intent;
import android.graphics.Color;
import android.os.Bundle;
import android.os.Message;
import android.support.v4.content.LocalBroadcastManager;
import android.util.Log;
import android.view.KeyEvent;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.Window;
import android.widget.Button;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.Toast;

import com.juma.sdk.JumaDevice;
import com.juma.sdk.JumaDeviceCallback;
import com.juma.sdk.ScanHelper;
import com.juma.sdk.ScanHelper.ScanCallback;
import com.juma.stsensor.CustomDialog.Callback;

import java.text.DecimalFormat;
import java.util.HashMap;
import java.util.UUID;

import static android.content.ContentValues.TAG;

public class STSensor extends Activity {
	private Button btStart;
	private TextView tv1, tv2, tv3, tv;
	private SeekBar pb1, pb2, pb3;
	private ScanHelper scanner;
	private JumaDevice myDevice;
	private boolean canReceive = true;
	private HashMap<UUID, JumaDevice> deviceList = new HashMap<UUID, JumaDevice>();
	public static final String ACTION_DEVICE_DISCOVERED = "com.example.temperaturegatheringdemo.ACTION_DEVICE_DISCOVERED";
	DecimalFormat df;

	private Button bt_stop;
	private Rudder rudder;
	/**
	 * ATTENTION: This was auto-generated to implement the App Indexing API.
	 * See https://g.co/AppIndexing/AndroidStudio for more information.
	 */
	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		requestWindowFeature(Window.FEATURE_NO_TITLE);
		setContentView(R.layout.activity_info);
		initView();
		scanDevice();
		clicked();
		backgroundSend();
	}

	@Override
	protected void onDestroy() {
		super.onDestroy();
		if (myDevice != null && myDevice.isConnected())
			myDevice.disconnect();
	}

	@Override
	public boolean onKeyDown(int keyCode, KeyEvent event) {
		if (keyCode == KeyEvent.KEYCODE_BACK && event.getRepeatCount() == 0) {
			exit();
			return true;
		}
		return super.onKeyDown(keyCode, event);
	}

	private long mExitTime;
	public void exit() {
		if ((System.currentTimeMillis() - mExitTime) > 2000) {
			Toast.makeText(this, "再按一次退出~", Toast.LENGTH_SHORT).show();
			mExitTime = System.currentTimeMillis();
		} else {
			finish();
		}
	}

	//后台发送
	private android.os.Handler handler = new android.os.Handler() {
		public void handleMessage(android.os.Message msg) {
			switch (msg.what) {
				case 0:
					Log.d(TAG, "handleMessage: time out...");
					if (myDevice != null && myDevice.isConnected()) {
						if(rudder.getPressStatus()){
							//如果方向盘是按下状态
							int angle = rudder.getAngle();
							String value = angle + "°";
							myDevice.send((byte) 0, value.getBytes());
						}
						else {
							//如果方向盘是松开状态
							String value = "0°";
							myDevice.send((byte) 2, value.getBytes());
						}
					}

				default:
					break;
			}
		};
	};

	private void backgroundSend(){
		Thread thread = new Thread(new Runnable() {
			@Override
			public void run() {
				while(true){
					try{
						Thread.sleep(1000);
						Message msg = new Message();
						msg.what = 0;
						handler.sendMessage(msg);
					}catch (Exception e){
						Log.d(TAG, "run: ", e);
					}
				}
			}
		});
		thread.start();
	}

	private int temputere, humidity, quantity;
	private byte Type;
	private JumaDeviceCallback callback = new JumaDeviceCallback() {
		@Override
		public void onConnectionStateChange(int status, int newState) {
			super.onConnectionStateChange(status, newState);
			if (newState == JumaDevice.STATE_CONNECTED && status == JumaDevice.SUCCESS) {
				runOnUiThread(new Runnable() {

					@Override
					public void run() {
						btStart.setText("Receiving");
						btStart.setBackgroundResource(R.drawable.bt_click);
						btStart.setEnabled(true);
					}

				});
			} else {
				runOnUiThread(new Runnable() {

					@Override
					public void run() {
						btStart.setText("Select Device");
						btStart.setBackgroundResource(R.drawable.bt_click);
						btStart.setEnabled(true);
						TextView[] tv = {tv1, tv2, tv3};
						SeekBar[] pb = {pb1, pb2, pb3};
						for (int i = 0; i < tv.length; i++) {
							tv[i].setText("");
							pb[i].setProgress(0);
						}
					}

				});
			}
		}

		@Override
		public void onReceive(byte type, byte[] message) {

			super.onReceive(type, message);
			if (canReceive) {
				canReceive = false;
				Type = type;

				switch (type) {
					case 0x00:
						temputere = message[0];
						temputere <<= 8;
						temputere |= (message[1] & 0x00FF);
						break;
					case 0x01:
						humidity = message[0];
						humidity <<= 8;
						humidity |= (message[1] & 0x00FF);
						break;
					case 0x02:
						quantity = message[0];
						break;
				}

				runOnUiThread(new Runnable() {
					@Override
					public void run() {
						switch (Type) {
							case 0x00:	//温度
								tv1.setText((double) temputere / 100 + "℃");
								pb1.setProgress(temputere / 100);
								break;
							case 0x01:	//湿度
								tv2.setText((double) humidity / 100 + "RH%");
								pb2.setProgress(humidity / 100);
								break;
							case 0x02:	//电量
								if(quantity <= 15) {
									tv3.setText(quantity + "%" + "\n哎呀 快没电了~\n快给宝宝充电哦~");
								}
								else {
									tv3.setText(quantity + "%");
								}
								pb3.setProgress(quantity);
								break;
						}
						canReceive = true;
					}
				});
			}
		}
	};

	private void clicked() {
		btStart.setOnClickListener(new OnClickListener() {

			@Override
			public void onClick(View arg0) {

				if (btStart.getText().equals("Receiving")) {
					myDevice.disconnect();
				} else {
					deviceList.clear();
					scanner.startScan(null);
					final CustomDialog scanDialog = new CustomDialog(STSensor.this, R.style.NobackDialog);
					scanDialog.setScanCallback(new Callback() {

						@Override
						public void onDevice(final UUID uuid, final String name) {
							scanner.stopScan();
							myDevice = deviceList.get(uuid);
							btStart.setEnabled(false);
							btStart.setBackgroundColor(Color.argb(0x20, 0x00, 0x00, 0x00));
							btStart.setText("Connecting");
							myDevice.connect(callback);
						}

						@Override
						public void onDismiss() {
							scanner.stopScan();
						}
					});

					scanDialog.setNegativeButton(new OnClickListener() {

						@Override
						public void onClick(View arg0) {
							scanDialog.dismiss();

						}
					});
					scanDialog.show();
				}
			}
		});
	}

	private void initView() {
		tv1 = (TextView) findViewById(R.id.tv1);
		tv2 = (TextView) findViewById(R.id.tv2);
		tv3 = (TextView) findViewById(R.id.tv3);
		pb1 = (SeekBar) findViewById(R.id.pb1);
		pb2 = (SeekBar) findViewById(R.id.pb2);
		pb3 = (SeekBar) findViewById(R.id.pb3);
		btStart = (Button) findViewById(R.id.bt_start);
		df = new DecimalFormat("######0.0");

		bt_stop = (Button) findViewById(R.id.bt_stop);

		rudder = (Rudder)findViewById(R.id.rudder);
		tv = (TextView)findViewById(R.id.textView3);

		rudder.setRudderListener(new Rudder.RudderListener() {
			@Override
			public void onSteeringWheelChanged(int action, int angle) {
				if(action == Rudder.ACTION_RUDDER) {
					//TO DO
					if (myDevice != null && myDevice.isConnected()) {
						String value = angle + "°";
						tv.setText(value);
						myDevice.send((byte) 0, value.getBytes());
					} else {
						tv.setText("~~蓝牙未连接~~");
					}
				}
			}

			@Override
			public void onSteeringWheelRelease() {
				tv.setText("");
				if (myDevice != null && myDevice.isConnected()) {
					String value = "0°";
					myDevice.send((byte) 0, value.getBytes());
					myDevice.send((byte) 0, value.getBytes());
					myDevice.send((byte) 0, value.getBytes());
				}
			}
		});

		bt_stop.setOnClickListener(new OnClickListener() {
			@Override
			public void onClick(View v) {
				if (myDevice != null && myDevice.isConnected()) {
					//发送消息让小车停止
					myDevice.send((byte) 3, "\0".getBytes());
				}
			}
		});
	}

	private void scanDevice() {
		scanner = new ScanHelper(getApplicationContext(), new ScanCallback() {

			@Override
			public void onDiscover(JumaDevice device, int rssi) {
				if (!deviceList.containsKey(device.getUuid())) {
					deviceList.put(device.getUuid(), device);
				}
				Intent intent = new Intent(STSensor.ACTION_DEVICE_DISCOVERED);
				intent.putExtra("name", device.getName());
				intent.putExtra("uuid", device.getUuid().toString());
				intent.putExtra("rssi", rssi);
				LocalBroadcastManager.getInstance(getApplicationContext()).sendBroadcast(intent);
			}

			@Override
			public void onScanStateChange(int arg0) {
			}
		});
	}
}
