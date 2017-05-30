package com.example.somatosensory;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.Set;
import java.util.UUID;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.Context;
import android.os.Handler;
import android.os.Message;
import android.widget.Toast;

public class BlueTooth {

	public boolean threadFlag = true;
	ReceviveThread myThread;

	private BluetoothAdapter mBluetoothAdapter = null;
	private BluetoothSocket btSocket = null;
	private static OutputStream outStream = null;
	private InputStream inStream = null;
	static public boolean bluetoothFlag = true;

	Set<BluetoothDevice> pairedDevices;
	ArrayList<String> BlueToothNameList = new ArrayList<String>();

	private static final UUID MY_UUID = UUID
			.fromString("00001101-0000-1000-8000-00805F9B34FB");

	private static String address = null;
	ArrayList<String> BlueToothAddressList = new ArrayList<String>();

	Context myContext = null;
	Handler handler = null;
	Message message = null;

	public BlueTooth(Handler handler) {
		this.handler = handler;
	}

	int Rec_Num = 2;
	float Fly_Data = 0;
	int Rec_Data = 0;
	int value1;
	int[] USART = new int[8];

	public class ReceviveThread extends Thread {
		@Override
		public void run() {
			// TODO Auto-generated method stub
			super.run();

			while (true) {
				try {
					if (threadFlag) {

						value1 = inStream.read();
						if (value1 != -1) {

							if (Rec_Num == 2) {
								USART[0] = USART[1];
								USART[1] = USART[2];
							}
							USART[Rec_Num] = value1;

							if (USART[0] == 0xAA && USART[1] == 0x85) {
								Rec_Num++;
							}
							if (Rec_Num == 8) {
								Rec_Data = (int) ((USART[4] << 24)
										| (USART[5] << 16) | (USART[6] << 8) | (USART[7]));
								if (USART[3] == 0) {
									Rec_Data = -Rec_Data;
								}

								Fly_Data = (float) Rec_Data / 1000.0f;

								switch (USART[2]) {
								/* 飞机高度 */
								case 7:
									message = handler.obtainMessage();
									message.what = 130;
									handler.sendMessage(message);
									break;

								/* 电量 */
								case 5:
									message = handler.obtainMessage();
									message.what = 140;
									handler.sendMessage(message);
									break;
								/* 温度 */
								case 10:
									message = handler.obtainMessage();
									message.what = 150;
									handler.sendMessage(message);
									break;

								/* 备用数据 */
								case 50:
									message = handler.obtainMessage();
									message.what = 100;
									handler.sendMessage(message);
									break;

								case 51:
									message = handler.obtainMessage();
									message.what = 110;
									handler.sendMessage(message);
									break;

								case 52:
									message = handler.obtainMessage();
									message.what = 120;
									handler.sendMessage(message);
									break;
								}
								Rec_Num = 2;
							}
						}
					}
				} catch (Exception e) {
				}
			}
		}
	}

	public void searchBluetooth() {
		mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
		if (mBluetoothAdapter == null) {
			bluetoothFlag = false;
			return;
		}

		if (!mBluetoothAdapter.isEnabled()) {
			bluetoothFlag = false;

			message = handler.obtainMessage();
			message.what = 50;
			handler.sendMessage(message);

			return;
		}

		message = handler.obtainMessage();
		message.what = 60;
		handler.sendMessage(message);

		// 获取蓝牙设备地址
		pairedDevices = mBluetoothAdapter.getBondedDevices();
		BlueToothNameList.clear();
		BlueToothAddressList.clear();

		if (pairedDevices.size() >= 1) {
			for (Iterator<BluetoothDevice> iterator = pairedDevices.iterator(); iterator
					.hasNext();) {
				BluetoothDevice bluetoothDevice = (BluetoothDevice) iterator
						.next();
				address = bluetoothDevice.getAddress();
				BlueToothAddressList.add(address);
				BlueToothNameList.add(bluetoothDevice.getName());
			}
		}
	}

	public void connectBluetoothDevice(int index) {

		device = mBluetoothAdapter.getRemoteDevice(BlueToothAddressList
				.get(index));

		try {
			btSocket = device.createRfcommSocketToServiceRecord(MY_UUID);
		} catch (IOException e) {
			bluetoothFlag = false;
		}
		mBluetoothAdapter.cancelDiscovery();
		try {
			btSocket.connect();

			message = handler.obtainMessage();
			message.what = 70;
			handler.sendMessage(message);

			bluetoothFlag = true;
		} catch (IOException e) {
			try {
				btSocket.close();
				bluetoothFlag = false;
			} catch (IOException e2) {
			}
		}

		if (bluetoothFlag) {
			try {
				inStream = btSocket.getInputStream();
			} catch (IOException e) {
				e.printStackTrace();
			} // 绑定读接口

			try {
				outStream = btSocket.getOutputStream();
			} catch (IOException e) {
				e.printStackTrace();
			} // 绑定写接口
		}

		threadFlag = true;
		myThread = new ReceviveThread();
		myThread.start();
	}

	public void closeBluetoothDevice(int index) {
		device = mBluetoothAdapter.getRemoteDevice(BlueToothAddressList
				.get(index));
		/*
		 * try { btSocket = device.createRfcommSocketToServiceRecord(MY_UUID);
		 * 
		 * } catch (IOException e) { bluetoothFlag = false; }
		 */
		threadFlag = false;
		mBluetoothAdapter.cancelDiscovery();
		try {
			if (btSocket != null) {
				btSocket.close();

				message = handler.obtainMessage();
				message.what = 80;
				handler.sendMessage(message);

				threadFlag = false;
				bluetoothFlag = false;
			}
		} catch (IOException e) {
			try {
				btSocket.close();
				bluetoothFlag = false;
			} catch (IOException e2) {
			}
		}
	}

	static byte[] msgBuffer = { (byte) 0xaa, (byte) 0x85, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00 };
	private BluetoothDevice device;

	public static void sendCmd(int type, int floatOrNot, int value)// 串口发送数据
	{
		if (!bluetoothFlag) {
			return;
		}

		msgBuffer[2] = (byte) type;
		msgBuffer[3] = (byte) floatOrNot;
		msgBuffer[4] = (byte) (value >> 24 & 0xff);
		msgBuffer[5] = (byte) (value >> 16 & 0xff);
		msgBuffer[6] = (byte) (value >> 8 & 0xff);
		msgBuffer[7] = (byte) (value >> 0 & 0xff);

		try {
			outStream.write(msgBuffer, 0, 8);
			outStream.flush();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	public int readByte() {// return -1 if no data
		int ret = -1;
		if (!bluetoothFlag) {
			return ret;
		}
		try {
			ret = inStream.read();
		} catch (IOException e) {
			e.printStackTrace();
		}
		return ret;
	}
}
