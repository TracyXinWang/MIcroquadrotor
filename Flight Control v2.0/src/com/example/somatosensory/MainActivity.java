package com.example.somatosensory;

import java.text.DecimalFormat;

import android.app.Activity;
import android.graphics.Color;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.util.DisplayMetrics;
import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.View.OnTouchListener;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.EditText;
import android.widget.RelativeLayout;
import android.widget.ScrollView;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;

import com.example.aircraftbluetooth20.R;

public class MainActivity extends Activity implements SensorEventListener {

	private SensorManager sensorManager;
	EditText et_power, et_height, et_pitch, et_yaw, et_roll, et_tem, et_param1,
			et_param2, et_param3;
	Button bt_output, bt_BT_Open, bt_height_add, bt_height_reduce,
			bt_fly_check, bt_phone_check;
	ToggleButton bt_status_lamp, bt_light_lamp, bt_bizhang;
	CheckBox cb_mode1, cb_mode2, cb_mode3;
	Spinner BT_Spinner;
	RelativeLayout rl_R1, rl_R2, rl_R3, rl_R4, rl_Param;

	static TextView tv_log;
	static ScrollView tv_scrollview;
	static StringBuilder sb_output = new StringBuilder();

	private int screenWidth;
	private int screenHeight;

	int sendDelay = 30; // ������ʱms
	static int UI_Time = 0;

	static boolean output_Flag = false;
	/* �ֻ�ԭʼ���� */
	float[] Accel = new float[3];
	float[] Gyro = new float[3];
	float[] Mag = new float[3];

	private PosCalculation posCal;
	BlueTooth BT;

	ArrayAdapter<String> adapter;

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_main);

		et_power = (EditText) findViewById(R.id.et_power);
		et_height = (EditText) findViewById(R.id.et_height);
		et_tem = (EditText) findViewById(R.id.et_tem);
		et_pitch = (EditText) findViewById(R.id.et_pitch);
		et_yaw = (EditText) findViewById(R.id.et_yaw);
		et_roll = (EditText) findViewById(R.id.et_roll);

		bt_output = (Button) findViewById(R.id.bt_output);
		bt_BT_Open = (Button) findViewById(R.id.bt_BT_Open);
		tv_log = (TextView) findViewById(R.id.tv_log);
		tv_scrollview = (ScrollView) findViewById(R.id.tv_scrollview);

		BT_Spinner = (Spinner) findViewById(R.id.Spinner_BT);
		rl_R1 = (RelativeLayout) findViewById(R.id.R1);
		rl_R2 = (RelativeLayout) findViewById(R.id.R2);
		rl_R3 = (RelativeLayout) findViewById(R.id.R3);
		rl_R4 = (RelativeLayout) findViewById(R.id.R4);
		rl_Param = (RelativeLayout) findViewById(R.id.RL_Param);

		et_param1 = (EditText) findViewById(R.id.et_Param1);
		et_param2 = (EditText) findViewById(R.id.et_Param2);
		et_param3 = (EditText) findViewById(R.id.et_Param3);

		/* ��ȡ��Ļ��� ���ò��ִ�С */
		DisplayMetrics dm = new DisplayMetrics();
		getWindowManager().getDefaultDisplay().getMetrics(dm);
		screenWidth = dm.widthPixels;
		screenHeight = dm.heightPixels;
		output_print("��Ļ�ֱ���:  " + screenWidth + " * " + screenHeight);

		RelativeLayout.LayoutParams linearParams = (RelativeLayout.LayoutParams) rl_R1
				.getLayoutParams();
		linearParams.width = (int) (screenWidth * 2.0 / 5);
		rl_R1.setLayoutParams(linearParams);

		linearParams = (RelativeLayout.LayoutParams) rl_R2.getLayoutParams();
		linearParams.width = (int) (screenWidth * 3.0 / 5);
		rl_R2.setLayoutParams(linearParams);

		linearParams = (RelativeLayout.LayoutParams) rl_Param.getLayoutParams();
		linearParams.width = (int) (screenWidth * 3.0 / 5);
		rl_Param.setLayoutParams(linearParams);

		linearParams = (RelativeLayout.LayoutParams) rl_R3.getLayoutParams();
		linearParams.width = (int) (screenWidth * 2.0 / 5);
		rl_R3.setLayoutParams(linearParams);

		linearParams = (RelativeLayout.LayoutParams) rl_R4.getLayoutParams();
		linearParams.width = (int) (screenWidth * 3.0 / 5);
		rl_R4.setLayoutParams(linearParams);

		/* ��Ӱ��������¼� */
		bt_height_add = (Button) findViewById(R.id.bt_height_add);
		bt_height_reduce = (Button) findViewById(R.id.bt_height_reduce);
		bt_fly_check = (Button) findViewById(R.id.bt_Plane_Calibrate);
		bt_phone_check = (Button) findViewById(R.id.bt_Phone_Calibrate);
		bt_status_lamp = (ToggleButton) findViewById(R.id.Toggle_STATUS_LAMP);
		bt_light_lamp = (ToggleButton) findViewById(R.id.Toggle_SHINE_LAMP);
		bt_bizhang = (ToggleButton) findViewById(R.id.Toggle_OV);
		cb_mode1 = (CheckBox) findViewById(R.id.cb_mode1);
		cb_mode2 = (CheckBox) findViewById(R.id.cb_mode2);
		cb_mode3 = (CheckBox) findViewById(R.id.cb_mode3);

		ButtonListener b = new ButtonListener();
		bt_height_add.setOnClickListener(b);
		bt_height_add.setOnTouchListener(b);
		bt_height_reduce.setOnClickListener(b);
		bt_height_reduce.setOnTouchListener(b);
		bt_fly_check.setOnClickListener(b);
		bt_fly_check.setOnTouchListener(b);
		bt_phone_check.setOnClickListener(b);
		bt_phone_check.setOnTouchListener(b);
		bt_status_lamp.setOnClickListener(b);
		bt_status_lamp.setOnTouchListener(b);
		bt_light_lamp.setOnClickListener(b);
		bt_light_lamp.setOnTouchListener(b);
		bt_bizhang.setOnClickListener(b);
		bt_bizhang.setOnTouchListener(b);

		cb_mode1.setOnClickListener(b);
		cb_mode2.setOnClickListener(b);
		cb_mode3.setOnClickListener(b);

		/* ��ȡ�ֻ������������� */
		sensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);

		/* ����̨��� */
		tv_log.requestFocus();

		/* ������������߳� */
		MyTimeThread t1 = new MyTimeThread();
		t1.start();

		/* ��̬���� */
		posCal = new PosCalculation(UI_Handler);
		PosCal_Thread t2 = new PosCal_Thread();
		t2.start();

		/* �������� */
		BT = new BlueTooth(UI_Handler);
		BlueToothThread t3 = new BlueToothThread();
		t3.start();

		if (posCal.check_zero_flag == 0) {
			output_print("������  ���ٶȼ�  ����У׼������");
		}
	}

	boolean Send_Heightadd = false;
	boolean Send_Heightareduce = false;

	/* �������� */
	class ButtonListener implements OnClickListener, OnTouchListener {

		public void onClick(View v) {
			if (v.getId() == R.id.bt_Plane_Calibrate) {
				sendInt(0, flycheckID);
				output_print("�ɻ�У׼��");
			}
			if (v.getId() == R.id.bt_Phone_Calibrate) {

				output_print("�ֻ���ʼУ׼��");

				posCal.imu.ready = 0;
				posCal.bFilterInit = 0;
				posCal.check_zero_flag = 0;
			}
			if (v.getId() == R.id.Toggle_STATUS_LAMP) {
				if (bt_status_lamp.isChecked()) {
					sendInt(1, statuslampID);
					output_print("��״̬�ƣ�");
				} else {
					sendInt(0, statuslampID);
					output_print("�ر�״̬�ƣ�");
				}
			}

			if (v.getId() == R.id.Toggle_SHINE_LAMP) {
				if (bt_light_lamp.isChecked()) {
					sendInt(1, lightlampID);
					output_print("�������ƣ�");
				} else {
					sendInt(0, lightlampID);
					output_print("�ر������ƣ�");
				}
			}

			if (v.getId() == R.id.Toggle_OV) {
				if (bt_bizhang.isChecked()) {
					sendInt(1, bizhangID);
					output_print("�������Ϲ��ܣ�");
				} else {
					sendInt(0, bizhangID);
					output_print("�رձ��Ϲ��ܣ�");
				}
			}

			if (v.getId() == R.id.cb_mode1) {
				if (cb_mode1.isChecked()) {
					cb_mode2.setChecked(false);
					cb_mode3.setChecked(false);
					sendInt(0, mode1ID);
					output_print("��ѡ��ģʽ1");
				}
			}
			if (v.getId() == R.id.cb_mode2) {
				if (cb_mode2.isChecked()) {
					cb_mode1.setChecked(false);
					cb_mode3.setChecked(false);
					sendInt(0, mode2ID);
					output_print("��ѡ��ģʽ2");
				}
			}
			if (v.getId() == R.id.cb_mode3) {
				if (cb_mode3.isChecked()) {
					cb_mode1.setChecked(false);
					cb_mode2.setChecked(false);
					sendInt(0, mode3ID);
					output_print("��ѡ��ģʽ3");
				}
			}
		}

		public boolean onTouch(View v, MotionEvent event) {
			if (v.getId() == R.id.bt_height_add) {
				if (event.getAction() == MotionEvent.ACTION_UP) {
					Send_Heightadd = false;
					bt_height_add.setBackgroundResource(R.drawable.up);
					output_print("ֹͣ������");
				}
				if (event.getAction() == MotionEvent.ACTION_DOWN) {
					Send_Heightadd = true;
					bt_height_add.setBackgroundResource(R.drawable.up1);
					output_print("�߶�������");
				}
			}

			if (v.getId() == R.id.bt_height_reduce) {
				if (event.getAction() == MotionEvent.ACTION_UP) {
					Send_Heightareduce = false;
					bt_height_reduce.setBackgroundResource(R.drawable.down);
					output_print("ֹͣ�½���");
				}
				if (event.getAction() == MotionEvent.ACTION_DOWN) {
					Send_Heightareduce = true;
					bt_height_reduce.setBackgroundResource(R.drawable.down1);
					output_print("�߶��½���");
				}
			}
			return false;
		}
	}

	int Bluetooth_Time = 1000;
	int BT_Flag = 0;
	int BT_Index = 0;

	class BlueToothThread extends Thread {
		@Override
		public void run() {
			// TODO Auto-generated method stub
			while (true) {
				try {
					switch (BT_Flag) {
					case 0:
						BT_Flag = 10;
						BT.searchBluetooth();

						Data_Msg = UI_Handler.obtainMessage();
						Data_Msg.what = 90;
						UI_Handler.sendMessage(Data_Msg);
						break;

					case 1:
						BT_Flag = 10;
						BT.connectBluetoothDevice(BT_Index);
						break;

					case 2:
						BT_Flag = 10;
						BT.closeBluetoothDevice(BT_Index);
						break;
					}
					Thread.sleep(Bluetooth_Time);
				} catch (Exception e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		}
	}

	/* ����ˢ�� */
	public void bt_refresh(View v) {
		BT_Flag = 0;
	}

	/* �������� */
	boolean bt_open_flag = false;

	public void bt_open_close(View v) {
		if (!bt_open_flag) {
			bt_open_flag = true;
			BT_Flag = 1;
			bt_BT_Open.setText("�رմ���");
		} else {
			bt_open_flag = false;
			BT_Flag = 2;
			bt_BT_Open.setText("�򿪴���");
		}
	}

	int pitchID = 0;
	int yawID = 1;
	int rollID = 2;
	int flycheckID = 3;
	int upID = 4;
	int downID = 5;
	int statuslampID = 6; /* 1�� 0�� */
	int lightlampID = 7;
	int bizhangID = 8;
	int mode1ID = 9;
	int mode2ID = 10;
	int mode3ID = 11;

	/* ���ݷ������� */
	class MyTimeThread extends Thread {

		@Override
		public void run() {
			// TODO Auto-generated method stub
			while (true) {
				try {
					Thread.sleep(sendDelay);
					if (output_Flag && BT.bluetoothFlag
							&& (posCal.check_zero_flag == 1)) {

						sendFloat(posCal.imu.pitch, pitchID);
						Thread.sleep(5);
						sendFloat(posCal.imu.yaw, yawID);
						Thread.sleep(5);
						sendFloat(posCal.imu.roll, rollID);
					}

					if (BT.bluetoothFlag) {
						if (Send_Heightadd && (!Send_Heightareduce)) {
							/* ���� */
							sendInt(0, upID);
							Thread.sleep(5);
						}

						if ((!Send_Heightadd) && Send_Heightareduce) {
							/* �½� */
							sendInt(0, downID);
							Thread.sleep(5);
						}
					}
				} catch (Exception e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		}
	}

	/* ��̬�����߳� */
	static Object obj_lock = new Object();
	int show_num = 0;
	Message PosCal_Message = null;

	class PosCal_Thread extends Thread {
		int PosCal_Time = 10;

		public void run() {
			// TODO Auto-generated method stub
			while (true) {
				try {
					if (show_num++ == 5) {
						show_num = 0;

						Data_Msg = UI_Handler.obtainMessage();
						Data_Msg.what = 40;
						UI_Handler.sendMessage(Data_Msg);
					}
					Thread.sleep(PosCal_Time);
					posCal.IMUSO3Thread(Accel, Gyro, Mag);
				} catch (Exception e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		}
	}

	/* ���͸������� ��ֵ ���� */
	public void sendFloat(float floatNum, int type) {
		int floatOrNot = 1;
		int num = (int) (floatNum * 1000);
		BT.sendCmd(type, floatOrNot, num);
	}

	/* ������������ */
	public void sendInt(int intNum, int type) {
		int floatOrNot = 0;
		BT.sendCmd(type, floatOrNot, intNum);
	}

	/* ��Ϣ��ӡ��� */
	public static void output_print(String str) {

		if (UI_Time++ == 250) {
			UI_Time = 0;
			tv_log.setText("");
		}

		tv_log.append(str + "\n");
		scroll2Bottom(tv_scrollview, tv_log);
	}

	public void bt_output(View v) {

		if (!output_Flag) {
			output_print("��ʼ��п��Ʒɻ�������");
			bt_output.setText("ֹͣ");
			bt_output.setTextColor(Color.RED);
			output_Flag = true;
		} else {
			output_print("������п��Ʒɻ�������");
			bt_output.setText("����");
			bt_output.setTextColor(Color.GREEN);
			output_Flag = false;
		}
	}

	@Override
	protected void onResume() {
		// TODO Auto-generated method stub
		super.onResume();

		sensorManager.registerListener(this,
				sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE),
				SensorManager.SENSOR_DELAY_GAME);
		sensorManager.registerListener(this,
				sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER),
				SensorManager.SENSOR_DELAY_GAME);
		sensorManager.registerListener(this,
				sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD),
				SensorManager.SENSOR_DELAY_GAME);
	}

	Message Data_Msg = null;

	public void onSensorChanged(SensorEvent event) {
		// TODO Auto-generated method stub
		float[] values = event.values;
		int sensorType = event.sensor.getType();
		switch (sensorType) {
		case Sensor.TYPE_ACCELEROMETER:
			Accel[0] = -event.values[1];
			Accel[1] = event.values[0];
			Accel[2] = event.values[2];
			break;

		case Sensor.TYPE_GYROSCOPE:
			Gyro[0] = -event.values[1];
			Gyro[1] = event.values[0];
			Gyro[2] = event.values[2];
			break;

		case Sensor.TYPE_MAGNETIC_FIELD:
			Mag[0] = -event.values[1];
			Mag[1] = event.values[0];
			Mag[2] = event.values[2];
			break;
		}
	}

	/* ��������λ���ײ� */
	public static void scroll2Bottom(final ScrollView scroll, final View inner) {
		Handler handler = new Handler();
		handler.post(new Runnable() {

			@Override
			public void run() {
				// TODO Auto-generated method stub
				if (scroll == null || inner == null) {
					return;
				}
				// �ڲ�߶ȳ������
				int offset = inner.getMeasuredHeight()
						- scroll.getMeasuredHeight();
				if (offset < 0) {
					System.out.println("��λ...");
					offset = 0;
				}
				scroll.scrollTo(0, offset);
			}
		});
	}

	public void onAccuracyChanged(Sensor sensor, int accuracy) {
		// TODO Auto-generated method stub
	}

	int k = 0;
	DecimalFormat decimalFormat = new DecimalFormat("0.00");
	Handler UI_Handler = new Handler() {
		public void handleMessage(Message msg) {
			switch (msg.what) {

			case 40:
				et_pitch.setText(" Pitch: "
						+ decimalFormat.format(posCal.imu.pitch));
				et_yaw.setText(" Yaw  : "
						+ decimalFormat.format(posCal.imu.yaw));
				et_roll.setText(" Roll : "
						+ decimalFormat.format(posCal.imu.roll));
				break;

			case 50:
				Toast.makeText(MainActivity.this, "����������������г���", 0).show();
				break;

			case 60:
				Toast.makeText(MainActivity.this, "�����������豸!", 0).show();
				break;

			case 70:
				Toast.makeText(MainActivity.this, "���ӳɹ����������Կ�ʼ�ٿ���!", 0).show();
				break;

			case 80:
				Toast.makeText(MainActivity.this, "�����ѹرգ�����", 0).show();
				break;

			case 200:
				output_print("�ֻ�У׼���!");

				/* �������� */
			case 100:
				et_param1.setText(decimalFormat.format(BT.Fly_Data));
				break;

			case 110:
				et_param2.setText(decimalFormat.format(BT.Fly_Data));
				break;

			case 120:
				et_param3.setText(decimalFormat.format(BT.Fly_Data));
				break;

			/* �߶���Ϣ */
			case 130:
				et_height.setText(decimalFormat.format(BT.Fly_Data) + "m");
				break;
			/* ������Ϣ */
			case 140:
				et_power.setText(decimalFormat.format(BT.Fly_Data) + "%");
				break;
			/* �¶���Ϣ */
			case 150:
				et_tem.setText(decimalFormat.format(BT.Fly_Data) + "��");
				break;

			case 90:
				adapter = new ArrayAdapter<String>(MainActivity.this,
						R.layout.myspinner, BT.BlueToothNameList);
				adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);

				BT_Spinner.setAdapter(adapter);
				BT_Spinner
						.setOnItemSelectedListener(new Spinner.OnItemSelectedListener() {
							public void onItemSelected(AdapterView<?> arg0,
									View arg1, int arg2, long arg3) {
								BT_Index = arg2;
								output_print("��ǰѡ��������ǣ�"
										+ adapter.getItem(arg2) + "  "
										+ BT_Index);

								/* ��Spinner ��ʾ */
								arg0.setVisibility(View.VISIBLE);
							}

							public void onNothingSelected(AdapterView<?> arg0) {
								// TODO Auto-generated method stub
								output_print("NONE");
								arg0.setVisibility(View.VISIBLE);
							}
						});
				/* �����˵�����������ѡ����¼����� */
				BT_Spinner.setOnTouchListener(new Spinner.OnTouchListener() {
					public boolean onTouch(View v, MotionEvent event) {
						// TODO Auto-generated method stub
						return false;
					}
				});
				/* �����˵�����������ѡ���ı��¼����� */
				BT_Spinner
						.setOnFocusChangeListener(new Spinner.OnFocusChangeListener() {
							public void onFocusChange(View v, boolean hasFocus) {
								// TODO Auto-generated method stub
							}
						});
				break;
			}
		};
	};

}
