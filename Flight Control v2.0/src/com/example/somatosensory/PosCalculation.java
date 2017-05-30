package com.example.somatosensory;

import android.os.Handler;
import android.os.Message;

public class PosCalculation {

	IMU_tt imu;
	Filter filter;
	
	Handler handler;
	Message message;
	

	public PosCalculation(Handler handler) {
		imu = new IMU_tt();
		filter = new Filter();
		this.handler = handler;
	}

	int Init_flag = 0;
	int bFilterInit = 0;
	static int offset_count = 0;
	static float[] gyro_offsets_sum = { 0.0f, 0.0f, 0.0f };
	static float[] acc_offsets_sum = { 0.0f, 0.0f, 0.0f };
	float so3_comp_params_Kp = 1.0f;
	float so3_comp_params_Ki = 0.05f;

	static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
	static float dq0 = 0.0f, dq1 = 0.0f, dq2 = 0.0f, dq3 = 0.0f;
	static float[] gyro_bias = { 0.0f, 0.0f, 0.0f };
	/** bias estimation */
	static float q0q0, q0q1, q0q2, q0q3;
	static float q1q1, q1q2, q1q3;
	static float q2q2, q2q3;
	static float q3q3;

	class IMU_tt {
		int caliPass = 0;
		int ready = 0;
		int[] accADC = new int[3];
		int[] gyroADC = new int[3];
		int[] magADC = new int[3];
		float[] accRaw = new float[3]; // m/s^2
		float[] gyroRaw = new float[3]; // rad/s
		float[] magRaw = new float[3]; //
		float[] accOffset = new float[3]; // m/s^2
		float[] gyroOffset = new float[3];
		float[] accb = new float[3]; // filted, in body frame
		float[] accg = new float[3];
		float[] gyro = new float[3];
		float[] q = new float[4];
		float roll = 0;
		float pitch = 0;
		float yaw = 0;
		float rollRad = 0;
		float pitchRad = 0;
		float yawRad = 0;

		public IMU_tt() {
			for (int i = 0; i < 3; i++) {
				magADC[i] = 0;
				accRaw[i] = 0;
				gyroRaw[i] = 0;
				magRaw[i] = 0;
				accOffset[i] = 0;
				gyroOffset[i] = 0;
				accb[i] = 0;
				accg[i] = 0;
				gyro[i] = 0;
			}
			for (int i = 0; i < 4; i++) {
				q[i] = 0;
			}
		}
	}

	float IMU_SAMPLE_RATE = 200f;
	float IMU_FILTER_CUTOFF_FREQ = 30f;

	public void IMU_Init() {
		imu.ready = 0;
		imu.caliPass = 1;

		filter.LPF2pSetCutoffFreq_1(IMU_SAMPLE_RATE, IMU_FILTER_CUTOFF_FREQ);
		filter.LPF2pSetCutoffFreq_2(IMU_SAMPLE_RATE, IMU_FILTER_CUTOFF_FREQ);
		filter.LPF2pSetCutoffFreq_3(IMU_SAMPLE_RATE, IMU_FILTER_CUTOFF_FREQ);
		filter.LPF2pSetCutoffFreq_4(IMU_SAMPLE_RATE, IMU_FILTER_CUTOFF_FREQ);
		filter.LPF2pSetCutoffFreq_5(IMU_SAMPLE_RATE, IMU_FILTER_CUTOFF_FREQ);
		filter.LPF2pSetCutoffFreq_6(IMU_SAMPLE_RATE, IMU_FILTER_CUTOFF_FREQ);
	}

	float Q1, Q2, Q3, Q4, Q5;
	float k1, k2, k3, k4;

	void NonlinearSO3AHRSinit(float ax, float ay, float az, float mx, float my,
			float mz) {
		float initialRoll, initialPitch;
		float cosRoll, sinRoll, cosPitch, sinPitch;
		float magX, magY;
		float initialHdg, cosHeading, sinHeading;

		initialRoll = (float) Math.atan2(-ay, -az);
		initialPitch = (float) Math.atan2(ax, -az);

		cosRoll = (float) Math.cos(initialRoll);
		sinRoll = (float) Math.sin(initialRoll);
		cosPitch = (float) Math.cos(initialPitch);
		sinPitch = (float) Math.sin(initialPitch);

		magX = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll
				* sinPitch;

		magY = my * cosRoll - mz * sinRoll;

		initialHdg = (float) Math.atan2(-magY, magX);

		cosRoll = (float) Math.cos(initialRoll * 0.5f);
		sinRoll = (float) Math.sin(initialRoll * 0.5f);

		cosPitch = (float) Math.cos(initialPitch * 0.5f);
		sinPitch = (float) Math.sin(initialPitch * 0.5f);

		cosHeading = (float) Math.cos(initialHdg * 0.5f);
		sinHeading = (float) Math.sin(initialHdg * 0.5f);

		q0 = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
		q1 = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
		q2 = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
		q3 = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;

		// auxillary variables to reduce number of repeated operations, for 1st
		// pass
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

	}

	public void NonlinearSO3AHRSupdate(float gx, float gy, float gz, float ax,
			float ay, float az, float mx, float my, float mz, float twoKp,
			float twoKi, float dt) {
		float recipNorm;
		float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;

		// Make filter converge to initial solution faster
		// This function assumes you are in static position.
		// WARNING : in case air reboot, this can cause problem. But this is
		// very unlikely happen.
		if (bFilterInit == 0) {
			NonlinearSO3AHRSinit(ax, ay, az, mx, my, mz);
			bFilterInit = 1;
		}

		// ! If magnetometer measurement is available, use it.
		if (!((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))) {
			float hx, hy, hz, bx, bz;
			float halfwx, halfwy, halfwz;

			// Normalise magnetometer measurement
			// Will sqrt work better? PX4 system is powerful enough?
			recipNorm = 1.0f / (float) Math.sqrt(mx * mx + my * my + mz * mz);
			mx *= recipNorm;
			my *= recipNorm;
			mz *= recipNorm;

			// Reference direction of Earth's magnetic field
			hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz
					* (q1q3 + q0q2));
			hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz
					* (q2q3 - q0q1));
			hz = 2.0f * mx * (q1q3 - q0q2) + 2.0f * my * (q2q3 + q0q1) + 2.0f
					* mz * (0.5f - q1q1 - q2q2);
			bx = (float) Math.sqrt(hx * hx + hy * hy);
			bz = hz;

			// Estimated direction of magnetic field
			halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
			halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
			halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

			// Error is sum of cross product between estimated direction and
			// measured direction of field vectors
			halfex += (my * halfwz - mz * halfwy);
			halfey += (mz * halfwx - mx * halfwz);
			halfez += (mx * halfwy - my * halfwx);
		}

		// 增加一个条件： 加速度的模量与G相差不远时。 0.75*G < normAcc < 1.25*G
		// Compute feedback only if accelerometer measurement valid (avoids NaN
		// in accelerometer normalisation)
		if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
			float halfvx, halfvy, halfvz;

			// Normalise accelerometer measurement
			recipNorm = 1.0f / (float) Math.sqrt(ax * ax + ay * ay + az * az);

			ax *= recipNorm;
			ay *= recipNorm;
			az *= recipNorm;

			// Estimated direction of gravity and magnetic field
			halfvx = q1q3 - q0q2;
			halfvy = q0q1 + q2q3;
			halfvz = q0q0 - 0.5f + q3q3;

			// Error is sum of cross product between estimated direction and
			// measured direction of field vectors
			halfex += ay * halfvz - az * halfvy;
			halfey += az * halfvx - ax * halfvz;
			halfez += ax * halfvy - ay * halfvx;
		}

		// Apply feedback only when valid data has been gathered from the
		// accelerometer or magnetometer
		if (halfex != 0.0f && halfey != 0.0f && halfez != 0.0f) {
			// Compute and apply integral feedback if enabled
			if (twoKi > 0.0f) {
				gyro_bias[0] += twoKi * halfex * dt; // integral error scaled by
														// Ki
				gyro_bias[1] += twoKi * halfey * dt;
				gyro_bias[2] += twoKi * halfez * dt;

				// apply integral feedback
				gx += gyro_bias[0];
				gy += gyro_bias[1];
				gz += gyro_bias[2];
			} else {
				gyro_bias[0] = 0.0f; // prevent integral windup
				gyro_bias[1] = 0.0f;
				gyro_bias[2] = 0.0f;
			}

			// Apply proportional feedback
			gx += twoKp * halfex;
			gy += twoKp * halfey;
			gz += twoKp * halfez;
		}

		// Time derivative of quaternion. q_dot = 0.5*q\otimes omega.
		// ! q_k = q_{k-1} + dt*\dot{q}
		// ! \dot{q} = 0.5*q \otimes P(\omega)
		dq0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
		dq1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
		dq2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
		dq3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

		q0 += dt * dq0;
		q1 += dt * dq1;
		q2 += dt * dq2;
		q3 += dt * dq3;

		// Normalise quaternion
		recipNorm = 1.0f / (float) Math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3
				* q3);
		q0 *= recipNorm;
		q1 *= recipNorm;
		q2 *= recipNorm;
		q3 *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;
	}

	public void ReadIMUSensorHandle(float[] Accel, float[] Gyro) {
		imu.accRaw[0] = Accel[0];
		imu.accRaw[1] = Accel[1];
		imu.accRaw[2] = Accel[2];
		imu.gyroRaw[0] = Gyro[0];
		imu.gyroRaw[1] = Gyro[1];
		imu.gyroRaw[2] = Gyro[2];

		if (imu.ready == 1) {
			imu.accb[0] = filter.LPF2pApply_1(imu.accRaw[0] - imu.accOffset[0]);
			imu.accb[1] = filter.LPF2pApply_2(imu.accRaw[1] - imu.accOffset[1]);
			imu.accb[2] = filter.LPF2pApply_3(imu.accRaw[2] - imu.accOffset[2]);
			imu.gyro[0] = filter.LPF2pApply_4(imu.gyroRaw[0]);
			imu.gyro[1] = filter.LPF2pApply_5(imu.gyroRaw[1]);
			imu.gyro[2] = filter.LPF2pApply_6(imu.gyroRaw[2]);
		}

	}

	long now = 0;
	long tPrev = 0;
	float t = 0;
	float[] t1;
	int Zero_num = 0;
	float Yaw_Zero_Sum = 0;
	float Yaw_Zero = 0;
	float Cal_Yaw = 0;
	int check_zero_flag = 0;

	public void IMUSO3Thread(float[] Accel, float[] Gyro, float[] Mag) {
		// ! Time constant
		float dt = 0.01f; // s
		/* output euler angles */
		float[] euler = { 0.0f, 0.0f, 0.0f }; // rad
		/* Initialization */
		float[] Rot_matrix = { 1.f, 0.0f, 0.0f, 0.0f, 1.f, 0.0f, 0.0f, 0.0f,
				1.f };
		float[] acc = { 0.0f, 0.0f, 0.0f }; // m/s^2
		float[] gyro = { 0.0f, 0.0f, 0.0f }; // rad/s
		float[] mag = { 0.0f, 0.0f, 0.0f };
		mag[0] = Mag[0];
		mag[1] = Mag[1];
		mag[2] = Mag[2];
		/*
		 * mag[0] = 0.0f; mag[1] = 0.0f; mag[2] = 0.0f;
		 */

		now = System.nanoTime();
		dt = (tPrev > 0) ? (now - tPrev) / 1000000000.0f : 0;
		tPrev = now;

		if (Init_flag == 0) {
			IMU_Init();
			Init_flag = 1;
		}

		ReadIMUSensorHandle(Accel, Gyro);

		if (imu.ready == 0) {

			gyro_offsets_sum[0] += imu.gyroRaw[0];
			gyro_offsets_sum[1] += imu.gyroRaw[1];
			gyro_offsets_sum[2] += imu.gyroRaw[2];

			acc_offsets_sum[0] += imu.accRaw[0];
			acc_offsets_sum[1] += imu.accRaw[1];
			acc_offsets_sum[2] += imu.accRaw[2];

			offset_count++;

			if (offset_count == 500) {
				imu.gyroOffset[0] = gyro_offsets_sum[0] / offset_count;
				imu.gyroOffset[1] = gyro_offsets_sum[1] / offset_count;
				imu.gyroOffset[2] = gyro_offsets_sum[2] / offset_count;

				imu.accOffset[0] = acc_offsets_sum[0] / offset_count;
				imu.accOffset[1] = acc_offsets_sum[1] / offset_count;
				imu.accOffset[2] = acc_offsets_sum[2] / offset_count;

				imu.accOffset[2] = acc_offsets_sum[2] / offset_count - 9.8f;

				gyro_offsets_sum[0] = 0;
				gyro_offsets_sum[1] = 0;
				gyro_offsets_sum[2] = 0;
				
				acc_offsets_sum[0] = 0;
				acc_offsets_sum[1] = 0;
				acc_offsets_sum[2] = 0;
				
				offset_count = 0;
				imu.ready = 1;
			}
			return;
		}

		gyro[0] = imu.gyro[0] - imu.gyroOffset[0];
		gyro[1] = imu.gyro[1] - imu.gyroOffset[1];
		gyro[2] = imu.gyro[2] - imu.gyroOffset[2];

		acc[0] = -imu.accb[0];
		acc[1] = -imu.accb[1];
		acc[2] = -imu.accb[2];

		NonlinearSO3AHRSupdate(gyro[0], gyro[1], gyro[2], -acc[0], -acc[1],
				-acc[2], mag[0], mag[1], mag[2], so3_comp_params_Kp,
				so3_comp_params_Ki, dt);

		// Convert q->R, This R converts inertial frame to body frame.
		Rot_matrix[0] = q0q0 + q1q1 - q2q2 - q3q3;// 11
		Rot_matrix[1] = 2.f * (q1 * q2 + q0 * q3); // 12
		Rot_matrix[2] = 2.f * (q1 * q3 - q0 * q2); // 13
		Rot_matrix[3] = 2.f * (q1 * q2 - q0 * q3); // 21
		Rot_matrix[4] = q0q0 - q1q1 + q2q2 - q3q3;// 22
		Rot_matrix[5] = 2.f * (q2 * q3 + q0 * q1); // 23
		Rot_matrix[6] = 2.f * (q1 * q3 + q0 * q2); // 31
		Rot_matrix[7] = 2.f * (q2 * q3 - q0 * q1); // 32
		Rot_matrix[8] = q0q0 - q1q1 - q2q2 + q3q3;// 33

		euler[0] = (float) Math.atan2(Rot_matrix[5], Rot_matrix[8]); // ! Roll
		euler[1] = (float) -Math.asin(Rot_matrix[2]); // ! Pitch
		euler[2] = (float) Math.atan2(Rot_matrix[1], Rot_matrix[0]);

		imu.rollRad = euler[0];
		imu.pitchRad = euler[1];
		imu.yawRad = euler[2];

		imu.roll = euler[0] * 57.3f;
		imu.pitch = euler[1] * 57.3f;
		Cal_Yaw = -euler[2] * 57.3f;

		/* Yaw校零 */
		if (check_zero_flag == 0) {
			if (Zero_num < 100) {
				Zero_num++;
				Yaw_Zero_Sum += Cal_Yaw;
			}
			if (Zero_num == 100) {
				Yaw_Zero = Yaw_Zero_Sum / 100.0f;
				Zero_num = 0;
				Yaw_Zero_Sum = 0;
				
				check_zero_flag = 1;
				
				message = handler.obtainMessage();
				message.what = 200;
				handler.sendMessage(message);
			}
			imu.yaw = 0;
		} else {
			imu.yaw = Cal_Yaw - Yaw_Zero;
			if (imu.yaw < -180)
				imu.yaw += 360;
		}
	}
}
