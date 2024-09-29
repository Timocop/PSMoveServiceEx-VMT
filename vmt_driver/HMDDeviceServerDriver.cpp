/*
MIT License

Copyright (c) 2020 gpsnmeajp

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "HMDDeviceServerDriver.h"

#define TRACKING_UNIVERSE_ID 6

namespace VMTDriver {
	//** 内部向け関数群 **

	//自動更新を有効にするか
	//仮想デバイスのコンストラクタ。(Listから暗黙的にコールされる)
	HMDDeviceServerDriver::HMDDeviceServerDriver()
	{
		m_deviceIndex = k_unTrackedDeviceIndexInvalid;
		m_propertyContainer = k_ulInvalidPropertyContainer;

		m_userIpdMeters = 0.067f;
	}

	//仮想デバイスのデストラクタ。(Listから暗黙的にコールされる)
	HMDDeviceServerDriver::~HMDDeviceServerDriver()
	{
	}

	//仮想デバイスにシリアル番号を設定
	void HMDDeviceServerDriver::SetDeviceSerial(string serial)
	{
		m_serial = serial;
	}

	//仮想デバイスに内部Indexを設定
	void HMDDeviceServerDriver::SetObjectIndex(uint32_t idx)
	{
		m_index = idx;
	}

	//仮想デバイスにOpenVR姿勢を設定
	void HMDDeviceServerDriver::SetPose(DriverPose_t pose)
	{
		m_pose = pose;
	}

	//仮想デバイスに内部姿勢を設定
	void HMDDeviceServerDriver::SetRawPose(RawHmdPose rawPose)
	{
		m_poweron = true; //有効な姿勢なので電源オン状態にする

		//自動更新が無効ならば内部姿勢を保存し、OpenVR姿勢を更新する
		m_lastRawPose = m_rawPose; //差分を取るために前回値を取っておく
		m_rawPose = rawPose;

		SetPose(RawPoseToPose());
	}

	//仮想デバイスに内部姿勢を設定
	void HMDDeviceServerDriver::SetupDisplaySettings(DisplaySettings displaySettings)
	{
		if (m_alreadyRegistered || m_registrationInProgress)
			return;

		m_displaySettings = displaySettings;
		m_displayValid = true;
	}

	void HMDDeviceServerDriver::SetupRenderSettings(RenderSettings renderSettings)
	{
		if (m_alreadyRegistered || m_registrationInProgress)
			return;

		m_renderSettings = renderSettings;
		m_renderValid = true;
	}

	void HMDDeviceServerDriver::SetIpdMeters(float userIpdMeters)
	{
		if (!m_alreadyRegistered) { return; }
		if (m_propertyContainer == k_ulInvalidPropertyContainer) { return; }

		if (userIpdMeters > 1.0) {
			userIpdMeters = 1.0;
		}
		if (userIpdMeters < 0) {
			userIpdMeters = 0;
		}
		if (isnan(userIpdMeters)) {
			userIpdMeters = 0;
		}

		if (m_userIpdMeters == userIpdMeters)
			return;

		m_userIpdMeters = userIpdMeters;

		VRProperties()->SetFloatProperty(m_propertyContainer, Prop_UserIpdMeters_Float, m_userIpdMeters);
	}

	void HMDDeviceServerDriver::SetVelocity(bool enable)
	{
		m_enableVelocity = enable;
	}

	//Joint計算を行う
	void HMDDeviceServerDriver::CalcJoint(DriverPose_t& pose, string serial, ReferMode_t mode, Eigen::Affine3d& RoomToDriverAffin) {
		vr::TrackedDevicePose_t devicePoses[k_unMaxTrackedDeviceCount]{};

		//OpenVRから全トラッキングデバイスの情報を取得する
		VRServerDriverHost()->GetRawTrackedDevicePoses(0.0f, devicePoses, k_unMaxTrackedDeviceCount);

		//接続済みのデバイスの中から、シリアル番号でデバイスを検索する
		int index = SearchDevice(devicePoses, serial);

		//探索エラーが帰ってきたら
		if (index == k_unTrackedDeviceIndexInvalid) {
			//デバイス = 接続済み・無効
			RejectTracking(pose);
			return;
		}

		vr::TrackedDevicePose_t& devicePose = devicePoses[index];

		//参照元のトラッキングステータスを継承させる(Reject無効化時に意味あり)
		pose.poseIsValid = devicePose.bPoseIsValid;
		pose.result = devicePose.eTrackingResult;

		//デバイスのトラッキング状態が正常なら
		if (devicePose.bPoseIsValid) {
			//デバイスの変換行列を取得し、Eigenの行列に変換
			float* m = (float*)(devicePose.mDeviceToAbsoluteTracking.m);

			Eigen::Affine3d rootDeviceToAbsoluteTracking;
			rootDeviceToAbsoluteTracking.matrix() <<
				m[0 * 4 + 0], m[0 * 4 + 1], m[0 * 4 + 2], m[0 * 4 + 3],
				m[1 * 4 + 0], m[1 * 4 + 1], m[1 * 4 + 2], m[1 * 4 + 3],
				m[2 * 4 + 0], m[2 * 4 + 1], m[2 * 4 + 2], m[2 * 4 + 3],
				0.0, 0.0, 0.0, 1.0;

			//位置の座標系をデバイス基準にする
			Eigen::Translation3d pos(rootDeviceToAbsoluteTracking.translation());
			pose.vecWorldFromDriverTranslation[0] = pos.x();
			pose.vecWorldFromDriverTranslation[1] = pos.y();
			pose.vecWorldFromDriverTranslation[2] = pos.z();

			//回転の座標系をルーム基準にしたりデバイス基準にしたりする
			Eigen::Quaterniond rot;
			switch (m_rawPose.mode) {
			case ReferMode_t::Follow:
				rot = Eigen::Quaterniond(RoomToDriverAffin.rotation());
				break;
			case ReferMode_t::Joint:
			default:
				rot = Eigen::Quaterniond(rootDeviceToAbsoluteTracking.rotation());
				break;
			}

			pose.qWorldFromDriverRotation.x = rot.x();
			pose.qWorldFromDriverRotation.y = rot.y();
			pose.qWorldFromDriverRotation.z = rot.z();
			pose.qWorldFromDriverRotation.w = rot.w();

			//デバイス = 接続済み・有効・特殊座標系
			return;
		}
		else {
			//デバイス = 接続済み・無効
			RejectTracking(pose);
			return;
		}
	}

	//デバイスをシリアル番号から探す
	int HMDDeviceServerDriver::SearchDevice(vr::TrackedDevicePose_t* poses, string serial)
	{
		IVRProperties* props = VRPropertiesRaw();
		CVRPropertyHelpers* helper = VRProperties();

		//デバイスシリアルが空白
		if (serial.empty()) {
			//探索エラーを返す
			return k_unTrackedDeviceIndexInvalid;
		}

		//デバイスシリアルがHMD(でかつ、HMD特別処理が有効なら)
		if (serial == "HMD" && Config::GetInstance()->GetHMDisIndex0()) {
			//HMDが接続OKなら
			if (poses[k_unTrackedDeviceIndex_Hmd].bDeviceIsConnected) {
				//HMDのインデックスを返す
				return k_unTrackedDeviceIndex_Hmd;
			}
			else {
				//(HMDがつながっていないのは普通ありえないが)探索エラーを返す
				return k_unTrackedDeviceIndexInvalid;
			}
		}

		//デバイスをOpenVR index順に調べる
		for (uint32_t i = 0; i < k_unMaxTrackedDeviceCount; i++) {
			//そのデバイスがつながっていないなら次のデバイスへ
			if (poses[i].bDeviceIsConnected != true) {
				continue;
			}

			//デバイスがつながっているので、シリアルナンバーを取得する
			PropertyContainerHandle_t h = props->TrackedDeviceToPropertyContainer(i);
			string SerialNumber = helper->GetStringProperty(h, ETrackedDeviceProperty::Prop_SerialNumber_String);

			//対象シリアルナンバーと比較し、違うデバイスなら、次のデバイスへ
			if (serial != SerialNumber) {
				continue;
			};

			//目的のデバイスを見つけたので返却
			return i;
		}
		//最後まで探したが、目的のデバイスは見つからなかった
		return k_unTrackedDeviceIndexInvalid;
	}

	//デバイスをトラッキング失敗状態にする
	void HMDDeviceServerDriver::RejectTracking(DriverPose_t& pose)
	{
		//(ただし、設定から有効な場合のみ。そうでない場合は無視してトラッキングを継続する)
		if (Config::GetInstance()->GetRejectWhenCannotTracking()) {
			//デバイス = 接続済み・無効
			pose.poseIsValid = false;
			pose.result = TrackingResult_Running_OutOfRange;
		}
	}

	DriverPose_t HMDDeviceServerDriver::RawPoseToPose()
	{
		DriverPose_t pose{ 0 };

		//ルーム変換行列の変換
		Eigen::Affine3d RoomToDriverAffin;
		RoomToDriverAffin = Config::GetInstance()->GetRoomToDriverMatrix();

		Eigen::Translation3d pos(RoomToDriverAffin.translation());
		Eigen::Quaterniond rot(RoomToDriverAffin.rotation());

		//OpenVR姿勢へ、一旦通常のデータを書き込む
		pose.poseTimeOffset = m_rawPose.timeoffset;

		if (m_rawPose.roomToDriver) {
			//ルーム空間に設定
			pose.qWorldFromDriverRotation.x = rot.x();
			pose.qWorldFromDriverRotation.y = rot.y();
			pose.qWorldFromDriverRotation.z = rot.z();
			pose.qWorldFromDriverRotation.w = rot.w();

			pose.vecWorldFromDriverTranslation[0] = pos.x();
			pose.vecWorldFromDriverTranslation[1] = pos.y();
			pose.vecWorldFromDriverTranslation[2] = pos.z();
		}
		else {
			//ドライバ生空間に設定
			pose.qWorldFromDriverRotation.x = 0;
			pose.qWorldFromDriverRotation.y = 0;
			pose.qWorldFromDriverRotation.z = 0;
			pose.qWorldFromDriverRotation.w = 1;

			pose.vecWorldFromDriverTranslation[0] = 0;
			pose.vecWorldFromDriverTranslation[1] = 0;
			pose.vecWorldFromDriverTranslation[2] = 0;
		}

		pose.vecWorldFromDriverTranslation[0] = pos.x();
		pose.vecWorldFromDriverTranslation[1] = pos.y();
		pose.vecWorldFromDriverTranslation[2] = pos.z();

		pose.qDriverFromHeadRotation = VMTDriver::HmdQuaternion_Identity;

		pose.vecDriverFromHeadTranslation[0] = 0.0f;
		pose.vecDriverFromHeadTranslation[1] = 0.0f;
		pose.vecDriverFromHeadTranslation[2] = 0.0f;

		pose.vecPosition[0] = m_rawPose.x;
		pose.vecPosition[1] = m_rawPose.y;
		pose.vecPosition[2] = m_rawPose.z;

		pose.vecVelocity[0] = 0.0f;
		pose.vecVelocity[1] = 0.0f;
		pose.vecVelocity[2] = 0.0f;

		pose.vecAcceleration[0] = 0.0f;
		pose.vecAcceleration[1] = 0.0f;
		pose.vecAcceleration[2] = 0.0f;

		pose.qRotation.x = m_rawPose.qx;
		pose.qRotation.y = m_rawPose.qy;
		pose.qRotation.z = m_rawPose.qz;
		pose.qRotation.w = m_rawPose.qw;

		pose.vecAngularVelocity[0] = 0.0f;
		pose.vecAngularVelocity[1] = 0.0f;
		pose.vecAngularVelocity[2] = 0.0f;

		pose.vecAngularAcceleration[0] = 0.0f;
		pose.vecAngularAcceleration[1] = 0.0f;
		pose.vecAngularAcceleration[2] = 0.0f;

		pose.result = TrackingResult_Running_OK;

		pose.poseIsValid = true;
		pose.willDriftInYaw = false;
		pose.shouldApplyHeadModel = false;

		pose.deviceIsConnected = true;

		//デバイスが有効でない場合、ステータスを無効で更新し、ここで返却
		if (m_rawPose.enable == 0) {
			pose.deviceIsConnected = false;
			pose.poseIsValid = false;
			pose.result = ETrackingResult::TrackingResult_Calibrating_OutOfRange;
			//デバイス = 非接続・無効
			return pose;
		}

		//ルームマトリクスが設定されていないとき、ステータスを無効で更新し、ここで返却
		if (!Config::GetInstance()->GetRoomMatrixStatus()) {
			//デバイス = 接続済み・無効
			RejectTracking(pose);
			return pose;
		}

		//速度エミュレーションが有効な場合、速度・各速度の計算を行い、更新する
		if (m_enableVelocity)
		{
			CalcVelocity(pose);
			CompensateVelocity(pose);
		}

		//トラッキングモードに合わせて処理する
		switch (m_rawPose.mode) {
		case ReferMode_t::None: {
			//通常のトラッキングモードの場合、何もしない
			//デバイス = 接続済み・有効・ルーム座標系

			//do noting
			break;
		}

		case ReferMode_t::Follow: {
			//デバイス = 接続済み・有効・デバイス位置座標系
			CalcJoint(pose, m_rawPose.root_sn, ReferMode_t::Follow, RoomToDriverAffin);
			break;
		}

		case ReferMode_t::Joint: {
			//デバイス = 接続済み・有効・デバイス位置回転座標系
			CalcJoint(pose, m_rawPose.root_sn, ReferMode_t::Joint, RoomToDriverAffin);
			break;
		}
		default: {
			//デバイス = 接続済み・無効
			RejectTracking(pose);
			break;
		}
		}
		return pose;
	}

	void HMDDeviceServerDriver::CalcVelocity(DriverPose_t& pose) {
		double smoothingFactor = Config::GetInstance()->GetVelocitySmoothingFactor();
		if (smoothingFactor <= 0.00)
			return;

		if (smoothingFactor > 1.0)
			smoothingFactor = 1.0;

		double delta_time = std::chrono::duration_cast<std::chrono::microseconds>(m_rawPose.time - m_lastRawPose.time).count() / (1000.0 * 1000.0);
		if (delta_time <= std::numeric_limits<double>::epsilon())
			return;

		// Linear Velocity
		Eigen::Vector3d newVel(
			(m_rawPose.x - m_lastRawPose.x) / delta_time,
			(m_rawPose.y - m_lastRawPose.y) / delta_time,
			(m_rawPose.z - m_lastRawPose.z) / delta_time
		);

		Eigen::Vector3d posVelocity(
			smoothingFactor * newVel.x() + (1.0 - smoothingFactor) * m_lastVecVeloctiy[0],
			smoothingFactor * newVel.y() + (1.0 - smoothingFactor) * m_lastVecVeloctiy[1],
			smoothingFactor * newVel.z() + (1.0 - smoothingFactor) * m_lastVecVeloctiy[2]
		);

		// Angular Velocity
		Eigen::Quaterniond newQuat(m_rawPose.qw, m_rawPose.qx, m_rawPose.qy, m_rawPose.qz);
		Eigen::Quaterniond oldQuat(m_lastRawPose.qw, m_lastRawPose.qx, m_lastRawPose.qy, m_lastRawPose.qz);

		Eigen::Vector3d vecAngularVelocity = AngularVelocityBetweenQuats(oldQuat, newQuat, delta_time);

		Eigen::Vector3d angVelocity(
			smoothingFactor * vecAngularVelocity.x() + (1.0 - smoothingFactor) * m_lastAngVeloctiy[0],
			smoothingFactor * vecAngularVelocity.y() + (1.0 - smoothingFactor) * m_lastAngVeloctiy[1],
			smoothingFactor * vecAngularVelocity.z() + (1.0 - smoothingFactor) * m_lastAngVeloctiy[2]
		);

		pose.vecVelocity[0] = posVelocity[0];
		pose.vecVelocity[1] = posVelocity[1];
		pose.vecVelocity[2] = posVelocity[2];
		pose.vecAngularVelocity[0] = angVelocity[0];
		pose.vecAngularVelocity[1] = angVelocity[1];
		pose.vecAngularVelocity[2] = angVelocity[2];

		m_lastVecVeloctiy[0] = posVelocity[0];
		m_lastVecVeloctiy[1] = posVelocity[1];
		m_lastVecVeloctiy[2] = posVelocity[2];
		m_lastAngVeloctiy[0] = angVelocity[0];
		m_lastAngVeloctiy[1] = angVelocity[1];
		m_lastAngVeloctiy[2] = angVelocity[2];
	}

	void HMDDeviceServerDriver::CompensateVelocity(DriverPose_t& pose) {
		double delta_time = std::chrono::duration_cast<std::chrono::microseconds>(m_rawPose.time - m_lastRawPose.time).count() / (1000.0 * 1000.0);
		if (delta_time <= std::numeric_limits<double>::epsilon())
			return;

		// Compensate linear velocity
		Eigen::Vector3d compPosition(
			pose.vecPosition[0] - (pose.vecVelocity[0] * delta_time),
			pose.vecPosition[1] - (pose.vecVelocity[1] * delta_time),
			pose.vecPosition[2] - (pose.vecVelocity[2] * delta_time)
		);

		// Compensate angular velocity
		Eigen::Quaterniond angularCorrection = QuaternionFromAngularVelocity(
			Eigen::Vector3d(pose.vecAngularVelocity[0], pose.vecAngularVelocity[1], pose.vecAngularVelocity[2]), delta_time
		);
		Eigen::Quaterniond newQuat(pose.qRotation.w, pose.qRotation.x, pose.qRotation.y, pose.qRotation.z);
		Eigen::Quaterniond compRotation = newQuat * angularCorrection.conjugate();

		pose.vecPosition[0] = compPosition[0];
		pose.vecPosition[1] = compPosition[1];
		pose.vecPosition[2] = compPosition[2];
		pose.qRotation.x = compRotation.x();
		pose.qRotation.y = compRotation.y();
		pose.qRotation.z = compRotation.z();
		pose.qRotation.w = compRotation.w();
	}

	Eigen::Quaterniond HMDDeviceServerDriver::QuaternionFromAngularVelocity(
		const Eigen::Vector3d& angularVelocity, double deltaTime
	) {
		// Only convert to quaternion if angular velocity is significant
		if (angularVelocity.norm() < std::numeric_limits<double>::epsilon()) {
			return Eigen::Quaterniond::Identity(); // No rotation
		}

		// Create the quaternion for angular velocity
		double halfTheta = angularVelocity.norm() * deltaTime * 0.5;
		Eigen::Vector3d axis = angularVelocity.normalized();
		double sinHalfAngle = sin(halfTheta);

		return Eigen::Quaterniond(cos(halfTheta), axis.x() * sinHalfAngle, axis.y() * sinHalfAngle, axis.z() * sinHalfAngle);
	}

	// Sourced from https://mariogc.com/post/angular-velocity-quaternions/
	Eigen::Vector3d HMDDeviceServerDriver::AngularVelocityBetweenQuats(
		const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2, double dt
	) {
		double r = (2.0f / dt);

		return Eigen::Vector3d(
			(q1.w() * q2.x() - q1.x() * q2.w() - q1.y() * q2.z() + q1.z() * q2.y()) * r,
			(q1.w() * q2.y() + q1.x() * q2.z() - q1.y() * q2.w() - q1.z() * q2.x()) * r,
			(q1.w() * q2.z() - q1.x() * q2.y() + q1.y() * q2.x() - q1.z() * q2.w()) * r);
	}

	//仮想デバイスからOpenVRへデバイスの登録を依頼する
	void HMDDeviceServerDriver::RegisterToVRSystem()
	{
		// Set up display first.
		if (!m_displayValid || !m_renderValid)
			return;

		if (!m_alreadyRegistered && !m_registrationInProgress)
		{
			VRServerDriverHost()->TrackedDeviceAdded(m_serial.c_str(), ETrackedDeviceClass::TrackedDeviceClass_HMD, this);
			m_registrationInProgress = true;
		}
	}

	//仮想デバイスの状態をリセットする
	void HMDDeviceServerDriver::Reset()
	{
		m_poweron = false; //電源オフ状態にする

		if (!m_alreadyRegistered) { return; }
		DriverPose_t pose{ 0 };
		pose.qRotation = VMTDriver::HmdQuaternion_Identity;
		pose.qWorldFromDriverRotation = VMTDriver::HmdQuaternion_Identity;
		pose.qDriverFromHeadRotation = VMTDriver::HmdQuaternion_Identity;
		pose.deviceIsConnected = false;
		pose.poseIsValid = false;
		pose.result = ETrackingResult::TrackingResult_Calibrating_OutOfRange;
		SetPose(pose);

		UpdatePoseToVRSystem();
	}

	//仮想デバイスからOpenVRへデバイスの姿勢の更新を通知する(サーバーから毎フレームコールされる)
	void HMDDeviceServerDriver::UpdatePoseToVRSystem()
	{
		if (!m_alreadyRegistered) { return; }
		//姿勢を更新
		VRServerDriverHost()->TrackedDevicePoseUpdated(m_deviceIndex, GetPose(), sizeof(DriverPose_t));
	}

	//仮想デバイスでOpenVRイベントを処理する(サーバーからイベントがあるタイミングでコールされる)
	void HMDDeviceServerDriver::ProcessEvent(VREvent_t& VREvent)
	{
		//未登録 or 電源オフ時は反応しない
		if (!m_alreadyRegistered || !m_poweron) {
			return;
		}

		//異常値を除去(なんで送られてくるんだ？)
		if (VREvent_VendorSpecific_Reserved_End < VREvent.eventType) {
			return;
		}

		if (m_displayValid && m_frame > frameCycle) {
			float frameRate = VRProperties()->GetFloatProperty(m_propertyContainer, Prop_DisplayFrequency_Float);

			OSCReceiver::SendHmdInfo(frameRate, m_displaySettings.display_w, m_displaySettings.display_h, m_displaySettings.directMode);

			m_frame = 0;
		}
		m_frame++;
	}

	/**
	 * 指定されたシリアルナンバーのデバイスの姿勢を取得する
	 * @param out_pose デバイスの姿勢が格納される
	 * @param in_serialNumber 姿勢を取得するデバイスのシリアルナンバー
	 * @return 取得に成功したか
	 */
	bool HMDDeviceServerDriver::GetDevicePose(Eigen::Affine3d& out_pose, const char* in_serialNumber)
	{
		const std::string serialNumber = in_serialNumber ? in_serialNumber : "";

		//OpenVRから全トラッキングデバイスの情報を取得する
		vr::TrackedDevicePose_t devicePoses[k_unMaxTrackedDeviceCount]{};
		VRServerDriverHost()->GetRawTrackedDevicePoses(0.0f, devicePoses, k_unMaxTrackedDeviceCount);

		//接続済みのデバイスの中から、シリアル番号でデバイスを検索する
		int index = SearchDevice(devicePoses, serialNumber);

		//探索エラーが帰ってきたら失敗
		if (index == k_unTrackedDeviceIndexInvalid) {
			return false;
		}

		vr::TrackedDevicePose_t& devicePose = devicePoses[index];

		//デバイスのトラッキング状態が正常なら
		if (devicePose.bPoseIsValid)
		{
			//デバイスの変換行列を取得し、Eigenの行列に変換
			float* m = (float*)(devicePose.mDeviceToAbsoluteTracking.m);

			out_pose.matrix() <<
				m[0 * 4 + 0], m[0 * 4 + 1], m[0 * 4 + 2], m[0 * 4 + 3],
				m[1 * 4 + 0], m[1 * 4 + 1], m[1 * 4 + 2], m[1 * 4 + 3],
				m[2 * 4 + 0], m[2 * 4 + 1], m[2 * 4 + 2], m[2 * 4 + 3],
				0.0, 0.0, 0.0, 1.0;

			return true;
		}
		return false;
	}

	//** OpenVR向け関数群 **

	//OpenVRからのデバイス有効化コール
	EVRInitError HMDDeviceServerDriver::Activate(uint32_t unObjectId)
	{
		//OpenVR Indexの記録
		m_deviceIndex = unObjectId;

		//OpenVR プロパティコンテナの保持
		m_propertyContainer = VRProperties()->TrackedDeviceToPropertyContainer(unObjectId);

		std::string registeredDeviceType = Config::GetInstance()->GetDriverName();
		registeredDeviceType += "/";
		registeredDeviceType += m_serial.c_str();
		VRProperties()->SetStringProperty(m_propertyContainer, Prop_RegisteredDeviceType_String, registeredDeviceType.c_str());
		VRProperties()->SetStringProperty(m_propertyContainer, Prop_ModelNumber_String, m_serial.c_str());
		VRProperties()->SetStringProperty(m_propertyContainer, Prop_SerialNumber_String, m_serial.c_str());

		std::string trackingSystemName = Config::GetInstance()->GetDriverName();
		VRProperties()->SetStringProperty(m_propertyContainer, Prop_TrackingSystemName_String, trackingSystemName.c_str());
		VRProperties()->SetUint64Property(m_propertyContainer, Prop_CurrentUniverseId_Uint64, TRACKING_UNIVERSE_ID);
		VRProperties()->SetUint64Property(m_propertyContainer, Prop_PreviousUniverseId_Uint64, TRACKING_UNIVERSE_ID);

		VRProperties()->SetStringProperty(m_propertyContainer, Prop_ManufacturerName_String, "HTC");
		VRProperties()->SetStringProperty(m_propertyContainer, Prop_ResourceRoot_String, "htc");
		VRProperties()->SetInt32Property(m_propertyContainer, Prop_DeviceClass_Int32, TrackedDeviceClass_HMD);

		VRProperties()->SetFloatProperty(m_propertyContainer, Prop_UserIpdMeters_Float, m_userIpdMeters);
		VRProperties()->SetFloatProperty(m_propertyContainer, Prop_UserHeadToEyeDepthMeters_Float, 0.0f);
		VRProperties()->SetFloatProperty(m_propertyContainer, Prop_DisplayFrequency_Float, static_cast<float>(m_displaySettings.frameRate));
		VRProperties()->SetFloatProperty(m_propertyContainer, Prop_SecondsFromVsyncToPhotons_Float, 1.f / m_displaySettings.frameRate);

		//VRProperties()->SetBoolProperty(m_propertyContainer, Prop_DisplayDebugMode_Bool, true);

		VRProperties()->SetBoolProperty(m_propertyContainer, Prop_CanUnifyCoordinateSystemWithHmd_Bool, true);
		VRProperties()->SetBoolProperty(m_propertyContainer, Prop_ContainsProximitySensor_Bool, false);
		VRProperties()->SetBoolProperty(m_propertyContainer, Prop_ReportsTimeSinceVSync_Bool, false);
		VRProperties()->SetBoolProperty(m_propertyContainer, Prop_DisplayAllowNightMode_Bool, true);

		VRProperties()->SetBoolProperty(m_propertyContainer, Prop_IsOnDesktop_Bool, false);
		VRProperties()->SetBoolProperty(m_propertyContainer, Prop_HasDisplayComponent_Bool, true);
		VRProperties()->SetBoolProperty(m_propertyContainer, Prop_HasCameraComponent_Bool, false);
		VRProperties()->SetBoolProperty(m_propertyContainer, Prop_HasDriverDirectModeComponent_Bool, false);
		VRProperties()->SetBoolProperty(m_propertyContainer, Prop_HasVirtualDisplayComponent_Bool, false);

		VRProperties()->SetStringProperty(m_propertyContainer, Prop_NamedIconPathDeviceOff_String, "{htc}/icons/headset_status_off.png");
		VRProperties()->SetStringProperty(m_propertyContainer, Prop_NamedIconPathDeviceSearching_String, "{htc}/icons/headset_status_searching.gif");
		VRProperties()->SetStringProperty(m_propertyContainer, Prop_NamedIconPathDeviceSearchingAlert_String, "{htc}/icons/headset_status_searching_alert.gif");
		VRProperties()->SetStringProperty(m_propertyContainer, Prop_NamedIconPathDeviceReady_String, "{htc}/icons/headset_status_ready.png");
		VRProperties()->SetStringProperty(m_propertyContainer, Prop_NamedIconPathDeviceReadyAlert_String, "{htc}/icons/headset_status_standby_alert.png");
		VRProperties()->SetStringProperty(m_propertyContainer, Prop_NamedIconPathDeviceNotReady_String, "{htc}/icons/headset_status_error.png");
		VRProperties()->SetStringProperty(m_propertyContainer, Prop_NamedIconPathDeviceStandby_String, "{htc}/icons/headset_status_standby.png");
		VRProperties()->SetStringProperty(m_propertyContainer, Prop_NamedIconPathDeviceAlertLow_String, "{htc}/icons/headset_status_error.png");

		if (m_displaySettings.directMode)
		{
			VRProperties()->SetInt32Property(m_propertyContainer, Prop_EdidVendorID_Int32, m_displaySettings.vendorId);
			VRProperties()->SetInt32Property(m_propertyContainer, Prop_EdidProductID_Int32, m_displaySettings.productId);
		}

		m_alreadyRegistered = true;
		m_registrationInProgress = false;
		return EVRInitError::VRInitError_None;
	}

	//OpenVRからのデバイス無効化コール
	void HMDDeviceServerDriver::Deactivate()
	{
		m_deviceIndex = k_unTrackedDeviceIndexInvalid;
		m_propertyContainer = k_ulInvalidPropertyContainer;
	}

	//OpenVRからのデバイス電源オフコール
	void HMDDeviceServerDriver::EnterStandby()
	{
		//電源オフ要求が来た
		Reset();
	}

	//OpenVRからのデバイス固有機能の取得(ない場合はnullptrを返す)
	void* HMDDeviceServerDriver::GetComponent(const char* pchComponentNameAndVersion)
	{
		if (!pchComponentNameAndVersion)
		{
			return nullptr;
		}

		if (std::string{pchComponentNameAndVersion} == vr::IVRDisplayComponent_Version)
		{
			return static_cast<vr::IVRDisplayComponent *>(this);
		}

		return nullptr;

	}

	//OpenVRからのデバイスのデバッグリクエスト
	void HMDDeviceServerDriver::DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize)
	{
		//デバッグ用
		//Log::printf("DebugRequest: %s", pchRequest);
		if (unResponseBufferSize > 0) {
			pchResponseBuffer[0] = '\0';
		}
	}

	//OpenVRからのデバイス姿勢取得
	DriverPose_t HMDDeviceServerDriver::GetPose()
	{
		//現在のOpenVR向け姿勢を返却する
		return m_pose;
	}

	void HMDDeviceServerDriver::GetWindowBounds(int32_t* pnX, int32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight)
	{
		*pnX = m_displaySettings.display_x;
		*pnY = m_displaySettings.display_y;
		*pnWidth = m_displaySettings.display_w;
		*pnHeight = m_displaySettings.display_h;
	}

	bool HMDDeviceServerDriver::IsDisplayOnDesktop()
	{
		if (m_displaySettings.directMode)
		{
			return false;
		}

		return true;
	}

	bool HMDDeviceServerDriver::IsDisplayRealDisplay()
	{
		return true;
	}

	void HMDDeviceServerDriver::GetRecommendedRenderTargetSize(uint32_t* pnWidth, uint32_t* pnHeight)
	{
		*pnWidth = m_displaySettings.render_w;
		*pnHeight = m_displaySettings.render_h;
	}

	void HMDDeviceServerDriver::GetEyeOutputViewport(EVREye eEye, uint32_t* pnX, uint32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight)
	{
		*pnY = 0;
		*pnWidth = m_displaySettings.display_w / 2;
		*pnHeight = m_displaySettings.display_h;

		if (eEye == Eye_Left) {
			*pnX = 0;
		}
		else {
			*pnX = m_displaySettings.display_w / 2;
		}
	}

	void HMDDeviceServerDriver::GetProjectionRaw(EVREye eEye, float* pfLeft, float* pfRight, float* pfTop, float* pfBottom)
	{
		*pfTop = -tan(m_renderSettings.vFov / 2);
		*pfBottom = tan(m_renderSettings.vFov / 2);
		*pfLeft = -tan(m_renderSettings.hFov / 2);
		*pfRight = tan(m_renderSettings.hFov / 2);
	}

	DistortionCoordinates_t HMDDeviceServerDriver::ComputeDistortion(EVREye eEye, float fU, float fV)
	{
		const float r2 = (fU - 0.5f) * (fU - 0.5f) + (fV - 0.5f) * (fV - 0.5f);
		const float r4 = r2 * r2;
		const float dist = (1.0f + m_renderSettings.distortionK0 * r2 + m_renderSettings.distortionK1 * r4);

		float fBlueScale = (m_renderSettings.distortionScale + m_renderSettings.distortionBlueOffset);
		float fGreenScale = (m_renderSettings.distortionScale + m_renderSettings.distortionGreenOffset);
		float fRedScale = (m_renderSettings.distortionScale + m_renderSettings.distortionRedOffset);

		float fBlueU = (((fU * 2.0f - 1.0f) * dist) * fBlueScale + 1.0f) * 0.5f;
		float fBlueV = (((fV * 2.0f - 1.0f) * dist) * fBlueScale + 1.0f) * 0.5f;
		float fGreenU = (((fU * 2.0f - 1.0f) * dist) * fGreenScale + 1.0f) * 0.5f;
		float fGreenV = (((fV * 2.0f - 1.0f) * dist) * fGreenScale + 1.0f) * 0.5f;
		float fRedU = (((fU * 2.0f - 1.0f) * dist) * fRedScale + 1.0f) * 0.5f;
		float fRedV = (((fV * 2.0f - 1.0f) * dist) * fRedScale + 1.0f) * 0.5f;

		vr::DistortionCoordinates_t oDistortion{};
		oDistortion.rfBlue[0] = fBlueU;
		oDistortion.rfBlue[1] = fBlueV;
		oDistortion.rfGreen[0] = fGreenU;
		oDistortion.rfGreen[1] = fGreenV;
		oDistortion.rfRed[0] = fRedU;
		oDistortion.rfRed[1] = fRedV;
		return oDistortion;
	}
}