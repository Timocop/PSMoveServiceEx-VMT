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

#include "TrackedDeviceServerDriver.h"

#define TRACKING_UNIVERSE_ID 6

namespace VMTDriver {
    //** 内部向け関数群 **

    //自動更新を有効にするか
    //仮想デバイスのコンストラクタ。(Listから暗黙的にコールされる)
    TrackedDeviceServerDriver::TrackedDeviceServerDriver()
    {
        m_deviceIndex = k_unTrackedDeviceIndexInvalid;
        m_propertyContainer = k_ulInvalidPropertyContainer;
    }

    //仮想デバイスのデストラクタ。(Listから暗黙的にコールされる)
    TrackedDeviceServerDriver::~TrackedDeviceServerDriver()
    {
    }

    //仮想デバイスにシリアル番号を設定
    void TrackedDeviceServerDriver::SetDeviceSerial(string serial)
    {
        m_serial = serial;
    }

    //仮想デバイスに内部Indexを設定
    void TrackedDeviceServerDriver::SetObjectIndex(uint32_t idx)
    {
        m_index = idx;
    }

    //仮想デバイスにOpenVR姿勢を設定
    void TrackedDeviceServerDriver::SetPose(DriverPose_t pose)
    {
        m_pose = pose;
    }

    //仮想デバイスに内部姿勢を設定
    void TrackedDeviceServerDriver::SetRawPose(RawPose rawPose)
    {
        m_poweron = true; //有効な姿勢なので電源オン状態にする

		//自動更新が無効ならば内部姿勢を保存し、OpenVR姿勢を更新する
		m_lastRawPose = m_rawPose; //差分を取るために前回値を取っておく
		m_rawPose = rawPose;

		SetPose(RawPoseToPose());
    }

	void TrackedDeviceServerDriver::SetVelocity(bool enable)
	{
		m_enableVelocity = enable;
	}

    //Joint計算を行う
    void TrackedDeviceServerDriver::CalcJoint(DriverPose_t& pose, string serial, ReferMode_t mode, Eigen::Affine3d& RoomToDriverAffin) {
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
    int TrackedDeviceServerDriver::SearchDevice(vr::TrackedDevicePose_t* poses, string serial)
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
    void TrackedDeviceServerDriver::RejectTracking(DriverPose_t& pose)
    {
        //(ただし、設定から有効な場合のみ。そうでない場合は無視してトラッキングを継続する)
        if (Config::GetInstance()->GetRejectWhenCannotTracking()) {
            //デバイス = 接続済み・無効
            pose.poseIsValid = false;
            pose.result = TrackingResult_Running_OutOfRange;
        }
    }

    DriverPose_t TrackedDeviceServerDriver::RawPoseToPose()
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

	void TrackedDeviceServerDriver::CalcVelocity(DriverPose_t& pose) {
		double smoothingFactor = Config::GetInstance()->GetVelocitySmoothingFactor();
		if (smoothingFactor > 1.0)
			smoothingFactor = 1.0;
		if (smoothingFactor < 0.01)
			smoothingFactor = 0.01;

		if (smoothingFactor >= 1.0)
			return;

		if (m_rawPose.x != m_lastRawPose.x
			|| m_rawPose.y != m_lastRawPose.y
			|| m_rawPose.z != m_lastRawPose.z) {
			double delta_time = std::chrono::duration_cast<std::chrono::microseconds>(m_rawPose.time - m_lastVecTime).count() / (1000.0 * 1000.0);
			if (m_lastVecValid && delta_time > std::numeric_limits<double>::epsilon()) {
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

				pose.vecVelocity[0] = posVelocity[0];
				pose.vecVelocity[1] = posVelocity[1];
				pose.vecVelocity[2] = posVelocity[2];
				m_lastVecVeloctiy[0] = posVelocity[0];
				m_lastVecVeloctiy[1] = posVelocity[1];
				m_lastVecVeloctiy[2] = posVelocity[2];

				// Compensate Linear Velocity
				Eigen::Vector3d compPosition(
					pose.vecPosition[0] - (pose.vecVelocity[0] * delta_time),
					pose.vecPosition[1] - (pose.vecVelocity[1] * delta_time),
					pose.vecPosition[2] - (pose.vecVelocity[2] * delta_time)
				);

				pose.vecPosition[0] = compPosition[0];
				pose.vecPosition[1] = compPosition[1];
				pose.vecPosition[2] = compPosition[2];
				m_lastVec[0] = compPosition[0];
				m_lastVec[1] = compPosition[1];
				m_lastVec[2] = compPosition[2];
			}

			m_lastVecTime = m_rawPose.time;
			m_lastVecValid = true;
		}
		else
		{
			pose.vecVelocity[0] = m_lastVecVeloctiy[0];
			pose.vecVelocity[1] = m_lastVecVeloctiy[1];
			pose.vecVelocity[2] = m_lastVecVeloctiy[2];
			pose.vecPosition[0] = m_lastVec[0];
			pose.vecPosition[1] = m_lastVec[1];
			pose.vecPosition[2] = m_lastVec[2];
		}


		if (m_rawPose.qx != m_lastRawPose.qx
				|| m_rawPose.qy != m_lastRawPose.qy
				|| m_rawPose.qz != m_lastRawPose.qz
				|| m_rawPose.qw != m_lastRawPose.qw) {
			double delta_time = std::chrono::duration_cast<std::chrono::microseconds>(m_rawPose.time - m_lastAngTime).count() / (1000.0 * 1000.0);
			if (m_lastAngValid && delta_time > std::numeric_limits<double>::epsilon()) {
				// Angular Velocity
				Eigen::Quaterniond newQuat(m_rawPose.qw, m_rawPose.qx, m_rawPose.qy, m_rawPose.qz);
				Eigen::Quaterniond oldQuat(m_lastRawPose.qw, m_lastRawPose.qx, m_lastRawPose.qy, m_lastRawPose.qz);

				Eigen::Vector3d vecAngularVelocity = AngularVelocityBetweenQuats(oldQuat, newQuat, delta_time);

				Eigen::Vector3d angVelocity(
					smoothingFactor * vecAngularVelocity.x() + (1.0 - smoothingFactor) * m_lastAngVeloctiy[0],
					smoothingFactor * vecAngularVelocity.y() + (1.0 - smoothingFactor) * m_lastAngVeloctiy[1],
					smoothingFactor * vecAngularVelocity.z() + (1.0 - smoothingFactor) * m_lastAngVeloctiy[2]
				);

				pose.vecAngularVelocity[0] = angVelocity[0];
				pose.vecAngularVelocity[1] = angVelocity[1];
				pose.vecAngularVelocity[2] = angVelocity[2];
				m_lastAngVeloctiy[0] = angVelocity[0];
				m_lastAngVeloctiy[1] = angVelocity[1];
				m_lastAngVeloctiy[2] = angVelocity[2];

				// Compensate Angular Velocity
				Eigen::Quaterniond angularCorrection = QuaternionFromAngularVelocity(
					Eigen::Vector3d(pose.vecAngularVelocity[0], pose.vecAngularVelocity[1], pose.vecAngularVelocity[2]), delta_time
				);
				Eigen::Quaterniond compRotation = newQuat * angularCorrection.conjugate();

				pose.qRotation.x = compRotation.x();
				pose.qRotation.y = compRotation.y();
				pose.qRotation.z = compRotation.z();
				pose.qRotation.w = compRotation.w();
				m_lastAng[0] = compRotation.x();
				m_lastAng[1] = compRotation.y();
				m_lastAng[2] = compRotation.z();
				m_lastAng[3] = compRotation.w();
			}

			m_lastAngTime = m_rawPose.time;
			m_lastAngValid = true;
		}
		else
		{
			pose.vecAngularVelocity[0] = m_lastAngVeloctiy[0];
			pose.vecAngularVelocity[1] = m_lastAngVeloctiy[1];
			pose.vecAngularVelocity[2] = m_lastAngVeloctiy[2];
			pose.qRotation.x = m_lastAng[0];
			pose.qRotation.y = m_lastAng[1];
			pose.qRotation.z = m_lastAng[2];
			pose.qRotation.w = m_lastAng[3];
		}

	}

	Eigen::Quaterniond TrackedDeviceServerDriver::QuaternionFromAngularVelocity(
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
	Eigen::Vector3d TrackedDeviceServerDriver::AngularVelocityBetweenQuats(
		const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2, double dt
	) {
		double r = (2.0f / dt);

		return Eigen::Vector3d(
			(q1.w() * q2.x() - q1.x() * q2.w() - q1.y() * q2.z() + q1.z() * q2.y()) * r,
			(q1.w() * q2.y() + q1.x() * q2.z() - q1.y() * q2.w() - q1.z() * q2.x()) * r,
			(q1.w() * q2.z() - q1.x() * q2.y() + q1.y() * q2.x() - q1.z() * q2.w()) * r);
	}

    //仮想デバイスからOpenVRへデバイスの登録を依頼する
    void TrackedDeviceServerDriver::RegisterToVRSystem(eTrackerType type)
    {
        if (!m_alreadyRegistered && !m_registrationInProgress)
        {
            // $TODO: Add oculus/index emulation
			switch (type)
            {
			case eTrackerType::TrackerType_HtcBasestation:
				m_trackerType = type;

				VRServerDriverHost()->TrackedDeviceAdded(m_serial.c_str(), ETrackedDeviceClass::TrackedDeviceClass_TrackingReference, this);
				m_registrationInProgress = true;
				break;

			case eTrackerType::TrackerType_HtcRightController:
				m_enableVelocity = true;
				m_trackerType = type;

				VRServerDriverHost()->TrackedDeviceAdded(m_serial.c_str(), ETrackedDeviceClass::TrackedDeviceClass_Controller, this);
				m_registrationInProgress = true;
				break;

			case eTrackerType::TrackerType_HtcLeftController:
				m_enableVelocity = true;
				m_trackerType = type;

				VRServerDriverHost()->TrackedDeviceAdded(m_serial.c_str(), ETrackedDeviceClass::TrackedDeviceClass_Controller, this);
				m_registrationInProgress = true;
				break;

			case eTrackerType::TrackerType_HtcTracker:
				m_trackerType = type;

				VRServerDriverHost()->TrackedDeviceAdded(m_serial.c_str(), ETrackedDeviceClass::TrackedDeviceClass_GenericTracker, this);
				m_registrationInProgress = true;
				break;

			case eTrackerType::TrackerType_TrackingReference:
				m_trackerType = type;

				VRServerDriverHost()->TrackedDeviceAdded(m_serial.c_str(), ETrackedDeviceClass::TrackedDeviceClass_TrackingReference, this);
				m_registrationInProgress = true;
				break;

			case eTrackerType::TrackerType_RightController:
				m_enableVelocity = true;
				m_trackerType = type;

                VRServerDriverHost()->TrackedDeviceAdded(m_serial.c_str(), ETrackedDeviceClass::TrackedDeviceClass_Controller, this);
				m_registrationInProgress = true;
                break;

			case eTrackerType::TrackerType_LeftController:
				m_enableVelocity = true;
				m_trackerType = type;

                VRServerDriverHost()->TrackedDeviceAdded(m_serial.c_str(), ETrackedDeviceClass::TrackedDeviceClass_Controller, this);
				m_registrationInProgress = true;
                break;

			case eTrackerType::TrackerType_GenericTracker:
				m_trackerType = type;

                VRServerDriverHost()->TrackedDeviceAdded(m_serial.c_str(), ETrackedDeviceClass::TrackedDeviceClass_GenericTracker, this);
				m_registrationInProgress = true;
                break;

            default:
                break;
            }
        }
    }


    //仮想デバイスからOpenVRへデバイスのボタン状態の更新を通知する
    void TrackedDeviceServerDriver::UpdateButtonInput(uint32_t index, bool value, double timeoffset)
    {
        if (!m_alreadyRegistered) { return; }
        if (0 <= index && index <= 7)
        {
            VRDriverInput()->UpdateBooleanComponent(ButtonComponent[index], value, timeoffset);
        }
    }

    //仮想デバイスからOpenVRへデバイスのトリガー(1軸)状態の更新を通知する
    void TrackedDeviceServerDriver::UpdateTriggerInput(uint32_t index, float value, double timeoffset)
    {
        if (!m_alreadyRegistered) { return; }
        if (value > 1.0) {
            value = 1.0;
        }
        if (value < 0) {
            value = 0;
        }
        if (isnan(value)) {
            value = 0;
        }

        if (0 <= index && index <= 1)
        {
            VRDriverInput()->UpdateScalarComponent(TriggerComponent[index], value, timeoffset);
        }
    }

	//仮想デバイスからOpenVRへデバイスのジョイスティック(2軸)状態の更新を通知する
	void TrackedDeviceServerDriver::UpdateJoystickInput(uint32_t index, float x, float y, double timeoffset)
	{
		if (!m_alreadyRegistered) { return; }
		if (index == 0)
		{
			VRDriverInput()->UpdateScalarComponent(JoystickComponent[index + 0], x, timeoffset);
			VRDriverInput()->UpdateScalarComponent(JoystickComponent[index + 1], y, timeoffset);
		}
	}

	void TrackedDeviceServerDriver::UpdateBatteryProperty(float value)
	{
		if (!m_alreadyRegistered) { return; } 
		if(m_propertyContainer == k_ulInvalidPropertyContainer) { return; }

		if (value > 1.0) {
			value = 1.0;
		}
		if (value < 0) {
			value = 0;
		}
		if (isnan(value)) {
			value = 0;
		}

		VRProperties()->SetFloatProperty(m_propertyContainer, Prop_DeviceBatteryPercentage_Float, value);
	}

    //仮想デバイスの状態をリセットする
    void TrackedDeviceServerDriver::Reset()
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

        //全状態を初期化する
        for (int i = 0; i < 16; i++) {
            UpdateButtonInput(i, false, 0);
            UpdateTriggerInput(i, 0, 0);
            UpdateJoystickInput(i, 0, 0, 0);
        }

		UpdatePoseToVRSystem();
    }

    //仮想デバイスからOpenVRへデバイスの姿勢の更新を通知する(サーバーから毎フレームコールされる)
    void TrackedDeviceServerDriver::UpdatePoseToVRSystem()
    {
        if (!m_alreadyRegistered) { return; }
        //姿勢を更新
        VRServerDriverHost()->TrackedDevicePoseUpdated(m_deviceIndex, GetPose(), sizeof(DriverPose_t));
    }

    //仮想デバイスでOpenVRイベントを処理する(サーバーからイベントがあるタイミングでコールされる)
    void TrackedDeviceServerDriver::ProcessEvent(VREvent_t& VREvent)
    {
        //未登録 or 電源オフ時は反応しない
        if (!m_alreadyRegistered || !m_poweron) {
            return;
        }

        //異常値を除去(なんで送られてくるんだ？)
        if (VREvent_VendorSpecific_Reserved_End < VREvent.eventType) {
            return;
        }

        switch (VREvent.eventType)
        {
        case EVREventType::VREvent_Input_HapticVibration:
            //バイブレーション
            if (VREvent.data.hapticVibration.componentHandle == HapticComponent) {
                OSCReceiver::SendHaptic(m_index, VREvent.data.hapticVibration.fFrequency, VREvent.data.hapticVibration.fAmplitude, VREvent.data.hapticVibration.fDurationSeconds);
            }
            break;
        default:
            //デバッグ用
            //if (m_serial == "VMT_0") {
            //    Log::printf("Event: %d\n",VREvent.eventType);
            //}
            break;
        }
    }

    /**
     * 指定されたシリアルナンバーのデバイスの姿勢を取得する
     * @param out_pose デバイスの姿勢が格納される
     * @param in_serialNumber 姿勢を取得するデバイスのシリアルナンバー
     * @return 取得に成功したか
     */
    bool TrackedDeviceServerDriver::GetDevicePose(Eigen::Affine3d& out_pose, const char* in_serialNumber)
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
    EVRInitError TrackedDeviceServerDriver::Activate(uint32_t unObjectId)
    {
        //OpenVR Indexの記録
        m_deviceIndex = unObjectId;

        //OpenVR プロパティコンテナの保持
        m_propertyContainer = VRProperties()->TrackedDeviceToPropertyContainer(unObjectId);

		switch (m_trackerType)
		{
		case eTrackerType::TrackerType_HtcBasestation:
		{
			if (Config::GetInstance()->GetOptoutTrackingRole()) {
				VRProperties()->SetInt32Property(m_propertyContainer, Prop_ControllerRoleHint_Int32, ETrackedControllerRole::TrackedControllerRole_OptOut); //手に割り当てないように
			}

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

			VRProperties()->SetStringProperty(m_propertyContainer, Prop_RenderModelName_String, "lh_basestation_vive");
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_ManufacturerName_String, "HTC");
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_ResourceRoot_String, "htc");
			VRProperties()->SetInt32Property(m_propertyContainer, Prop_DeviceClass_Int32, TrackedDeviceClass_TrackingReference);

			VRProperties()->SetStringProperty(m_propertyContainer, Prop_NamedIconPathDeviceOff_String, "{htc}/icons/base_status_off.png");
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_NamedIconPathDeviceSearching_String, "{htc}/icons/base_status_searching.gif");
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_NamedIconPathDeviceSearchingAlert_String, "{htc}/icons/base_status_searching_alert.gif");
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_NamedIconPathDeviceReady_String, "{htc}/icons/base_status_ready.png");
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_NamedIconPathDeviceReadyAlert_String, "{htc}/icons/base_status_ready_alert.png");
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_NamedIconPathDeviceNotReady_String, "{htc}/icons/base_status_error.png");
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_NamedIconPathDeviceStandby_String, "{htc}/icons/base_status_standby.png");
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_NamedIconPathDeviceAlertLow_String, "{htc}/icons/base_status_error.png");

			break;
		}
		case eTrackerType::TrackerType_HtcLeftController:
		case eTrackerType::TrackerType_HtcRightController:
		{
			if (m_trackerType == eTrackerType::TrackerType_HtcLeftController)
			{
				VRProperties()->SetInt32Property(m_propertyContainer, Prop_ControllerRoleHint_Int32, TrackedControllerRole_LeftHand);
				VRProperties()->SetStringProperty(m_propertyContainer, Prop_RegisteredDeviceType_String, "htc/vive_controllerLHR-F94B3BD8");
				//VRProperties()->SetStringProperty(m_propertyContainer, Prop_SerialNumber_String, "LHR-F94B3BD8");
			}
			else
			{
				VRProperties()->SetInt32Property(m_propertyContainer, Prop_ControllerRoleHint_Int32, TrackedControllerRole_RightHand);
				VRProperties()->SetStringProperty(m_propertyContainer, Prop_RegisteredDeviceType_String, "htc/vive_controllerLHR-F94B3BD9");
				//VRProperties()->SetStringProperty(m_propertyContainer, Prop_SerialNumber_String, "LHR-F94B3BD9");
			}

			//std::string registeredDeviceType = Config::GetInstance()->GetDriverName();
			//registeredDeviceType += "/";
			//registeredDeviceType += m_serial.c_str();
			//VRProperties()->SetStringProperty(m_propertyContainer, Prop_RegisteredDeviceType_String, registeredDeviceType.c_str());
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_SerialNumber_String, m_serial.c_str());

			std::string trackingSystemName = Config::GetInstance()->GetDriverName();
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_TrackingSystemName_String, trackingSystemName.c_str());
			VRProperties()->SetUint64Property(m_propertyContainer, Prop_CurrentUniverseId_Uint64, TRACKING_UNIVERSE_ID);
			VRProperties()->SetUint64Property(m_propertyContainer, Prop_PreviousUniverseId_Uint64, TRACKING_UNIVERSE_ID);

			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_WillDriftInYaw_Bool, false);
			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_DeviceIsWireless_Bool, true);
			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_DeviceIsCharging_Bool, false);
			VRProperties()->SetFloatProperty(m_propertyContainer, Prop_DeviceBatteryPercentage_Float, 1.f);
			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_Firmware_UpdateAvailable_Bool, false);
			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_Firmware_ManualUpdate_Bool, false);
			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_DeviceProvidesBatteryStatus_Bool, true);
			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_DeviceCanPowerOff_Bool, true);
			VRProperties()->SetInt32Property(m_propertyContainer, Prop_DeviceClass_Int32, TrackedDeviceClass_Controller);
			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_Firmware_ForceUpdateRequired_Bool, false);
			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_Identifiable_Bool, true);
			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_Firmware_RemindUpdate_Bool, false);
			VRProperties()->SetInt32Property(m_propertyContainer, Prop_Axis0Type_Int32, k_eControllerAxis_TrackPad);
			VRProperties()->SetInt32Property(m_propertyContainer, Prop_Axis1Type_Int32, k_eControllerAxis_Trigger);

			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_HasDisplayComponent_Bool, false);
			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_HasCameraComponent_Bool, false);
			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_HasDriverDirectModeComponent_Bool, false);
			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_HasVirtualDisplayComponent_Bool, false);
			VRProperties()->SetInt32Property(m_propertyContainer, Prop_ControllerHandSelectionPriority_Int32, 0);
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_ModelNumber_String, "Vive. Controller MV");
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_RenderModelName_String, "vr_controller_vive_1_5");
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_ManufacturerName_String, "HTC");
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_ResourceRoot_String, "htc");

			VRProperties()->SetStringProperty(m_propertyContainer, Prop_InputProfilePath_String, "{htc}/input/vive_controller_profile.json");
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_NamedIconPathDeviceOff_String, "{htc}/icons/controller_status_off.png");
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_NamedIconPathDeviceSearching_String, "{htc}/icons/controller_status_searching.gif");
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_NamedIconPathDeviceSearchingAlert_String, "{htc}/icons/controller_status_searching_alert.gif");
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_NamedIconPathDeviceReady_String, "{htc}/icons/controller_status_ready.png");
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_NamedIconPathDeviceReadyAlert_String, "{htc}/icons/controller_status_ready_alert.png");
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_NamedIconPathDeviceNotReady_String, "{htc}/icons/controller_status_error.png");
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_NamedIconPathDeviceStandby_String, "{htc}/icons/controller_status_off.png");
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_NamedIconPathDeviceAlertLow_String, "{htc}/icons/controller_status_ready_low.png");
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_ControllerType_String, "vive_controller");

			VRDriverInput()->CreateBooleanComponent(m_propertyContainer, "/input/system/click", &ButtonComponent[0]);
			VRDriverInput()->CreateBooleanComponent(m_propertyContainer, "/input/trigger/click", &ButtonComponent[1]);
			VRDriverInput()->CreateBooleanComponent(m_propertyContainer, "/input/trackpad/touch", &ButtonComponent[2]);
			VRDriverInput()->CreateBooleanComponent(m_propertyContainer, "/input/trackpad/click", &ButtonComponent[3]);
			VRDriverInput()->CreateBooleanComponent(m_propertyContainer, "/input/grip/click", &ButtonComponent[4]);
			VRDriverInput()->CreateBooleanComponent(m_propertyContainer, "/input/application_menu/click", &ButtonComponent[5]);

			VRDriverInput()->CreateScalarComponent(m_propertyContainer, "/input/trigger/value", &TriggerComponent[0], VRScalarType_Absolute, VRScalarUnits_NormalizedOneSided);
			VRDriverInput()->CreateScalarComponent(m_propertyContainer, "/input/trackpad/x", &JoystickComponent[0], VRScalarType_Absolute, VRScalarUnits_NormalizedTwoSided);
			VRDriverInput()->CreateScalarComponent(m_propertyContainer, "/input/trackpad/y", &JoystickComponent[1], VRScalarType_Absolute, VRScalarUnits_NormalizedTwoSided);

			VRDriverInput()->CreateHapticComponent(m_propertyContainer, "/output/haptic", &HapticComponent);
			break;
		}
		case eTrackerType::TrackerType_HtcTracker:
		{
			if (Config::GetInstance()->GetOptoutTrackingRole()) {
				VRProperties()->SetInt32Property(m_propertyContainer, Prop_ControllerRoleHint_Int32, ETrackedControllerRole::TrackedControllerRole_OptOut); //手に割り当てないように
			}

			std::string trackingSystemName = Config::GetInstance()->GetDriverName();
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_TrackingSystemName_String, trackingSystemName.c_str());
			VRProperties()->SetUint64Property(m_propertyContainer, Prop_CurrentUniverseId_Uint64, TRACKING_UNIVERSE_ID);
			VRProperties()->SetUint64Property(m_propertyContainer, Prop_PreviousUniverseId_Uint64, TRACKING_UNIVERSE_ID);

			VRProperties()->SetStringProperty(m_propertyContainer, Prop_ModelNumber_String, m_serial.c_str());
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_SerialNumber_String, m_serial.c_str());
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_RenderModelName_String, "{htc}vr_tracker_vive_1_0");
			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_WillDriftInYaw_Bool, false);
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_ManufacturerName_String, "HTC");
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_TrackingFirmwareVersion_String,
				"1541800000 RUNNER-WATCHMAN$runner-watchman@runner-watchman 2018-01-01 FPGA 512(2.56/0/0) BL 0 VRC 1541800000 Radio 1518800000");
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_HardwareRevision_String,
				"product 128 rev 2.5.6 lot 2000/0/0 0");

			VRProperties()->SetStringProperty(m_propertyContainer, Prop_ConnectedWirelessDongle_String, "D0000BE000");
			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_DeviceIsWireless_Bool, true);
			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_DeviceIsCharging_Bool, false);
			VRProperties()->SetFloatProperty(m_propertyContainer, Prop_DeviceBatteryPercentage_Float, 1.f);


			HmdMatrix34_t l_transform = { -1.f, 0.f, 0.f, 0.f, 0.f, 0.f, -1.f, 0.f, 0.f, -1.f, 0.f, 0.f };
			VRProperties()->SetProperty(m_propertyContainer, Prop_StatusDisplayTransform_Matrix34, &l_transform, sizeof(HmdMatrix34_t), k_unHmdMatrix34PropertyTag);

			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_Firmware_UpdateAvailable_Bool, false);
			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_Firmware_ManualUpdate_Bool, false);
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_Firmware_ManualUpdateURL_String,
				"https://developer.valvesoftware.com/wiki/SteamVR/HowTo_Update_Firmware");
			VRProperties()->SetUint64Property(m_propertyContainer, Prop_HardwareRevision_Uint64, 2214720000);
			VRProperties()->SetUint64Property(m_propertyContainer, Prop_FirmwareVersion_Uint64, 1541800000);
			VRProperties()->SetUint64Property(m_propertyContainer, Prop_FPGAVersion_Uint64, 512);
			VRProperties()->SetUint64Property(m_propertyContainer, Prop_VRCVersion_Uint64, 1514800000);
			VRProperties()->SetUint64Property(m_propertyContainer, Prop_RadioVersion_Uint64, 1518800000);
			VRProperties()->SetUint64Property(m_propertyContainer, Prop_DongleVersion_Uint64, 8933539758);

			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_DeviceProvidesBatteryStatus_Bool, true);
			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_DeviceCanPowerOff_Bool, true);
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_Firmware_ProgrammingTarget_String, Version.c_str());
			VRProperties()->SetInt32Property(m_propertyContainer, Prop_DeviceClass_Int32, TrackedDeviceClass_GenericTracker);
			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_Firmware_ForceUpdateRequired_Bool, false);
			//VRProperties()->SetUint64Property(m_propertyContainer, Prop_ParentDriver_Uint64, 8589934597);
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_ResourceRoot_String, "htc");

			std::string registeredDeviceType = Config::GetInstance()->GetDriverName();
			registeredDeviceType += "/";
			registeredDeviceType += m_serial.c_str();
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_RegisteredDeviceType_String, registeredDeviceType.c_str());

			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_Identifiable_Bool, false);
			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_Firmware_RemindUpdate_Bool, false);
			VRProperties()->SetInt32Property(m_propertyContainer, Prop_ControllerHandSelectionPriority_Int32, -1);

			VRProperties()->SetStringProperty(m_propertyContainer, Prop_NamedIconPathDeviceOff_String, "{htc}/icons/tracker_status_off.png");
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_NamedIconPathDeviceSearching_String, "{htc}/icons/tracker_status_searching.gif");
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_NamedIconPathDeviceSearchingAlert_String, "{htc}/icons/tracker_status_searching_alert.gif");
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_NamedIconPathDeviceReady_String, "{htc}/icons/tracker_status_ready.png");
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_NamedIconPathDeviceReadyAlert_String, "{htc}/icons/tracker_status_ready_alert.png");
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_NamedIconPathDeviceNotReady_String, "{htc}/icons/tracker_status_error.png");
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_NamedIconPathDeviceStandby_String, "{htc}/icons/tracker_status_standby.png");
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_NamedIconPathDeviceAlertLow_String, "{htc}/icons/tracker_status_ready_low.png");

			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_HasDisplayComponent_Bool, false);
			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_HasCameraComponent_Bool, false);
			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_HasDriverDirectModeComponent_Bool, false);
			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_HasVirtualDisplayComponent_Bool, false);

			VRProperties()->SetStringProperty(m_propertyContainer, Prop_InputProfilePath_String, "{htc}/input/vive_tracker_profile.json");
			VRProperties()->SetStringProperty(m_propertyContainer, 	Prop_ControllerType_String, "vive_tracker");

			// Create components
			VRDriverInput()->CreateBooleanComponent(m_propertyContainer, "/input/system/click", &ButtonComponent[0]);
			VRDriverInput()->CreateHapticComponent(m_propertyContainer, "/output/haptic", &HapticComponent);
			break;
		}
		default:
		{
			switch (m_trackerType)
			{
			case eTrackerType::TrackerType_TrackingReference:
			{
				if (Config::GetInstance()->GetOptoutTrackingRole()) {
					VRProperties()->SetInt32Property(m_propertyContainer, Prop_ControllerRoleHint_Int32, ETrackedControllerRole::TrackedControllerRole_OptOut); //手に割り当てないように
				}
				break;
			}
			case eTrackerType::TrackerType_RightController:
			{
				VRProperties()->SetInt32Property(m_propertyContainer, Prop_ControllerRoleHint_Int32, ETrackedControllerRole::TrackedControllerRole_RightHand);
				break;
			}
			case eTrackerType::TrackerType_LeftController:
			{
				VRProperties()->SetInt32Property(m_propertyContainer, Prop_ControllerRoleHint_Int32, ETrackedControllerRole::TrackedControllerRole_LeftHand);
				break;
			}
			case eTrackerType::TrackerType_GenericTracker:
			{
				if (Config::GetInstance()->GetOptoutTrackingRole()) {
					VRProperties()->SetInt32Property(m_propertyContainer, Prop_ControllerRoleHint_Int32, ETrackedControllerRole::TrackedControllerRole_OptOut); //手に割り当てないように
				}
				break;
			}
			}

			std::string driverName = Config::GetInstance()->GetDriverName();
			 
			std::string ressourcePath = "";
			ressourcePath += "{";
			ressourcePath += driverName.c_str();
			ressourcePath += "}";

			std::string renderModelName = "";
			renderModelName += ressourcePath.c_str();
			renderModelName += Config::GetInstance()->GetRessourceRenderModelPrefix(m_trackerType);
			renderModelName += "_rendermodel";

			std::string trackingSystemName = Config::GetInstance()->GetDriverName();
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_TrackingSystemName_String, trackingSystemName.c_str());
			VRProperties()->SetUint64Property(m_propertyContainer, Prop_CurrentUniverseId_Uint64, TRACKING_UNIVERSE_ID);
			VRProperties()->SetUint64Property(m_propertyContainer, Prop_PreviousUniverseId_Uint64, TRACKING_UNIVERSE_ID);

			VRProperties()->SetStringProperty(m_propertyContainer, Prop_ModelNumber_String, m_serial.c_str());
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_SerialNumber_String, m_serial.c_str());
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_RenderModelName_String, renderModelName.c_str());
			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_WillDriftInYaw_Bool, false);
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_ManufacturerName_String, "VirtualMotionTracker");
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_TrackingFirmwareVersion_String, Version.c_str());
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_HardwareRevision_String, Version.c_str());

			VRProperties()->SetStringProperty(m_propertyContainer, Prop_ConnectedWirelessDongle_String, Version.c_str());
			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_DeviceIsWireless_Bool, true);
			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_DeviceIsCharging_Bool, false);
			VRProperties()->SetFloatProperty(m_propertyContainer, Prop_DeviceBatteryPercentage_Float, 1.0f);

			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_Firmware_UpdateAvailable_Bool, false);
			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_Firmware_ManualUpdate_Bool, true);
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_Firmware_ManualUpdateURL_String, "https://github.com/gpsnmeajp/VirtualMotionTracker");
			VRProperties()->SetUint64Property(m_propertyContainer, Prop_HardwareRevision_Uint64, 0);
			VRProperties()->SetUint64Property(m_propertyContainer, Prop_FirmwareVersion_Uint64, 0);
			VRProperties()->SetUint64Property(m_propertyContainer, Prop_FPGAVersion_Uint64, 0);
			VRProperties()->SetUint64Property(m_propertyContainer, Prop_VRCVersion_Uint64, 0);
			VRProperties()->SetUint64Property(m_propertyContainer, Prop_RadioVersion_Uint64, 0);
			VRProperties()->SetUint64Property(m_propertyContainer, Prop_DongleVersion_Uint64, 0);



			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_DeviceProvidesBatteryStatus_Bool, true);
			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_DeviceCanPowerOff_Bool, true);
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_Firmware_ProgrammingTarget_String, Version.c_str());



			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_Firmware_ForceUpdateRequired_Bool, false);

			VRProperties()->SetUint64Property(m_propertyContainer, Prop_ParentDriver_Uint64, 0);

			VRProperties()->SetStringProperty(m_propertyContainer, Prop_ResourceRoot_String, driverName.c_str());

			std::string registeredDeviceType = "";
			registeredDeviceType += driverName.c_str();
			registeredDeviceType += "/";
			registeredDeviceType += m_serial.c_str(); 
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_RegisteredDeviceType_String, registeredDeviceType.c_str());


			std::string inputProfilePath = "";
			inputProfilePath += ressourcePath.c_str();
			inputProfilePath += "/input/";
			inputProfilePath += driverName.c_str();
			inputProfilePath += "_profile.json";
			VRProperties()->SetStringProperty(m_propertyContainer, Prop_InputProfilePath_String, inputProfilePath.c_str()); //vmt_profile.jsonに影響する
			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_NeverTracked_Bool, false);


			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_Identifiable_Bool, true);
			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_CanWirelessIdentify_Bool, true);

			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_Firmware_RemindUpdate_Bool, false);

			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_ReportsTimeSinceVSync_Bool, false);

			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_DisplaySupportsRuntimeFramerateChange_Bool, false);
			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_DisplaySupportsAnalogGain_Bool, false);
			VRProperties()->SetFloatProperty(m_propertyContainer, Prop_DisplayMinAnalogGain_Float, 1.0f);
			VRProperties()->SetFloatProperty(m_propertyContainer, Prop_DisplayMaxAnalogGain_Float, 1.0f);

			std::string iconRessourcePath = "";
			iconRessourcePath += ressourcePath.c_str();
			iconRessourcePath += "/icons/";
			iconRessourcePath += Config::GetInstance()->GetRessourceIconPrefix(m_trackerType);
			
			std::string iconRessourceName = "";
			iconRessourceName += iconRessourcePath.c_str();
			iconRessourceName += "_Off32x32.png";

			VRProperties()->SetStringProperty(m_propertyContainer, Prop_NamedIconPathDeviceOff_String, iconRessourceName.c_str());

			iconRessourceName = "";
			iconRessourceName += iconRessourcePath.c_str();
			iconRessourceName += "_Searching32x32.png";

			VRProperties()->SetStringProperty(m_propertyContainer, Prop_NamedIconPathDeviceSearching_String, iconRessourceName.c_str());

			iconRessourceName = "";
			iconRessourceName += iconRessourcePath.c_str();
			iconRessourceName += "_SearchingAlert32x32.png";

			VRProperties()->SetStringProperty(m_propertyContainer, Prop_NamedIconPathDeviceSearchingAlert_String, iconRessourceName.c_str());

			iconRessourceName = "";
			iconRessourceName += iconRessourcePath.c_str();
			iconRessourceName += "_Ready32x32.png";

			VRProperties()->SetStringProperty(m_propertyContainer, Prop_NamedIconPathDeviceReady_String, iconRessourceName.c_str());

			iconRessourceName = "";
			iconRessourceName += iconRessourcePath.c_str();
			iconRessourceName += "_ReadyAlert32x32.png";

			VRProperties()->SetStringProperty(m_propertyContainer, Prop_NamedIconPathDeviceReadyAlert_String, iconRessourceName.c_str());

			iconRessourceName = "";
			iconRessourceName += iconRessourcePath.c_str();
			iconRessourceName += "_NotReady32x32.png";

			VRProperties()->SetStringProperty(m_propertyContainer, Prop_NamedIconPathDeviceNotReady_String, iconRessourceName.c_str());

			iconRessourceName = "";
			iconRessourceName += iconRessourcePath.c_str();
			iconRessourceName += "_Standby32x32.png";

			VRProperties()->SetStringProperty(m_propertyContainer, Prop_NamedIconPathDeviceStandby_String, iconRessourceName.c_str());

			iconRessourceName = "";
			iconRessourceName += iconRessourcePath.c_str();
			iconRessourceName += "_StandbyAlert32x32.png";

			VRProperties()->SetStringProperty(m_propertyContainer, Prop_NamedIconPathDeviceStandbyAlert_String, iconRessourceName.c_str());
			
			iconRessourceName = "";
			iconRessourceName += iconRessourcePath.c_str();
			iconRessourceName += "_AlertLow32x32.png";

			VRProperties()->SetStringProperty(m_propertyContainer, Prop_NamedIconPathDeviceAlertLow_String, iconRessourceName.c_str());

			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_HasDisplayComponent_Bool, false);
			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_HasCameraComponent_Bool, false);
			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_HasDriverDirectModeComponent_Bool, false);
			VRProperties()->SetBoolProperty(m_propertyContainer, Prop_HasVirtualDisplayComponent_Bool, false);

			//VRProperties()->SetStringProperty(m_propertyContainer, vmt_profile.json, "NO_SETTING"); //設定不可
			VRProperties()->SetInt32Property(m_propertyContainer, Prop_ControllerHandSelectionPriority_Int32, 0);


			//OpenVR デバイス入力情報の定義
			VRDriverInput()->CreateBooleanComponent(m_propertyContainer, "/input/Button0/click", &ButtonComponent[0]);
			VRDriverInput()->CreateBooleanComponent(m_propertyContainer, "/input/Button1/click", &ButtonComponent[1]);
			VRDriverInput()->CreateBooleanComponent(m_propertyContainer, "/input/Button2/click", &ButtonComponent[2]);
			VRDriverInput()->CreateBooleanComponent(m_propertyContainer, "/input/Button3/click", &ButtonComponent[3]);
			VRDriverInput()->CreateBooleanComponent(m_propertyContainer, "/input/Button4/click", &ButtonComponent[4]);
			VRDriverInput()->CreateBooleanComponent(m_propertyContainer, "/input/Button5/click", &ButtonComponent[5]);
			VRDriverInput()->CreateBooleanComponent(m_propertyContainer, "/input/Button6/click", &ButtonComponent[6]);
			VRDriverInput()->CreateBooleanComponent(m_propertyContainer, "/input/Button7/click", &ButtonComponent[7]);

			VRDriverInput()->CreateScalarComponent(m_propertyContainer, "/input/Trigger0/value", &TriggerComponent[0], EVRScalarType::VRScalarType_Absolute, EVRScalarUnits::VRScalarUnits_NormalizedOneSided);
			VRDriverInput()->CreateScalarComponent(m_propertyContainer, "/input/Trigger1/value", &TriggerComponent[1], EVRScalarType::VRScalarType_Absolute, EVRScalarUnits::VRScalarUnits_NormalizedOneSided);

			VRDriverInput()->CreateScalarComponent(m_propertyContainer, "/input/Joystick0/x", &JoystickComponent[0], EVRScalarType::VRScalarType_Absolute, EVRScalarUnits::VRScalarUnits_NormalizedTwoSided);
			VRDriverInput()->CreateScalarComponent(m_propertyContainer, "/input/Joystick0/y", &JoystickComponent[1], EVRScalarType::VRScalarType_Absolute, EVRScalarUnits::VRScalarUnits_NormalizedTwoSided);

			VRDriverInput()->CreateHapticComponent(m_propertyContainer, "/output/haptic", &HapticComponent);
			break;
		}
		}

		m_alreadyRegistered = true;
		m_registrationInProgress = false;
        return EVRInitError::VRInitError_None;
    }

    //OpenVRからのデバイス無効化コール
    void TrackedDeviceServerDriver::Deactivate()
    {
        m_deviceIndex = k_unTrackedDeviceIndexInvalid;
        m_propertyContainer = k_ulInvalidPropertyContainer;
    }

    //OpenVRからのデバイス電源オフコール
    void TrackedDeviceServerDriver::EnterStandby()
    {
        //電源オフ要求が来た
        Reset();
    }

    //OpenVRからのデバイス固有機能の取得(ない場合はnullptrを返す)
    void* TrackedDeviceServerDriver::GetComponent(const char* pchComponentNameAndVersion)
    {
        return nullptr;
    }

    //OpenVRからのデバイスのデバッグリクエスト
    void TrackedDeviceServerDriver::DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize)
    {
        //デバッグ用
        //Log::printf("DebugRequest: %s", pchRequest);
        if (unResponseBufferSize > 0) {
            pchResponseBuffer[0] = '\0';
        }
    }

    //OpenVRからのデバイス姿勢取得
    DriverPose_t TrackedDeviceServerDriver::GetPose()
    {
        //現在のOpenVR向け姿勢を返却する
        return m_pose;
    }
}