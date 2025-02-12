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
#pragma once
#include "dllmain.h"

//OpenVRデバイス
//サーバーにぶら下がる子である
namespace VMTDriver {
    const HmdQuaternion_t HmdQuaternion_Identity{ 1,0,0,0 };

    struct RawPose {
        bool roomToDriver{};
        int idx{};
        int enable{};

        double x{};
        double y{};
        double z{};
        double qx{};
        double qy{};
        double qz{};
        double qw{};

		double vpx{};
		double vpy{};
		double vpz{};
		double vax{};
		double vay{};
		double vaz{};

        double timeoffset{};
        ReferMode_t mode{};
        std::string root_sn{};
		std::chrono::system_clock::time_point time{};
    };

    //個々のデバイス
    class TrackedDeviceServerDriver : public ITrackedDeviceServerDriver
    {
	private:
		eTrackerType m_trackerType = eTrackerType::TrackerType_Invalid;
		bool m_alreadyRegistered = false;
		bool m_registrationInProgress = false;
		string m_serial = "";
		TrackedDeviceIndex_t m_deviceIndex{ 0 };
		PropertyContainerHandle_t m_propertyContainer{ 0 };
		uint32_t m_index = k_unTrackedDeviceIndexInvalid;

		DriverPose_t m_pose{ 0 };
		RawPose m_rawPose{ 0 };
		RawPose m_lastRawPose{ 0 };
		double m_lastVec[3]{ 0 };
		double m_lastAng[4]{ 0 };
		double m_lastVecVeloctiy[3]{ 0 };
		double m_lastAngVeloctiy[3]{ 0 };
		std::chrono::system_clock::time_point m_lastVecTime{};
		std::chrono::system_clock::time_point m_lastAngTime{};
		bool m_lastVecValid = false;
		bool m_lastAngValid = false;

		uint32_t m_registeredButtons = 0;
		VRInputComponentHandle_t ButtonComponent[32]{ 0 };
		VRInputComponentHandle_t TriggerComponent[2]{ 0 };
		VRInputComponentHandle_t JoystickComponent[2]{ 0 };
		VRInputComponentHandle_t HapticComponent{ 0 };

		bool m_poweron = false;
	public:
		//内部向け
		TrackedDeviceServerDriver();
		~TrackedDeviceServerDriver();

		void SetDeviceSerial(string);
		void SetObjectIndex(uint32_t);
		void SetPose(DriverPose_t pose);
		void SetRawPose(RawPose rawPose);
		DriverPose_t RawPoseToPose();
		void RegisterToVRSystem(eTrackerType type);
		void UpdatePoseToVRSystem();
		void UpdateButtonInput(uint32_t index, bool value, double timeoffset);
		void UpdateTriggerInput(uint32_t index, float value, double timeoffset);
		void UpdateJoystickInput(uint32_t index, float x, float y, double timeoffset);
		void UpdateBatteryProperty(float value);
		void Reset();

        void CalcJoint(DriverPose_t& pose, string serial, ReferMode_t mode, Eigen::Affine3d& RoomToDriverAffin);
        static int SearchDevice(vr::TrackedDevicePose_t* poses, string serial);
        void RejectTracking(DriverPose_t& pose);
		void RunFrame();
		void ProcessEvent(VREvent_t &VREvent);

        static bool GetDevicePose(Eigen::Affine3d& out_pose, const char* in_serialNumber);

		//OpenVR向け
		virtual EVRInitError Activate(uint32_t unObjectId) override;
		virtual void Deactivate() override;
		virtual void EnterStandby() override;
		virtual void* GetComponent(const char* pchComponentNameAndVersion) override;
		virtual void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) override;
		virtual DriverPose_t GetPose() override; 

		int m_lastSeqNum{ 0 };
		int m_lastSeqNumFailure{ 0 };
    };
}