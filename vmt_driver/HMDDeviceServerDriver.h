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
	struct DisplaySettings {
		int display_x{};
		int display_y{};
		int display_w{};
		int display_h{};
		int render_w{};
		int render_h{};

		int frameRate{};

		bool directMode{};
		int vendorId{};
		int productId{};
	};

	struct RenderSettings {
		float distortionK0{};
		float distortionK1{};
		float distortionScale{};
		float distortionBlueOffset{};
		float distortionGreenOffset{};
		float distortionRedOffset{};
		float hFov{};
		float vFov{};
	};

	struct RawHmdPose {
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
		double timeoffset{};
		ReferMode_t mode{};
		std::string root_sn{};
		std::chrono::system_clock::time_point time{};
	};

	//個々のデバイス
	class HMDDeviceServerDriver : public ITrackedDeviceServerDriver, IVRDisplayComponent
	{
	private:
		bool m_alreadyRegistered = false;
		bool m_registrationInProgress = false;
		string m_serial = "";
		TrackedDeviceIndex_t m_deviceIndex{ 0 };
		PropertyContainerHandle_t m_propertyContainer{ 0 };
		uint32_t m_index = k_unTrackedDeviceIndexInvalid;
		bool m_enableVelocity = false;

		DriverPose_t m_pose{ 0 };
		RawHmdPose m_rawPose{ 0 };
		RawHmdPose m_lastRawPose{ 0 };
		double m_lastVecVeloctiy[3]{ 0 };
		double m_lastAngVeloctiy[3]{ 0 };

		DisplaySettings m_displaySettings{ 0 };
		RenderSettings m_renderSettings{ 0 };
		bool m_displayValid = false;
		bool m_renderValid = false;
		float m_userIpdMeters{};

		bool m_poweron = false;

		const int frameCycle = 120;
		int m_frame = 0;
	public:
		//内部向け
		HMDDeviceServerDriver();
		~HMDDeviceServerDriver();

		void SetDeviceSerial(string);
		void SetObjectIndex(uint32_t);
		void SetPose(DriverPose_t pose);
		void SetRawPose(RawHmdPose rawPose);
		void SetupDisplaySettings(DisplaySettings displaySettings);
		void SetupRenderSettings(RenderSettings displaySettings);
		void SetIpdMeters(float userIpdMeters);
		void SetVelocity(bool enable);
		DriverPose_t RawPoseToPose();
		void CalcVelocity(DriverPose_t & pose);
		void CompensateVelocity(DriverPose_t & pose);
		Eigen::Quaterniond QuaternionFromAngularVelocity(const Eigen::Vector3d & angularVelocity, double deltaTime);
		void RegisterToVRSystem();
		void UpdatePoseToVRSystem();
		void Reset();

		Eigen::Vector3d AngularVelocityBetweenQuats(const Eigen::Quaterniond & q1, const Eigen::Quaterniond & q2, double dt);
		void CalcJoint(DriverPose_t& pose, string serial, ReferMode_t mode, Eigen::Affine3d& RoomToDriverAffin);
		static int SearchDevice(vr::TrackedDevicePose_t* poses, string serial);
		void RejectTracking(DriverPose_t& pose);
		void ProcessEvent(VREvent_t &VREvent);

		static bool GetDevicePose(Eigen::Affine3d& out_pose, const char* in_serialNumber);

		//OpenVR向け
		virtual EVRInitError Activate(uint32_t unObjectId) override;
		virtual void Deactivate() override;
		virtual void EnterStandby() override;
		virtual void* GetComponent(const char* pchComponentNameAndVersion) override;
		virtual void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) override;
		virtual DriverPose_t GetPose() override;

		// IVRDisplayComponent
		virtual void GetWindowBounds(int32_t* pnX, int32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight) override;
		virtual bool IsDisplayOnDesktop() override;
		virtual bool IsDisplayRealDisplay() override;
		virtual void GetRecommendedRenderTargetSize(uint32_t* pnWidth, uint32_t* pnHeight) override;
		virtual void GetEyeOutputViewport(EVREye eEye, uint32_t* pnX, uint32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight) override;
		virtual void GetProjectionRaw(EVREye eEye, float* pfLeft, float* pfRight, float* pfTop, float* pfBottom) override;
		virtual DistortionCoordinates_t ComputeDistortion(EVREye eEye, float fU, float fV) override;

		int m_lastSeqNum{ 0 };
		int m_lastSeqNumFailure{ 0 };
	};
}