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
#include "CommunicationManager.h"
#include "osc/OscOutboundPacketStream.h"
#include "osc/OscReceivedElements.h"
#include "osc/OscPacketListener.h"
#include "ip/UdpSocket.h"
#include "DirectOSC.h"

#define MAX_SEQ_FAILURE 250

namespace VMTDriver {
	//別スレッドからのコール

	//姿勢を各仮想デバイスに設定する
	void OSCReceiver::SetPose(bool roomToDriver,
		int seqNum,
		int idx, int enable,
		double x, double y, double z,
		double qx, double qy, double qz, double qw,
		double vpx, double vpy, double vpz,
		double vax, double vay, double vaz,
		double timeoffset,
		const char* root_sn, ReferMode_t mode)
	{
		RawPose pose{};
		pose.roomToDriver = roomToDriver;
		pose.idx = idx;
		pose.enable = enable;
		pose.x = x;
		pose.y = y;
		pose.z = z;
		pose.qx = qx;
		pose.qy = qy;
		pose.qz = qz;
		pose.qw = qw;

		pose.vpx = vpx;
		pose.vpy = vpy;
		pose.vpz = vpz;
		pose.vax = vax;
		pose.vay = vay;
		pose.vaz = vaz;

		pose.timeoffset = timeoffset;
		pose.mode = mode;
		pose.root_sn = root_sn ? root_sn : "";
		pose.time = std::chrono::system_clock::now();

		//範囲チェック
		if (GetServer()->IsVMTDeviceIndex(idx))
		{
			if (GetServer()->GetDevice(idx).m_lastSeqNumFailure > MAX_SEQ_FAILURE || GetServer()->GetDevice(idx).m_lastSeqNum < seqNum) {
				GetServer()->GetDevice(idx).m_lastSeqNumFailure = 0;
				GetServer()->GetDevice(idx).m_lastSeqNum = seqNum;

				GetServer()->GetDevice(idx).RegisterToVRSystem((eTrackerType)enable);
				GetServer()->GetDevice(idx).SetRawPose(pose);
				GetServer()->GetDevice(idx).UpdatePoseToVRSystem();
			}
			else if (GetServer()->GetDevice(idx).m_lastSeqNum != seqNum) {
				GetServer()->GetDevice(idx).m_lastSeqNumFailure++;
			}
		}
	}

	//姿勢を各仮想デバイスに設定する
	void OSCReceiver::SetHmdPose(bool roomToDriver,
		int seqNum,
		double x, double y, double z,
		double qx, double qy, double qz, double qw,
		double vpx, double vpy, double vpz,
		double vax, double vay, double vaz,
		double timeoffset,
		const char* root_sn, ReferMode_t mode)
	{
		RawHmdPose pose{};
		pose.roomToDriver = roomToDriver;
		pose.idx = -1;
		pose.enable = 1;
		pose.x = x;
		pose.y = y;
		pose.z = z;
		pose.qx = qx;
		pose.qy = qy;
		pose.qz = qz;
		pose.qw = qw;

		pose.vpx = vpx;
		pose.vpy = vpy;
		pose.vpz = vpz;
		pose.vax = vax;
		pose.vay = vay;
		pose.vaz = vaz;

		pose.timeoffset = timeoffset;
		pose.mode = mode;
		pose.root_sn = root_sn ? root_sn : "";
		pose.time = std::chrono::system_clock::now();

		//範囲チェック
		if (GetServer()->GetHmdDevice().m_lastSeqNumFailure > MAX_SEQ_FAILURE || GetServer()->GetHmdDevice().m_lastSeqNum < seqNum) {
			GetServer()->GetHmdDevice().m_lastSeqNumFailure = 0;
			GetServer()->GetHmdDevice().m_lastSeqNum = seqNum;

			GetServer()->GetHmdDevice().RegisterToVRSystem();
			GetServer()->GetHmdDevice().SetRawPose(pose);
			GetServer()->GetHmdDevice().UpdatePoseToVRSystem();
		}
		else if (GetServer()->GetHmdDevice().m_lastSeqNum != seqNum) {
			GetServer()->GetHmdDevice().m_lastSeqNumFailure++;
		}
	}

	void OSCReceiver::SetupHmdDisplaySettings(
		int display_x, int display_y, int display_w, int display_h,
		int render_w, int render_h,
		int frameRate)
	{
		DisplaySettings displaySettings{};

		displaySettings.display_x = display_x;
		displaySettings.display_y = display_y;
		displaySettings.display_w = display_w;
		displaySettings.display_h = display_h;

		displaySettings.render_w = render_w;
		displaySettings.render_h = render_h;

		displaySettings.frameRate = frameRate;

		displaySettings.directMode = false;
		displaySettings.vendorId = 0;
		displaySettings.productId = 0;

		GetServer()->GetHmdDevice().SetupDisplaySettings(displaySettings);
	}

	void OSCReceiver::SetupHmdDisplayDirectSettings(
		int display_x, int display_y, int display_w, int display_h,
		int render_w, int render_h,
		int frameRate,
		int vendorId, int productId)
	{
		DisplaySettings displaySettings{};

		displaySettings.display_x = display_x;
		displaySettings.display_y = display_y;
		displaySettings.display_w = display_w;
		displaySettings.display_h = display_h;

		displaySettings.render_w = render_w;
		displaySettings.render_h = render_h;

		displaySettings.frameRate = frameRate;

		displaySettings.directMode = true;
		displaySettings.vendorId = vendorId;
		displaySettings.productId = productId;

		GetServer()->GetHmdDevice().SetupDisplaySettings(displaySettings);
	}

	void OSCReceiver::SetupHmdRenderSettings(
		float distortionK0, float distortionK1, float distortionScale,
		float distortionBlueOffset, float distortionGreenOffset, float distortionRedOffset,
		float hFov, float vFov)
	{
		RenderSettings renderSettings{};
		renderSettings.distortionK0 = distortionK0;
		renderSettings.distortionK1 = distortionK1;
		renderSettings.distortionScale = distortionScale;
		renderSettings.distortionBlueOffset = distortionBlueOffset;
		renderSettings.distortionGreenOffset = distortionGreenOffset;
		renderSettings.distortionRedOffset = distortionRedOffset;
		
		// FOV to rad
		renderSettings.hFov = hFov * (3.14159265358979323846f / 180.f);
		renderSettings.vFov = vFov * (3.14159265358979323846f / 180.f);

		GetServer()->GetHmdDevice().SetupRenderSettings(renderSettings);
	}

	//ログ情報を送信する(ダイアログを表示する)
	void OSCReceiver::SendLog(int stat, string msg) {
		const size_t bufsize = 8192;
		char buf[bufsize]{};
		osc::OutboundPacketStream packet(buf, bufsize);
		packet << osc::BeginMessage("/VMT/Out/Log")
			<< stat
			<< msg.c_str()
			<< osc::EndMessage;
		DirectOSC::OSC::GetInstance()->GetSocketTx()->Send(packet.Data(), packet.Size());
	}

	//生存信号を送信する
	void OSCReceiver::SendAlive() {
		auto now = std::chrono::system_clock::now();
		auto duration_since_epoch = now.time_since_epoch();
		auto total_milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration_since_epoch).count();

		const size_t bufsize = 8192;
		char buf[bufsize]{};
		osc::OutboundPacketStream packet(buf, bufsize);
		packet << osc::BeginMessage("/VMT/Out/Alive")
			<< Version.c_str()
			<< GetServer()->GetInstallPath().c_str()
			<< total_milliseconds
			<< osc::EndMessage;
		DirectOSC::OSC::GetInstance()->GetSocketTx()->Send(packet.Data(), packet.Size());
	}

	//振動情報を送信する
	void OSCReceiver::SendHaptic(int index, float frequency, float amplitude, float duration)
	{
		const size_t bufsize = 8192;
		char buf[bufsize]{};
		osc::OutboundPacketStream packet(buf, bufsize);
		packet << osc::BeginMessage("/VMT/Out/Haptic")
			<< index
			<< frequency
			<< amplitude
			<< duration
			<< osc::EndMessage;
		DirectOSC::OSC::GetInstance()->GetSocketTx()->Send(packet.Data(), packet.Size());
	}

	//エラー情報を送信する
	void OSCReceiver::SendUnavailable(int code, std::string reason) {
		const size_t bufsize = 8192;
		char buf[bufsize]{};
		osc::OutboundPacketStream packet(buf, bufsize);
		packet << osc::BeginMessage("/VMT/Out/Unavailable")
			<< code
			<< reason.c_str()
			<< osc::EndMessage;
		DirectOSC::OSC::GetInstance()->GetSocketTx()->Send(packet.Data(), packet.Size());
	}

	//エラー情報を送信する
	void OSCReceiver::SendHmdInfo(float framerate, int screenWidth, int screenHeight, bool directMode) {
		const size_t bufsize = 8192;
		char buf[bufsize]{};
		osc::OutboundPacketStream packet(buf, bufsize);
		packet << osc::BeginMessage("/VMT/Out/HmdInfo")
			<< framerate
			<< screenWidth
			<< screenHeight
			<< directMode
			<< osc::EndMessage;
		DirectOSC::OSC::GetInstance()->GetSocketTx()->Send(packet.Data(), packet.Size());
	}

	//デバイス姿勢とシリアル番号を送信する
	void OSCReceiver::SendDevicePose(const Eigen::Affine3d& pose, const char* serialNumber)
	{
		const size_t bufsize = 8192;
		char buf[bufsize]{};
		osc::OutboundPacketStream packet(buf, bufsize);

		const Eigen::Translation3d pos(pose.translation());
		const Eigen::Quaterniond rot(pose.rotation());

		packet << osc::BeginMessage("/VMT/Out/DevicePose")
			<< serialNumber
			<< (float)pos.x()
			<< (float)pos.y()
			<< (float)pos.z()
			<< (float)rot.x()
			<< (float)rot.y()
			<< (float)rot.z()
			<< (float)rot.w()
			<< osc::EndMessage;
		DirectOSC::OSC::GetInstance()->GetSocketTx()->Send(packet.Data(), packet.Size());
	}

	void OSCReceiver::SendDevices(std::string msg)
	{
		const size_t bufsize = 8192;
		char buf[bufsize]{};
		osc::OutboundPacketStream packet(buf, bufsize);

		packet << osc::BeginMessage("/VMT/Out/DevicesList")
			<< msg.c_str()
			<< osc::EndMessage;
		DirectOSC::OSC::GetInstance()->GetSocketTx()->Send(packet.Data(), packet.Size());
	}

	//受信処理
	void OSCReceiver::ProcessMessage(const osc::ReceivedMessage& m, const IpEndpointName& remoteEndpoint)
	{
		int seqNum{};
		int idx{};
		int enable{};
		double timeoffset{};
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
		int ButtonIndex{};
		int ButtonValue{};
		float TriggerValue{};
		const char* root_sn = nullptr;
		const char* serialNumber = nullptr;
		float batteryValue;

		float input_x{};
		float input_y{};

		int display_x{};
		int display_y{};
		int display_w{};
		int display_h{};
		int render_w{};
		int render_h{};

		float distortionK0{};
		float distortionK1{};
		float distortionScale{};
		float distortionBlueOffset{};
		float distortionGreenOffset{};
		float distortionRedOffset{};
		float hFov{};
		float vFov{};

		int frameRate{};

		int vendorId{};
		int productId{};

		float fIpdMeters{};
		 
		float m1{};
		float m2{};
		float m3{};
		float m4{};
		float m5{};
		float m6{};
		float m7{};
		float m8{};
		float m9{};
		float m10{};
		float m11{};
		float m12{};

		const char* restartMessage = nullptr;

		try {
			string adr = m.AddressPattern();
			osc::ReceivedMessageArgumentStream args = m.ArgumentStream();

			//姿勢情報の受信
			if (adr == "/VMT/Room/Unity")
			{
				args >> seqNum >> idx >> enable >> timeoffset >> x >> y >> z >> qx >> qy >> qz >> qw >> vpx >> vpy >> vpz >> vax >> vay >> vaz >> osc::EndMessage;
				SetPose(true, seqNum, idx, enable, x, y, -z, qx, qy, -qz, -qw, vpx, vpy, -vpz, vax, vay, vaz, timeoffset);
			}
			else if (adr == "/VMT/Room/Driver")
			{
				args >> seqNum >> idx >> enable >> timeoffset >> x >> y >> z >> qx >> qy >> qz >> qw >> vpx >> vpy >> vpz >> vax >> vay >> vaz >> osc::EndMessage;
				SetPose(true, seqNum, idx, enable, x, y, z, qx, qy, qz, qw, vpx, vpy, vpz, vax, vay, vaz, timeoffset);
			}
			else if (adr == "/VMT/Raw/Unity")
			{
				args >> seqNum >> idx >> enable >> timeoffset >> x >> y >> z >> qx >> qy >> qz >> qw >> vpx >> vpy >> vpz >> vax >> vay >> vaz >> osc::EndMessage;
				SetPose(false, seqNum, idx, enable, x, y, -z, qx, qy, -qz, -qw, vpx, vpy, -vpz, vax, vay, vaz, timeoffset);
			}
			else if (adr == "/VMT/Raw/Driver")
			{
				args >> seqNum >> idx >> enable >> timeoffset >> x >> y >> z >> qx >> qy >> qz >> qw >> vpx >> vpy >> vpz >> vax >> vay >> vaz >> osc::EndMessage;
				SetPose(false, seqNum, idx, enable, x, y, z, qx, qy, qz, qw, vpx, vpy, vpz, vax, vay, vaz, timeoffset);
			}
			else if (adr == "/VMT/Joint/Unity")
			{
				args >> seqNum >> idx >> enable >> timeoffset >> x >> y >> z >> qx >> qy >> qz >> qw >> vpx >> vpy >> vpz >> vax >> vay >> vaz >> root_sn >> osc::EndMessage;
				SetPose(false, seqNum, idx, enable, x, y, -z, qx, qy, -qz, -qw, vpx, vpy, -vpz, vax, vay, vaz, timeoffset, root_sn, ReferMode_t::Joint);
			}
			else if (adr == "/VMT/Joint/Driver")
			{
				args >> seqNum >> idx >> enable >> timeoffset >> x >> y >> z >> qx >> qy >> qz >> qw >> vpx >> vpy >> vpz >> vax >> vay >> vaz >> root_sn >> osc::EndMessage;
				SetPose(false, seqNum, idx, enable, x, y, z, qx, qy, qz, qw, vpx, vpy, vpz, vax, vay, vaz, timeoffset, root_sn, ReferMode_t::Joint);
			}
			else if (adr == "/VMT/Follow/Unity")
			{
				args >> seqNum >> idx >> enable >> timeoffset >> x >> y >> z >> qx >> qy >> qz >> qw >> vpx >> vpy >> vpz >> vax >> vay >> vaz >> root_sn >> osc::EndMessage;
				SetPose(false, seqNum, idx, enable, x, y, -z, qx, qy, -qz, -qw, vpx, vpy, -vpz, vax, vay, vaz, timeoffset, root_sn, ReferMode_t::Follow);
			}
			else if (adr == "/VMT/Follow/Driver")
			{
				args >> seqNum >> idx >> enable >> timeoffset >> x >> y >> z >> qx >> qy >> qz >> qw >> vpx >> vpy >> vpz >> vax >> vay >> vaz >> root_sn >> osc::EndMessage;
				SetPose(false, seqNum, idx, enable, x, y, z, qx, qy, qz, qw, vpx, vpy, vpz, vax, vay, vaz, timeoffset, root_sn, ReferMode_t::Follow);
			}
			
			// HMD
			else if (adr == "/VMT/HMD/Room/Unity")
			{
				args >> seqNum >> timeoffset >> x >> y >> z >> qx >> qy >> qz >> qw >> vpx >> vpy >> vpz >> vax >> vay >> vaz >> osc::EndMessage;
				SetHmdPose(true, seqNum, x, y, -z, qx, qy, -qz, -qw, vpx, vpy, -vpz, vax, vay, vaz, timeoffset);
			}
			else if (adr == "/VMT/HMD/Room/Driver")
			{
				args >> seqNum >> timeoffset >> x >> y >> z >> qx >> qy >> qz >> qw >> vpx >> vpy >> vpz >> vax >> vay >> vaz >> osc::EndMessage;
				SetHmdPose(true, seqNum, x, y, z, qx, qy, qz, qw, vpx, vpy, vpz, vax, vay, vaz, timeoffset);
			}
			else if (adr == "/VMT/HMD/Raw/Unity")
			{
				args >> seqNum >> timeoffset >> x >> y >> z >> qx >> qy >> qz >> qw >> vpx >> vpy >> vpz >> vax >> vay >> vaz >> osc::EndMessage;
				SetHmdPose(false, seqNum, x, y, -z, qx, qy, -qz, -qw, vpx, vpy, -vpz, vax, vay, vaz, timeoffset);
			}
			else if (adr == "/VMT/HMD/Raw/Driver")
			{
				args >> seqNum >> timeoffset >> x >> y >> z >> qx >> qy >> qz >> qw >> vpx >> vpy >> vpz >> vax >> vay >> vaz >> osc::EndMessage;
				SetHmdPose(false, seqNum, x, y, z, qx, qy, qz, qw, vpx, vpy, vpz, vax, vay, vaz, timeoffset);
			}
			else if (adr == "/VMT/HMD/Joint/Unity")
			{
				args >> seqNum >> timeoffset >> x >> y >> z >> qx >> qy >> qz >> qw >> vpx >> vpy >> vpz >> vax >> vay >> vaz >> root_sn >> osc::EndMessage;
				SetHmdPose(false, seqNum, x, y, -z, qx, qy, -qz, -qw, vpx, vpy, -vpz, vax, vay, vaz, timeoffset, root_sn, ReferMode_t::Joint);
			}
			else if (adr == "/VMT/HMD/Joint/Driver")
			{
				args >> seqNum >> timeoffset >> x >> y >> z >> qx >> qy >> qz >> qw >> vpx >> vpy >> vpz >> vax >> vay >> vaz >> root_sn >> osc::EndMessage;
				SetHmdPose(false, seqNum, x, y, z, qx, qy, qz, qw, vpx, vpy, vpz, vax, vay, vaz, timeoffset, root_sn, ReferMode_t::Joint);
			}
			else if (adr == "/VMT/HMD/Follow/Unity")
			{
				args >> seqNum >> timeoffset >> x >> y >> z >> qx >> qy >> qz >> qw >> vpx >> vpy >> vpz >> vax >> vay >> vaz >> root_sn >> osc::EndMessage;
				SetHmdPose(false, seqNum, x, y, -z, qx, qy, -qz, -qw, vpx, vpy, -vpz, vax, vay, vaz, timeoffset, root_sn, ReferMode_t::Follow);
			}
			else if (adr == "/VMT/HMD/Follow/Driver")
			{
				args >> seqNum >> timeoffset >> x >> y >> z >> qx >> qy >> qz >> qw >> vpx >> vpy >> vpz >> vax >> vay >> vaz >> root_sn >> osc::EndMessage;
				SetHmdPose(false, seqNum, x, y, z, qx, qy, qz, qw, vpx, vpy, vpz, vax, vay, vaz, timeoffset, root_sn, ReferMode_t::Follow);
			}

			// HMD Settings
			else if (adr == "/VMT/HMD/SetupDisplay")
			{
				args >> display_x >> display_y >> display_w >> display_h >> render_w >> render_h >> frameRate >> osc::EndMessage;
				SetupHmdDisplaySettings(display_x, display_y, display_w, display_h, render_w, render_h, frameRate);
			}
			else if (adr == "/VMT/HMD/SetupDisplayDirect")
			{
				args >> display_x >> display_y >> display_w >> display_h >> render_w >> render_h >> frameRate >> vendorId >> productId >> osc::EndMessage;
				SetupHmdDisplayDirectSettings(display_x, display_y, display_w, display_h, render_w, render_h, frameRate, vendorId, productId);
			}
			else if (adr == "/VMT/HMD/SetupRender")
			{
				args >> distortionK0 >> distortionK1 >> distortionScale >> distortionRedOffset >> distortionGreenOffset >> distortionBlueOffset >> hFov >> vFov >> osc::EndMessage;
				SetupHmdRenderSettings(distortionK0, distortionK1, distortionScale, distortionRedOffset, distortionGreenOffset, distortionBlueOffset, hFov, vFov);
			}
			else if (adr == "/VMT/HMD/SetIpdMeters")
			{
				args >> fIpdMeters >> osc::EndMessage;
				GetServer()->GetHmdDevice().SetIpdMeters(fIpdMeters);
			}

			//デバイス入力系の受信
			else if (adr == "/VMT/Input/Button")
			{
				args >> idx >> ButtonIndex >> timeoffset >> ButtonValue >> osc::EndMessage;
				if (GetServer()->IsVMTDeviceIndex(idx))
				{
					GetServer()->GetDevice(idx).UpdateButtonInput(ButtonIndex, ButtonValue != 0, timeoffset);
				}
			}
			else if (adr == "/VMT/Input/Trigger")
			{
				args >> idx >> ButtonIndex >> timeoffset >> TriggerValue >> osc::EndMessage;
				if (GetServer()->IsVMTDeviceIndex(idx))
				{
					GetServer()->GetDevice(idx).UpdateTriggerInput(ButtonIndex, TriggerValue, timeoffset);
				}
			}
			else if (adr == "/VMT/Input/Joystick")
			{
				args >> idx >> ButtonIndex >> timeoffset >> input_x >> input_y >> osc::EndMessage;
				if (GetServer()->IsVMTDeviceIndex(idx))
				{
					GetServer()->GetDevice(idx).UpdateJoystickInput(ButtonIndex, input_x, input_y, timeoffset);
				}
			}
			else if (adr == "/VMT/Property/Battery")
			{
				args >> idx >> batteryValue >> osc::EndMessage;
				if (GetServer()->IsVMTDeviceIndex(idx))
				{
					GetServer()->GetDevice(idx).UpdateBatteryProperty(batteryValue);
				}
			}
			//すべてのデバイスのリセット
			else if (adr == "/VMT/Reset")
			{
				GetServer()->DeviceResetAll();
			}
			//設定の読み込み
			else if (adr == "/VMT/LoadSetting")
			{
				Config::GetInstance()->LoadSetting();
				SendLog(0, "Setting Loaded");
			}
			//ルーム変換行列の設定
			else if (adr == "/VMT/SetRoomMatrix")
			{
				args >> m1 >> m2 >> m3 >> m4 >> m5 >> m6 >> m7 >> m8 >> m9 >> m10 >> m11 >> m12 >> osc::EndMessage;
				Config::GetInstance()->SetRoomMatrix(true, m1, m2, m3, m4, m5, m6, m7, m8, m9, m10, m11, m12);
				SendLog(0, "Set Room Matrix Done.");
			}
			else if (adr == "/VMT/SetRoomMatrix/Temporary")
			{
				args >> m1 >> m2 >> m3 >> m4 >> m5 >> m6 >> m7 >> m8 >> m9 >> m10 >> m11 >> m12 >> osc::EndMessage;
				Config::GetInstance()->SetRoomMatrix(false, m1, m2, m3, m4, m5, m6, m7, m8, m9, m10, m11, m12);
				SendLog(0, "Set Room Matrix Done.(Temporary)");
			}
			//デバイス一覧の取得
			else if (adr == "/VMT/GetDevicesList")
			{
				std::string result = GetServer()->GetOpenVRDevicesString();
				OSCReceiver::SendDevices(result);
			}
			//デバイスの姿勢を取得して返信
			else if (adr == "/VMT/GetDevicePose")
			{
				args >> serialNumber >> osc::EndMessage;
				Eigen::Affine3d pose;
				if (TrackedDeviceServerDriver::GetDevicePose(pose, serialNumber))
				{
					OSCReceiver::SendDevicePose(pose, serialNumber);
				}
			}
			//すべてのデバイスのリセット
			else if (adr == "/VMT/RequestRestart")
			{
				args >> restartMessage >> osc::EndMessage;
				GetServer()->RequestRestart(restartMessage);
			}

			//不明なパケット
			else {
				Log::printf("Unkown: %s", adr.c_str());
			}
		}
		catch (osc::Exception& e)
		{
			Log::printf("Exp: %s\n", e.what());
		}
	}
}
namespace VMTDriver {
	//通信クラスのコンストラクタ
	CommunicationManager::CommunicationManager()
	{
	}

	//通信クラスのシングルトンインスタンスの取得
	CommunicationManager* CommunicationManager::GetInstance()
	{
		static CommunicationManager cm;
		return &cm;
	}

	//通信のオープン
	void CommunicationManager::Open()
	{
		//すでにオープン済みなら何もしない
		if (m_opened) {
			return;
		}

		Config* config = Config::GetInstance();

		//通信ポートオープン
		DirectOSC::OSC::GetInstance()->Open(&m_rcv, config->GetReceivePort(), config->GetSendIp(), config->GetSendPort());
		m_opened = true;
	}
	//通信のクローズ
	void CommunicationManager::Close()
	{
		DirectOSC::OSC::GetInstance()->Close();
		m_opened = false;
	}
	//フレーム単位の処理
	void CommunicationManager::Process()
	{
		if (m_frame > frameCycle) {
			//定期的に生存信号を送信
			OSCReceiver::SendAlive();

			//エラー状態を送信
			if (!Config::GetInstance()->GetRoomMatrixStatus()) {
				OSCReceiver::SendUnavailable(1, "Room Matrix has not been set.");
			}
			else {
				OSCReceiver::SendUnavailable(0, "OK");
			}
			m_frame = 0;
		}
		m_frame++;
	}
}