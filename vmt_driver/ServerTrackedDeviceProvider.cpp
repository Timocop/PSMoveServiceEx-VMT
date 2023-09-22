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
#include "ServerTrackedDeviceProvider.h"
namespace VMTDriver {
    //** 内部向け関数群 **

	//全仮想デバイスを返す
	vector<TrackedDeviceServerDriver>& ServerTrackedDeviceProvider::GetDevices()
	{
		return m_devices;
	}

	//特定仮想デバイスを返す
	TrackedDeviceServerDriver& ServerTrackedDeviceProvider::GetDevice(int index)
	{
		return m_devices[index];
	}

	//特定仮想デバイスを返す
	HMDDeviceServerDriver& ServerTrackedDeviceProvider::GetHmdDevice()
	{
		return m_hmd[0];
	}

    //全仮想デバイスにリセットを指示する
    void ServerTrackedDeviceProvider::DeviceResetAll()
    {
        for (int i = 0; i < m_devices.size(); i++)
        {
            m_devices[i].Reset();
        }
		m_hmd[0].Reset();
    }

    //仮想デバイス内部インデックスの範囲内に収まっているかをチェックする
    bool ServerTrackedDeviceProvider::IsVMTDeviceIndex(int index)
    {
        return (index >= 0 && index <= m_devices.size());
    }

    //インストールパスの取得
    string ServerTrackedDeviceProvider::GetInstallPath()
    {
        return m_installPath;
    }

	//再起動要求
	void ServerTrackedDeviceProvider::RequestRestart() {
		VRServerDriverHost()->RequestRestart("*** RESTART REQUIRED ***", "", "", "");
	}

	//全デバイス情報文字列の取得
	std::string ServerTrackedDeviceProvider::GetOpenVRDevicesString()
	{
		std::string result("");

		IVRProperties* props = VRPropertiesRaw();
		CVRPropertyHelpers* helper = VRProperties();
		vr::TrackedDevicePose_t poses[k_unMaxTrackedDeviceCount]{};

		//OpenVRから全トラッキングデバイスの情報を取得する
		VRServerDriverHost()->GetRawTrackedDevicePoses(0.0f, poses, k_unMaxTrackedDeviceCount);

		//デバイスをOpenVR index順に調べる
		for (uint32_t i = 0; i < k_unMaxTrackedDeviceCount; i++) {
			//そのデバイスがつながっていないなら次のデバイスへ
			if (poses[i].bDeviceIsConnected != true) {
				continue;
			}

			//デバイスがつながっているので、シリアルナンバーを取得する
			PropertyContainerHandle_t h = props->TrackedDeviceToPropertyContainer(i);
			string SerialNumber = helper->GetStringProperty(h, ETrackedDeviceProperty::Prop_SerialNumber_String);
			int deviceClass = helper->GetInt32Property(h, ETrackedDeviceProperty::Prop_DeviceClass_Int32);

			//取得できた情報を文字列に格納する
			result = result + std::to_string(i) + ":" + std::to_string(deviceClass) + ":" + SerialNumber + "\n";
		}
		return result;
	}



    //** OpenVR向け関数群 **

    //OpenVRからのデバイスサーバー初期化
    EVRInitError ServerTrackedDeviceProvider::Init(IVRDriverContext* pDriverContext)
    {
        //OpenVR定形の初期化処理を実行
        VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext)

        //デバイスコンテキストを保持
        m_pDriverContext = pDriverContext;

        //ログをオープン
        Log::Open(VRDriverLog());

        //ドライバのインストールパス取得
        m_installPath = VRProperties()->GetStringProperty(m_pDriverContext->GetDriverHandle(), Prop_InstallPath_String);

        //通信のオープン
        CommunicationManager::GetInstance()->Open();

        //仮想デバイスを準備
        m_devices.resize(58); //58デバイス(全合計64に満たないくらい)
		m_hmd.resize(1);

        //仮想デバイスを初期化
        for (int i = 0; i < m_devices.size(); i++)
        {
            //シリアル番号を準備
            string name = Config::GetInstance()->GetSerialPrefix();
			name.append("_");
            name.append(std::to_string(i));
            m_devices[i].SetDeviceSerial(name);

            //仮想デバイス内部インデックス(OpenVRのindexではなく、Listのindex)をセット
            m_devices[i].SetObjectIndex(i);
        }

		{
			//シリアル番号を準備
			string name = Config::GetInstance()->GetSerialPrefix();
			name.append("_HMD");
			m_hmd[0].SetDeviceSerial(name);

			//仮想デバイス内部インデックス(OpenVRのindexではなく、Listのindex)をセット
			m_hmd[0].SetObjectIndex(0);
		}

        //起動完了
        Log::Output("Startup OK");
        return EVRInitError::VRInitError_None;
    }

    //OpenVRからのデバイスサーバー停止処理
    void ServerTrackedDeviceProvider::Cleanup()
    {
        //OpenVR定形の開放処理を実行
        VR_CLEANUP_SERVER_DRIVER_CONTEXT()
        m_pDriverContext = nullptr;

        //通信をクローズ
        CommunicationManager::GetInstance()->Close();

        //ログをクローズ
        Log::Close();
    }

    //OpenVRからのOpenVRインターフェースバージョン問い合わせ
    const char* const* ServerTrackedDeviceProvider::GetInterfaceVersions()
    {
        return k_InterfaceVersions;
    }

    //OpenVRからのフレーム処理
    void ServerTrackedDeviceProvider::RunFrame()
    {
        //通信の毎フレーム処理をする
        CommunicationManager::GetInstance()->Process();

        //各仮想デバイスの姿勢情報のアップデート
        size_t size = m_devices.size();
        for (int i = 0; i < size; i++)
        {
            m_devices[i].UpdatePoseToVRSystem();
        }
		m_hmd[0].UpdatePoseToVRSystem();

        //OpenVRイベントの取得
        VREvent_t VREvent;
        VRServerDriverHost()->PollNextEvent(&VREvent, sizeof(VREvent_t));

        //各仮想デバイスのイベントを処理
        for (int i = 0; i < size; i++)
        {
            m_devices[i].ProcessEvent(VREvent);
        }
		m_hmd[0].ProcessEvent(VREvent);
    }

    //OpenVRからのスタンバイブロック問い合わせ
    bool ServerTrackedDeviceProvider::ShouldBlockStandbyMode()
    {
        return false;
    }

    //OpenVRからのスタンバイ開始通知
    void ServerTrackedDeviceProvider::EnterStandby()
    {
    }

    //OpenVRからのスタンバイ終了通知
    void ServerTrackedDeviceProvider::LeaveStandby()
    {
    }
}