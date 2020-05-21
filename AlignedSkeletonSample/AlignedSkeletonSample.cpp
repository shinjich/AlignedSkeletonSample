#ifndef STRICT
#define STRICT	// 厳密なコードを型を要求する
#endif
#include <windows.h>
#include <tchar.h>
#include <stdio.h>
#include <k4a/k4a.h>
#include <k4abt.h>

#pragma comment( lib, "k4a.lib" )
#pragma comment( lib, "k4abt.lib" )

// イメージの解像度
#define RESOLUTION_WIDTH		(1920)
#define RESOLUTION_HEIGHT		(1080)

#define MAX_BODIES				8

// アプリケーションのタイトル名
static const TCHAR szClassName[] = TEXT("骨格座標をカラー座標に合わせるサンプル");
HWND g_hWnd = NULL;							// アプリケーションのウィンドウ
HPEN g_hPen = NULL;							// 描画用のペン

HBITMAP g_hBMP = NULL, g_hBMPold = NULL;	// 表示するビットマップのハンドル
HDC g_hDCBMP = NULL;						// 表示するビットマップのコンテキスト
BITMAPINFO g_biBMP = { 0, };				// ビットマップの情報 (解像度やフォーマット)
LPDWORD g_pdwPixel = NULL;					// ビットマップの中身の先頭 (ピクセル情報)
LPDWORD g_pColorMap = NULL;					// カラーマップバッファのポインタ

k4a_device_t g_hAzureKinect = nullptr;		// Azure Kinect のデバイスハンドル
k4abt_tracker_t g_hTracker = nullptr;		// ボディトラッカーのハンドル
k4a_calibration_t g_Calibration;			// Azure Kinect のキャリブレーションデータ
k4abt_skeleton_t g_Skeleton[MAX_BODIES];	// ユーザーの 3D 骨格情報
k4a_float2_t g_fSkeleton2D[MAX_BODIES][K4ABT_JOINT_COUNT] = { 0.0f, };	// ユーザーの 2D 骨格座標 (表示用)
uint32_t g_uBodyID[MAX_BODIES] = { K4ABT_INVALID_BODY_ID, };
uint32_t g_uBodies = 0;						// 骨格追跡されている人数

// Kinect を初期化する
k4a_result_t CreateKinect()
{
	k4a_result_t hr;

	// Azure Kinect を初期化する
	hr = k4a_device_open( K4A_DEVICE_DEFAULT, &g_hAzureKinect );
	if ( hr == K4A_RESULT_SUCCEEDED )
	{
		// カメラを開始する
		k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
		config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
		config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
		config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
		config.camera_fps = K4A_FRAMES_PER_SECOND_30;
		config.synchronized_images_only = false;
		config.depth_delay_off_color_usec = 0;
		config.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
		config.subordinate_delay_off_master_usec = 0;
		config.disable_streaming_indicator = false;

		// Azure Kinect を開始する
		hr = k4a_device_start_cameras( g_hAzureKinect, &config );
		if ( hr == K4A_RESULT_SUCCEEDED )
		{
			// キャリブレーションデータを取得する
			hr = k4a_device_get_calibration( g_hAzureKinect, config.depth_mode, config.color_resolution, &g_Calibration );
			if ( hr == K4A_RESULT_SUCCEEDED )
			{
				// 骨格追跡を開始する
				hr = k4abt_tracker_create( &g_Calibration, K4ABT_TRACKER_CONFIG_DEFAULT, &g_hTracker );
				if ( hr == K4A_RESULT_SUCCEEDED )
				{
					return hr;
				}
				else
				{
					MessageBox( NULL, TEXT("骨格追跡を開始できませんでした"), TEXT("エラー"), MB_OK );
				}
			}
			else
			{
				MessageBox( NULL, TEXT("キャリブレーション情報を取得できませんでした"), TEXT("エラー"), MB_OK );
			}
			// Azure Kinect を停止する
			k4a_device_stop_cameras( g_hAzureKinect );
		}
		else
		{
			MessageBox( NULL, TEXT("Azure Kinect を開始できませんでした"), TEXT("エラー"), MB_OK );
		}
		// Azure Kinect の使用をやめる
		k4a_device_close( g_hAzureKinect );
	}
	else
	{
		MessageBox( NULL, TEXT("Azure Kinect の初期化に失敗 - カメラの状態を確認してください"), TEXT("エラー"), MB_OK );
	}
	return hr;
}

// Kinect を終了する
void DestroyKinect()
{
	// 骨格追跡を無効にする
	if ( g_hTracker )
	{
		k4abt_tracker_destroy( g_hTracker );
		g_hTracker = nullptr;
	}

	if ( g_hAzureKinect )
	{
		// Azure Kinect を停止する
		k4a_device_stop_cameras( g_hAzureKinect );

		// Azure Kinect の使用をやめる
		k4a_device_close( g_hAzureKinect );
		g_hAzureKinect = nullptr;
	}
}

// KINECT のメインループ処理
uint32_t KinectProc()
{
	k4a_wait_result_t hr;
	uint32_t uImageSize = 0;
	uint32_t uBodies = 0;

	k4a_capture_t hCapture = nullptr;
	// カメラでキャプチャーする
	hr = k4a_device_get_capture( g_hAzureKinect, &hCapture, K4A_WAIT_INFINITE );
	if ( hr == K4A_WAIT_RESULT_SUCCEEDED )
	{
		k4a_image_t hImage;

		// カラーイメージを取得する
		hImage = k4a_capture_get_color_image( hCapture );

		// 骨格追跡をキューする
		hr = k4abt_tracker_enqueue_capture( g_hTracker, hCapture, K4A_WAIT_INFINITE );
		// カメラキャプチャーを解放する
		k4a_capture_release( hCapture );

		if ( hImage )
		{
			// イメージピクセルの先頭ポインタを取得する
			uint8_t* p = k4a_image_get_buffer( hImage );
			if ( p )
			{
				// イメージサイズを取得する
				uImageSize = (uint32_t) k4a_image_get_size( hImage );
				CopyMemory( g_pColorMap, p, uImageSize );
			}
			// イメージを解放する
			k4a_image_release( hImage );
		}

		if ( hr == K4A_WAIT_RESULT_SUCCEEDED )
		{
			// 骨格追跡の結果を取得する
			k4abt_frame_t hBodyFrame = nullptr;
			hr = k4abt_tracker_pop_result( g_hTracker, &hBodyFrame, K4A_WAIT_INFINITE );
			if ( hr == K4A_WAIT_RESULT_SUCCEEDED )
			{
				// 認識された人数を取得する
				uBodies = k4abt_frame_get_num_bodies( hBodyFrame );
				if ( uBodies > MAX_BODIES )
					uBodies = MAX_BODIES;
				for( uint32_t uBody = 0; uBody < uBodies; uBody++ )
				{
					// 各人の骨格情報を取得する
					if ( k4abt_frame_get_body_skeleton( hBodyFrame, uBody, &g_Skeleton[uBody] ) == K4A_RESULT_SUCCEEDED )
					{
						g_uBodyID[uBody] = k4abt_frame_get_body_id( hBodyFrame, uBody );
						for( int iJoint = K4ABT_JOINT_PELVIS; iJoint < K4ABT_JOINT_COUNT; iJoint++ )
						{
							int iValid = 0;
							// 3D の骨格座標を 2D スクリーン座標に変換する
							k4a_calibration_3d_to_2d( &g_Calibration, &g_Skeleton[uBody].joints[iJoint].position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &g_fSkeleton2D[uBody][iJoint], &iValid );
							if ( iValid == 0 )
							{
								// 無効な値は (0,0) に設定
								g_fSkeleton2D[uBody][iJoint].xy.x = g_fSkeleton2D[uBody][iJoint].xy.y = 0.0f;
							}
						}
					}
				}
				// 骨格追跡フレームを解放する
				k4abt_frame_release( hBodyFrame );
				g_uBodies = uBodies;
			}
		}
	}
	return uImageSize;
}

LRESULT CALLBACK WndProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam )
{
	switch( uMsg )
	{
	case WM_PAINT:
		{
			// 画面表示処理
			PAINTSTRUCT ps;
			HDC hDC = BeginPaint( hWnd, &ps );

			// 画面サイズを取得する
			RECT rect;
			GetClientRect( hWnd, &rect );

			CopyMemory( g_pdwPixel, g_pColorMap, RESOLUTION_WIDTH * RESOLUTION_HEIGHT * sizeof(DWORD) );

			// カラー化した深度の表示
			StretchBlt( hDC, 0, 0, rect.right, rect.bottom, g_hDCBMP, 0, 0, RESOLUTION_WIDTH, RESOLUTION_HEIGHT, SRCCOPY );

			// 深度映像の縦横幅を取得して画面サイズとの割合を求める
			const DWORD dwWidth = RESOLUTION_WIDTH;
			const DWORD dwHeight = RESOLUTION_HEIGHT;
			const FLOAT fRelativeWidth = (FLOAT) rect.right / (FLOAT) dwWidth;
			const FLOAT fRelativeHeight = (FLOAT) rect.bottom / (FLOAT) dwHeight;

			// ペンを選択
			HPEN hPenPrev = (HPEN) SelectObject( hDC, g_hPen );

			// 文字の背景を透明に
			SetBkMode( hDC, TRANSPARENT );

			for( uint32_t uBody = 0; uBody < g_uBodies; uBody++ )
			{
				// 骨格を画面サイズに合わせてスケーリング
				LONG lSkeletonX[K4ABT_JOINT_COUNT];
				LONG lSkeletonY[K4ABT_JOINT_COUNT];
				for( int iJoint = K4ABT_JOINT_PELVIS; iJoint < K4ABT_JOINT_COUNT; iJoint++ )
				{
					lSkeletonX[iJoint] = (LONG) (g_fSkeleton2D[uBody][iJoint].xy.x * fRelativeWidth);
					lSkeletonY[iJoint] = (LONG) (g_fSkeleton2D[uBody][iJoint].xy.y * fRelativeHeight);
				}

				// 顔を線で結ぶ
				MoveToEx( hDC, lSkeletonX[K4ABT_JOINT_EAR_LEFT], lSkeletonY[K4ABT_JOINT_EAR_LEFT], NULL );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_EYE_LEFT], lSkeletonY[K4ABT_JOINT_EYE_LEFT] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_NOSE], lSkeletonY[K4ABT_JOINT_NOSE] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_EYE_RIGHT], lSkeletonY[K4ABT_JOINT_EYE_RIGHT] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_EAR_RIGHT], lSkeletonY[K4ABT_JOINT_EAR_RIGHT] );

				// 体を線で結ぶ
				MoveToEx( hDC, lSkeletonX[K4ABT_JOINT_PELVIS], lSkeletonY[K4ABT_JOINT_PELVIS], NULL );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_SPINE_NAVEL], lSkeletonY[K4ABT_JOINT_SPINE_NAVEL] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_SPINE_CHEST], lSkeletonY[K4ABT_JOINT_SPINE_CHEST] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_NECK], lSkeletonY[K4ABT_JOINT_NECK] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_HEAD], lSkeletonY[K4ABT_JOINT_HEAD] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_NOSE], lSkeletonY[K4ABT_JOINT_NOSE] );

				// 腕を線で結ぶ
				MoveToEx( hDC, lSkeletonX[K4ABT_JOINT_HAND_LEFT], lSkeletonY[K4ABT_JOINT_HAND_LEFT], NULL );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_WRIST_LEFT], lSkeletonY[K4ABT_JOINT_WRIST_LEFT] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_ELBOW_LEFT], lSkeletonY[K4ABT_JOINT_ELBOW_LEFT] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_SHOULDER_LEFT], lSkeletonY[K4ABT_JOINT_SHOULDER_LEFT] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_CLAVICLE_LEFT], lSkeletonY[K4ABT_JOINT_CLAVICLE_LEFT] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_SPINE_CHEST], lSkeletonY[K4ABT_JOINT_SPINE_CHEST] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_CLAVICLE_RIGHT], lSkeletonY[K4ABT_JOINT_CLAVICLE_RIGHT] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_SHOULDER_RIGHT], lSkeletonY[K4ABT_JOINT_SHOULDER_RIGHT] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_ELBOW_RIGHT], lSkeletonY[K4ABT_JOINT_ELBOW_RIGHT] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_WRIST_RIGHT], lSkeletonY[K4ABT_JOINT_WRIST_RIGHT] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_HAND_RIGHT], lSkeletonY[K4ABT_JOINT_HAND_RIGHT] );

				// 左手を線で結ぶ
				MoveToEx( hDC, lSkeletonX[K4ABT_JOINT_HANDTIP_LEFT], lSkeletonY[K4ABT_JOINT_HANDTIP_LEFT], NULL );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_HAND_LEFT], lSkeletonY[K4ABT_JOINT_HAND_LEFT] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_THUMB_LEFT], lSkeletonY[K4ABT_JOINT_THUMB_LEFT] );

				// 右手を線で結ぶ
				MoveToEx( hDC, lSkeletonX[K4ABT_JOINT_HANDTIP_RIGHT], lSkeletonY[K4ABT_JOINT_HANDTIP_RIGHT], NULL );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_HAND_RIGHT], lSkeletonY[K4ABT_JOINT_HAND_RIGHT] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_THUMB_RIGHT], lSkeletonY[K4ABT_JOINT_THUMB_RIGHT] );

				// 脚を線で結ぶ
				MoveToEx( hDC, lSkeletonX[K4ABT_JOINT_FOOT_LEFT], lSkeletonY[K4ABT_JOINT_FOOT_LEFT], NULL );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_ANKLE_LEFT], lSkeletonY[K4ABT_JOINT_ANKLE_LEFT] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_KNEE_LEFT], lSkeletonY[K4ABT_JOINT_KNEE_LEFT] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_HIP_LEFT], lSkeletonY[K4ABT_JOINT_HIP_LEFT] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_PELVIS], lSkeletonY[K4ABT_JOINT_PELVIS] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_HIP_RIGHT], lSkeletonY[K4ABT_JOINT_HIP_RIGHT] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_KNEE_RIGHT], lSkeletonY[K4ABT_JOINT_KNEE_RIGHT] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_ANKLE_RIGHT], lSkeletonY[K4ABT_JOINT_ANKLE_RIGHT] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_FOOT_RIGHT], lSkeletonY[K4ABT_JOINT_FOOT_RIGHT] );
			}
			// ペンを元に戻す
			SelectObject( hDC, hPenPrev );

			EndPaint( hWnd, &ps );
		}
		return 0;
	case WM_CLOSE:
		DestroyWindow( hWnd );
	case WM_DESTROY:
		PostQuitMessage( 0 );
		break;
	default:
		return DefWindowProc( hWnd, uMsg, wParam, lParam );
	}
	return 0;
}

// アプリケーションの初期化 (ウィンドウや描画用のペンを作成)
HRESULT InitApp( HINSTANCE hInst, int nCmdShow )
{
	WNDCLASSEX wc = { 0, };
	wc.cbSize = sizeof(WNDCLASSEX);
	wc.style = CS_HREDRAW | CS_VREDRAW;
	wc.lpfnWndProc = WndProc;
	wc.hInstance = hInst;
	wc.hIcon = LoadIcon( NULL, IDI_APPLICATION );
	wc.hCursor = LoadCursor( NULL, IDC_ARROW );
	wc.hbrBackground = (HBRUSH) GetStockObject( NULL_BRUSH );
	wc.lpszClassName = szClassName;
	wc.hIconSm = LoadIcon( NULL, IDI_APPLICATION );
	if ( ! RegisterClassEx( &wc ) )
	{
		MessageBox( NULL, TEXT("アプリケーションクラスの初期化に失敗"), TEXT("エラー"), MB_OK );
		return E_FAIL;
	}

	// アプリケーションウィンドウを作成
	g_hWnd = CreateWindow( szClassName, szClassName, WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT, CW_USEDEFAULT, CW_USEDEFAULT, NULL, NULL, hInst, NULL );
	if ( ! g_hWnd )
	{
		MessageBox( NULL, TEXT("ウィンドウの初期化に失敗"), TEXT("エラー"), MB_OK );
		return E_FAIL;
	}

	// 文字表示用のペンを作成
	g_hPen = CreatePen( PS_SOLID, 3, RGB( 0, 255, 0 ) );
	if ( ! g_hPen )
	{
		MessageBox( NULL, TEXT("ペンの初期化に失敗"), TEXT("エラー"), MB_OK );
		return E_FAIL;
	}

	// 画面表示用のビットマップを作成する
	ZeroMemory( &g_biBMP, sizeof(g_biBMP) );
	g_biBMP.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
	g_biBMP.bmiHeader.biBitCount = 32;
	g_biBMP.bmiHeader.biPlanes = 1;
	g_biBMP.bmiHeader.biWidth = RESOLUTION_WIDTH;
	g_biBMP.bmiHeader.biHeight = -(int) RESOLUTION_HEIGHT;
	g_hBMP = CreateDIBSection( NULL, &g_biBMP, DIB_RGB_COLORS, (LPVOID*) (&g_pdwPixel), NULL, 0 );
	HDC hDC = GetDC( g_hWnd );
	g_hDCBMP = CreateCompatibleDC( hDC );
	ReleaseDC( g_hWnd, hDC );
	g_hBMPold = (HBITMAP) SelectObject( g_hDCBMP, g_hBMP );

	// 深度マップ用バッファを作成
	g_pColorMap = new DWORD[RESOLUTION_WIDTH * RESOLUTION_HEIGHT];
	memset( g_pColorMap, 0, RESOLUTION_WIDTH * RESOLUTION_HEIGHT * sizeof(DWORD) );

	// ウィンドウを表示する
	ShowWindow( g_hWnd, nCmdShow );
	UpdateWindow( g_hWnd );

	return S_OK;
}

// アプリケーションの後始末
HRESULT UninitApp()
{
	// カラーマップを解放する
	if ( g_pColorMap )
	{
		delete [] g_pColorMap;
		g_pColorMap = NULL;
	}

	// 画面表示用のビットマップを解放する
	if ( g_hDCBMP || g_hBMP )
	{
		SelectObject( g_hDCBMP, g_hBMPold );
		DeleteObject( g_hBMP );
		DeleteDC( g_hDCBMP );
		g_hBMP = NULL;
		g_hDCBMP = NULL;
	}

	// 文字表示用のペンを削除
	if ( g_hPen )
	{
		DeleteObject( (HGDIOBJ) g_hPen );
		g_hPen = NULL;
	}
	return S_OK;
}

// エントリーポイント
int WINAPI WinMain( HINSTANCE hInst, HINSTANCE, LPSTR, int nCmdShow )
{
	// アプリケーションを初期化する
	if ( FAILED( InitApp( hInst, nCmdShow ) ) )
		return 1;

	// KINECT を初期化する
	if ( FAILED( CreateKinect() ) )
		return 1;

	// アプリケーションループ
	MSG msg;
	while( GetMessage( &msg, NULL, 0, 0 ) )
	{
		// ウィンドウメッセージを処理
		TranslateMessage( &msg );
		DispatchMessage( &msg );

		// Kinect 情報に更新があれば描画
		if ( KinectProc() )
		{
			// 描画
			InvalidateRect( g_hWnd, NULL, TRUE );
		}
	}

	// KINECT を終了する
	DestroyKinect();

	// アプリケーションを終了する
	UninitApp();

	return (int) msg.wParam;
}
