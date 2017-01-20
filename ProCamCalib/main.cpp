
#include "Graycode.h"
#include "Header.h"
#include "PGROpenCV.h"
#include "Calibration.h"


#define MASK_ADDRESS "./GrayCodeImage/mask.bmp"
#define IMAGE_DIRECTORY "./UseImage"
#define SAVE_DIRECTORY "./UseImage/resize"


TPGROpenCV	pgrOpenCV(0);

void eular2rot(double yaw,double pitch, double roll, cv::Mat& dest);

int main()
{

	printf("0：ProCam間の幾何対応を取得\n");
	printf("1：今まで取得した幾何対応からProCamキャリブレーション\n");
	printf("2：保存してある幾何対応からProCamキャリブレーション\n");
	printf("3：キャリブレーションの読み込み\n");
	printf("4：キャリブレーション結果を使って投影\n");
	printf("5：再投影誤差の計測\n");
	printf("6：3次元復元\n");

	printf("w：待機時に白画像を投影するかしないか\n");
	printf("\n");

	pgrOpenCV.init(FlyCapture2::PIXEL_FORMAT_MONO8, FlyCapture2::HQ_LINEAR);
	//pgrOpenCV.setCameraParams(4.0);
	GRAYCODE gc;

	// カメラ画像確認用
	char windowNameCamera[] = "camera";
	cv::namedWindow(windowNameCamera, cv::WINDOW_AUTOSIZE);
	cv::moveWindow(windowNameCamera, 500, 300);

	static bool prjWhite = true;


	// キャリブレーション用
	Calibration calib(10, 7, 23.0);
	std::vector<std::vector<cv::Point3f>>	worldPoints;
	std::vector<std::vector<cv::Point2f>>	cameraPoints;
	std::vector<std::vector<cv::Point2f>>	projectorPoints;
	int calib_count = 0;


	// キー入力受付用の無限ループ
	while(true){
		printf("====================\n");
		printf("数字を入力してください....\n");
		int command;

		// 白い画像を全画面で投影（撮影環境を確認しやすくするため）
		pgrOpenCV.start();
		cv::Mat cam, cam2;
		while(true){
			// trueで白を投影、falseで通常のディスプレイを表示
			if(prjWhite){
				cv::Mat white = cv::Mat(PROJECTOR_WIDTH, PROJECTOR_HEIGHT, CV_8UC3, cv::Scalar(255, 255, 255));
				//cv::Mat white = cv::imread("chessPattern_1920_1080_76_76_77.png");
				cv::namedWindow("white_black", 0);
				Projection::MySetFullScrean(DISPLAY_NUMBER, "white_black");
				cv::imshow("white_black", white);
			}

			// 何かのキーが入力されたらループを抜ける
			command = cv::waitKey(33);
			if ( command > 0 ) break;

			pgrOpenCV.queryFrame();
			cam = pgrOpenCV.getVideo();
			cam.copyTo(cam2);

			//見やすいように適当にリサイズ
			cv::resize(cam, cam, cv::Size(), 0.45, 0.45);
			cv::imshow(windowNameCamera, cam);
		}

		// カメラを止める
		pgrOpenCV.stop();
		cv::destroyWindow("white_black");

		// 条件分岐
		switch (command){

		case '0':
			{

				// チェッカーパターンの交点を描画(カメラ)
				std::vector<cv::Point2f> imagePoint;
				std::vector<cv::Point2f> projPoint;
				cv::Mat draw_corner;
				bool detect_flag = calib.getCheckerCorners(imagePoint, cam2, draw_corner);
				cv::imshow( "Image Corner", draw_corner);

				// カメラ上でチェッカーパターンを検出できたら
				if (detect_flag)
				{
					// グレイコード投影
					gc.code_projection();
					gc.make_thresh();
					gc.makeCorrespondence();


					// チェッカーパターンの交点を描画(プロジェクタ)
					cv::Mat dst;
					gc.getCorrespondSubPixelProjPoints(projPoint, imagePoint, 20);
					gc.transport_camera_projector(cam2,dst);

					if(imagePoint.size() == projPoint.size()) {
						cv::drawChessboardCorners( dst, calib.checkerPattern, projPoint, true );
					} else {
						cv::drawChessboardCorners( dst, calib.checkerPattern, projPoint, false );
					}

					cv::imshow("prj",dst);
					cv::imwrite("./prj.jpg", dst );


					// チェッカーパターンの交点を投影
					cv::destroyWindow("white_black");
					cv::namedWindow("Corner", 0);
					Projection::MySetFullScrean(DISPLAY_NUMBER, "Corner");
					cv::imshow("Corner", dst);

					// プロジェクタ上でチェッカーパターンを検出できたら
					if(imagePoint.size() == projPoint.size())
					{
						std::cout << calib_count+1 << "回目の検出成功" << std::endl;

						// 追加
						worldPoints.emplace_back(calib.worldPoint);
						cameraPoints.emplace_back(imagePoint);
						projectorPoints.emplace_back(projPoint);

						// 検出点の保存
						std::string fileName = "checkerPoint" + std::to_string(calib_count) + ".xml";
						cv::FileStorage fs(fileName, cv::FileStorage::WRITE);
						cv::write(fs,"world", calib.worldPoint);
						cv::write(fs,"camera", imagePoint);
						cv::write(fs,"projector", projPoint);

						calib_count++;
					}else{
						std::cout << "プロジェクタ上のチェッカーパターンの検出失敗" << std::endl;
					}
				} else {
					std::cout << "カメラ上のチェッカーパターンの検出失敗" << std::endl;
				}

				cv::waitKey(0);
			}
			break;

		case '1':

			if(calib_count > 2)
			{
				std::cout << "キャリブレーション中…" << std::endl;

				// キャリブレーション
				calib.proCamCalibration(worldPoints, cameraPoints, projectorPoints, cv::Size(CAMERA_WIDTH, CAMERA_HEIGHT), cv::Size(PROJECTOR_WIDTH, PROJECTOR_HEIGHT));

			} else {
				std::cout << "" << std::endl;
			}
			break;

		case '2':
			{
				worldPoints.clear();
				cameraPoints.clear();
				projectorPoints.clear();

				// ファイルの探索
				WIN32_FIND_DATA ffd;
				HANDLE hF;

				std::string imageFlieName = "checkerPoint*.*";
				hF = FindFirstFile( imageFlieName.c_str(), &ffd);
				if (hF != INVALID_HANDLE_VALUE) {
					// フォルダ内のファイルの探索
					do {
						std::string fullpath = ffd.cFileName;

						std::vector<cv::Point3f> worldPoint;
						std::vector<cv::Point2f> imagePoint;
						std::vector<cv::Point2f> projPoint;

						// xmlファイルの読み込み
						cv::FileStorage cvfs(fullpath, cv::FileStorage::READ);

						cvfs["world"] >> worldPoint;
						cvfs["camera"] >> imagePoint;
						cvfs["projector"] >> projPoint;
						
						// 追加
						worldPoints.emplace_back(worldPoint);
						cameraPoints.emplace_back(imagePoint);
						projectorPoints.emplace_back(projPoint);

					} while (FindNextFile(hF, &ffd ) != 0);
					FindClose(hF);
				}

				if(worldPoints.size() > 0)
				{
					std::cout << "キャリブレーション中…" << std::endl;

					// キャリブレーション
					calib.proCamCalibration(worldPoints, cameraPoints, projectorPoints, cv::Size(CAMERA_WIDTH, CAMERA_HEIGHT), cv::Size(PROJECTOR_WIDTH, PROJECTOR_HEIGHT));

				}
			}
			break;

		case '3':

			std::cout << "キャリブレーション結果の読み込み中…" << std::endl;
			calib.loadCalibParam("calibration.xml");
				
			break;

		case '4':

			if(calib.calib_flag)
			{
				std::cout << "チェッカーパターンへの再投影中…" << std::endl;
				
				// 透視投影行列の生成
				cv::Mat cam_perspective = calib.getCamPerspectiveMat();
				cv::Mat proj_perspective = calib.getProjPerspectiveMat();

				pgrOpenCV.start();
				cv::Mat white = cv::Mat(PROJECTOR_HEIGHT, PROJECTOR_WIDTH, CV_8UC3, cv::Scalar(255, 255, 255));
				cv::Mat proj_img = cv::Mat(PROJECTOR_HEIGHT, PROJECTOR_WIDTH, CV_8UC3, cv::Scalar(255, 255, 255));
				while(true){
					// 投影画像の更新
					white.copyTo(proj_img);


					// 何かのキーが入力されたらループを抜ける
					command = cv::waitKey(33);
					if ( command > 0 ) break;

					pgrOpenCV.queryFrame();
					cam = pgrOpenCV.getVideo();
					
					// チェッカーパターンの交点を描画(カメラ)
					cv::Mat cam3;
					cv::undistort(cam, cam3, calib.cam_K, calib.cam_dist);		// 歪み除去

					std::vector<cv::Point3f> worldPoint;
					std::vector<cv::Point2f> imagePoint;
					std::vector<cv::Point2f> projPoint;
					bool detect_flag = calib.getCheckerCorners(imagePoint, cam3, cam3);						// チェッカーパターン検出

					if(detect_flag) {
						// カメラを中心としたチェッカーパターンの位置取得
						calib.getCameraWorldPoint(worldPoint, imagePoint);

						// カメラ座標への投影(歪み除去後のため歪みパラメータは用いない)
						std::vector<cv::Point2f> cam_projection;
						cv::Mat cam_R = cv::Mat::eye(3, 3, CV_64F);
						cv::Mat cam_T = cv::Mat::zeros(3, 1, CV_64F);
						cv::projectPoints(worldPoint, cam_R, cam_T, calib.cam_K, cv::Mat(), cam_projection);

						// カメラの再投影誤差
						float cam_error = 0;
						for(int i=0; i<imagePoint.size(); ++i)
						{
							cam_error += std::sqrt((imagePoint[i].x - cam_projection[i].x)*(imagePoint[i].x - cam_projection[i].x) + (imagePoint[i].y - cam_projection[i].y)*(imagePoint[i].y - cam_projection[i].y));
						}

						std::cout << "カメラの再投影誤差：" << cam_error/imagePoint.size() << std::endl;

						// プロジェクタ座標への投影(歪みあり)
						cv::projectPoints(worldPoint, calib.R, calib.T, calib.proj_K, calib.proj_dist, projPoint);

						// プロジェクタ投影画像
						cv::drawChessboardCorners( proj_img, calib.checkerPattern, projPoint, true );
					}

					// 映像の投影
					cv::namedWindow("white_black", 0);
					Projection::MySetFullScrean(DISPLAY_NUMBER, "white_black");
					cv::imshow("white_black", proj_img);


					//見やすいように適当にリサイズ
					cv::resize(cam3, cam3, cv::Size(), 0.45, 0.45);
					cv::imshow(windowNameCamera, cam3);
				}

				// カメラを止める
				pgrOpenCV.stop();
				cv::destroyWindow("white_black");


			} else {
				std::cout << "キャリブレーションデータがありません" << std::endl;
			}
				
			break;

		case '5':
			{
				if(calib.calib_flag)
				{
					// チェッカーパターンの交点を描画(カメラ)
					std::vector<cv::Point3f> worldPoint;
					std::vector<cv::Point2f> imagePoint;
					std::vector<cv::Point2f> projPoint;
					cv::Mat draw_corner;
					bool detect_flag = calib.getCheckerCorners(imagePoint, cam2, draw_corner);

					// カメラの歪み除去
					cv::Mat draw_corner2;
					cv::undistort(draw_corner, draw_corner2, calib.cam_K, calib.cam_dist);		// 歪み除去

					cv::imshow( "Image Corner", draw_corner2);

					// カメラ上でチェッカーパターンを検出できたら
					if (detect_flag)
					{
						std::cout << "再投影誤差の計測中…" << std::endl;

						// グレイコード投影
						gc.code_projection();
						gc.make_thresh();
						gc.makeCorrespondence();

						// チェッカーパターンの交点を描画(プロジェクタ)
						cv::Mat dst;
						gc.getCorrespondSubPixelProjPoints(projPoint, imagePoint, 30);		// 対応点の取得
						gc.transport_camera_projector(cam2,dst);

						if(imagePoint.size() == projPoint.size()) {
							cv::drawChessboardCorners( dst, calib.checkerPattern, projPoint, true );
						} else {
							cv::drawChessboardCorners( dst, calib.checkerPattern, projPoint, false );
						}
						cv::imshow("prj",dst);

						
						// 対応点の歪み除去
						std::vector<cv::Point2f> undistort_imagePoint;
						std::vector<cv::Point2f> undistort_projPoint;
						cv::undistortPoints(imagePoint, undistort_imagePoint, calib.cam_K, calib.cam_dist);
						cv::undistortPoints(projPoint, undistort_projPoint, calib.proj_K, calib.proj_dist);
						for(int i=0; i<imagePoint.size(); ++i)
						{
							undistort_imagePoint[i].x = undistort_imagePoint[i].x * calib.cam_K.at<double>(0,0) + calib.cam_K.at<double>(0,2);
							undistort_imagePoint[i].y = undistort_imagePoint[i].y * calib.cam_K.at<double>(1,1) + calib.cam_K.at<double>(1,2);
							undistort_projPoint[i].x = undistort_projPoint[i].x * calib.proj_K.at<double>(0,0) + calib.proj_K.at<double>(0,2);
							undistort_projPoint[i].y = undistort_projPoint[i].y * calib.proj_K.at<double>(1,1) + calib.proj_K.at<double>(1,2);
						}

						// カメラを中心としたチェッカーパターンの位置取得
						calib.getCameraWorldPoint(worldPoint, undistort_imagePoint);

						// カメラ座標への投影(歪み除去後のため歪みパラメータは用いない)
						std::vector<cv::Point2f> cam_projection;
						cv::Mat cam_R = cv::Mat::eye(3, 3, CV_64F);
						cv::Mat cam_T = cv::Mat::zeros(3, 1, CV_64F);
						cv::projectPoints(worldPoint, cam_R, cam_T, calib.cam_K, cv::Mat(), cam_projection);

						// カメラの再投影誤差
						float cam_error = 0;
						for(int i=0; i<undistort_imagePoint.size(); ++i)
						{
							cam_error += std::sqrt((undistort_imagePoint[i].x - cam_projection[i].x)*(undistort_imagePoint[i].x - cam_projection[i].x) + (undistort_imagePoint[i].y - cam_projection[i].y)*(undistort_imagePoint[i].y - cam_projection[i].y));
						}

						std::cout << "カメラの再投影誤差：" << cam_error/undistort_imagePoint.size() << std::endl;


						// プロジェクタ座標への投影(歪み除去後のため歪みパラメータは用いない)
						std::vector<cv::Point2f> proj_projection;
						cv::projectPoints(worldPoint, calib.R, calib.T, calib.proj_K, cv::Mat(), proj_projection);

						// プロジェクタの再投影誤差
						float proj_error = 0;
						for(int i=0; i<undistort_projPoint.size(); ++i)
						{
							proj_error += std::sqrt((undistort_projPoint[i].x - proj_projection[i].x)*(undistort_projPoint[i].x - proj_projection[i].x) + (undistort_projPoint[i].y - proj_projection[i].y)*(undistort_projPoint[i].y - proj_projection[i].y));
						}

						std::cout << "プロジェクタの再投影誤差：" << proj_error/undistort_projPoint.size() << std::endl;


						// プロジェクタ投影画像
						cv::Mat proj_img = cv::Mat(PROJECTOR_HEIGHT, PROJECTOR_WIDTH, CV_8UC3, cv::Scalar(255, 255, 255));
						std::vector<cv::Point2f> distort_projection;
						cv::projectPoints(worldPoint, calib.R, calib.T, calib.proj_K, calib.proj_dist, distort_projection);		// 投影用に歪み補正
						cv::drawChessboardCorners( proj_img, calib.checkerPattern, distort_projection, true );
						cv::namedWindow("proj_img", 0);
						Projection::MySetFullScrean(DISPLAY_NUMBER, "proj_img");
						cv::imshow("proj_img", proj_img);

						cv::waitKey(0);
					}
				}
			}
			break;

		case '6':
			if(calib.calib_flag)
			{
				std::cout << "3次元復元中…" << std::endl;

				// グレイコード投影
				gc.code_projection();
				gc.make_thresh();
				gc.makeCorrespondence();

				//***対応点の取得(プロジェクタ画素→3次元点)******************************************
				std::vector<cv::Point2f> imagePoint;
				std::vector<cv::Point2f> projPoint;
				std::vector<int> isValid; //有効な対応点かどうかのフラグ
				std::vector<cv::Point3f> reconstructPoint;
				gc.getCorrespondAllPoints(projPoint, imagePoint, isValid);
				//gc.getCorrespondAllPoints_ProCam(projPoint, imagePoint);

				// 対応点の歪み除去
				std::vector<cv::Point2f> undistort_imagePoint;
				std::vector<cv::Point2f> undistort_projPoint;
				cv::undistortPoints(imagePoint, undistort_imagePoint, calib.cam_K, calib.cam_dist);
				cv::undistortPoints(projPoint, undistort_projPoint, calib.proj_K, calib.proj_dist);
				for(int i=0; i<imagePoint.size(); ++i)
				{
					if(isValid[i] == 1)
					{
						undistort_imagePoint[i].x = undistort_imagePoint[i].x * calib.cam_K.at<double>(0,0) + calib.cam_K.at<double>(0,2);
						undistort_imagePoint[i].y = undistort_imagePoint[i].y * calib.cam_K.at<double>(1,1) + calib.cam_K.at<double>(1,2);
						undistort_projPoint[i].x = undistort_projPoint[i].x * calib.proj_K.at<double>(0,0) + calib.proj_K.at<double>(0,2);
						undistort_projPoint[i].y = undistort_projPoint[i].y * calib.proj_K.at<double>(1,1) + calib.proj_K.at<double>(1,2);
					}
					else
					{
						undistort_imagePoint[i].x = -1;
						undistort_imagePoint[i].y = -1;
						undistort_projPoint[i].x = -1;
						undistort_projPoint[i].y = -1;
					}
				}

				////エピポーラ方程式が成り立つか確認
				//for(int i =0; i < undistort_imagePoint.size(); i++)
				//{
				//	if(isValid[i] == 1)
				//	{
				//		cv::Mat cp = (cv::Mat_<double>(3, 1) << (double)undistort_imagePoint.at(i).x,  (double)undistort_imagePoint.at(i).y,  1);
				//		cv::Mat pp = (cv::Mat_<double>(3, 1) << (double)undistort_projPoint.at(i).x,  (double)undistort_projPoint.at(i).y,  1);
				//		cv::Mat projK_inv_t = calib.proj_K.inv().t();
				//		cv::Mat tx = (cv::Mat_<double>(3, 3) << 0, -calib.T.at<double>(2,0), calib.T.at<double>(1,0), calib.T.at<double>(2,0), 0, -calib.T.at<double>(0,0), -calib.T.at<double>(1,0), calib.T.at<double>(0,0), 0);
				//		cv::Mat e = pp.t() * projK_inv_t * tx * calib.R * calib.cam_K.inv() * cp;
				//		double error = e.at<double>(0, 0);
				//		std::cout << "e[" << i << "] = " << error << std::endl; 
				//	}
				//}

				// 3次元復元
				calib.reconstruction(reconstructPoint, undistort_projPoint, undistort_imagePoint, isValid);

				//==保存==//
				cv::FileStorage fs("./reconstructPoints_proj.xml", cv::FileStorage::WRITE);
				//プロジェクタ画素→3次元点
				write(fs, "points", reconstructPoint);
				std::cout << "pro->3d points saved." << std::endl;

				//**********************************************************************************

				//***対応点の取得(カメラ画素→3次元点)******************************************
				std::vector<cv::Point2f> imagePoint_camera;
				std::vector<cv::Point2f> projPoint_camera;
				std::vector<int> isValid_camera; //有効な対応点かどうかのフラグ
				std::vector<cv::Point3f> reconstructPoint_camera;
				gc.getCorrespondAllPoints_ProCam(projPoint_camera, imagePoint_camera, isValid_camera);

				// 対応点の歪み除去
				std::vector<cv::Point2f> undistort_imagePoint_camera;
				std::vector<cv::Point2f> undistort_projPoint_camera;
				cv::undistortPoints(imagePoint_camera, undistort_imagePoint_camera, calib.cam_K, calib.cam_dist);
				cv::undistortPoints(projPoint_camera, undistort_projPoint_camera, calib.proj_K, calib.proj_dist);
				for(int i=0; i<imagePoint_camera.size(); ++i)
				{
					if(isValid_camera[i] == 1)
					{
						undistort_imagePoint_camera[i].x = undistort_imagePoint_camera[i].x * calib.cam_K.at<double>(0,0) + calib.cam_K.at<double>(0,2);
						undistort_imagePoint_camera[i].y = undistort_imagePoint_camera[i].y * calib.cam_K.at<double>(1,1) + calib.cam_K.at<double>(1,2);
						undistort_projPoint_camera[i].x = undistort_projPoint_camera[i].x * calib.proj_K.at<double>(0,0) + calib.proj_K.at<double>(0,2);
						undistort_projPoint_camera[i].y = undistort_projPoint_camera[i].y * calib.proj_K.at<double>(1,1) + calib.proj_K.at<double>(1,2);
					}
					else
					{
						undistort_imagePoint_camera[i].x = -1;
						undistort_imagePoint_camera[i].y = -1;
						undistort_projPoint_camera[i].x = -1;
						undistort_projPoint_camera[i].y = -1;
					}
				}

				// 3次元復元
				calib.reconstruction(reconstructPoint_camera, undistort_projPoint_camera, undistort_imagePoint_camera, isValid_camera);

				//==保存==//
				cv::FileStorage fs_camera("./reconstructPoints_camera.xml", cv::FileStorage::WRITE);
				//プロジェクタ画素→3次元点
				write(fs_camera, "points", reconstructPoint_camera);
				std::cout << "camera->3d points saved." << std::endl;

				//**********************************************************************************

/*
				//カメラ画素→3次元点
				std::vector<cv::Point3f> reconstructPoint_camera;
				for( int y = 0; y < CAMERA_HEIGHT; y++ ) {
					for( int x = 0; x < CAMERA_WIDTH; x++ ) {
						cv::Point pp = gc.c->ProCam[y][x];
						if(pp.x != -1) //プロジェクタの投影範囲内に入っていたら、3次元点を取得する
						{
							int index = pp.y * PROJECTOR_WIDTH + pp.x;
							reconstructPoint_camera.emplace_back(reconstructPoint.at(index));
						}else
						{
							reconstructPoint_camera.emplace_back(cv::Point3f(-1, -1, -1));
						}
					}
				}
				//保存
				cv::FileStorage fs_c("./reconstructPoints_camera.xml", cv::FileStorage::WRITE);
				//カメラ画素→3次元点
				write(fs_c, "points", reconstructPoint_camera);
				std::cout << "cam->3d points saved." << std::endl;
*/



				// 描画
				cv::Mat R = cv::Mat::eye(3,3,CV_64F);
				cv::Mat t = cv::Mat::zeros(3,1,CV_64F);
				int key=0;
				cv::Point3d viewpoint(0.0,0.0,400.0);		// 視点位置
				cv::Point3d lookatpoint(0.0,0.0,0.0);	// 視線方向
				const double step = 50;

				// キーボード操作
				while(true)
				{
					//// 回転の更新
					double x=(lookatpoint.x-viewpoint.x);
					double y=(lookatpoint.y-viewpoint.y);
					double z=(lookatpoint.z-viewpoint.z);
					double pitch =asin(x/sqrt(x*x+z*z))/CV_PI*180.0;
					double yaw   =asin(-y/sqrt(y*y+z*z))/CV_PI*180.0;
					eular2rot(yaw, pitch, 0, R);
					// 移動の更新
					t.at<double>(0,0)=viewpoint.x;
					t.at<double>(1,0)=viewpoint.y;
					t.at<double>(2,0)=viewpoint.z;

					//プロジェクタ画素→3次元点
					calib.pointCloudRender(reconstructPoint, imagePoint, cam2, std::string("viewer"), R, t);
					//カメラ画素→3次元点
					//ノイズ付加
					//cv::Mat R_noise = (cv::Mat_<double>(3, 3) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1);
					//cv::Mat T_noise = (cv::Mat_<double>(3, 1) << 0.1, 0.1, 0.1);
					//cv::Mat eR = calib.R + R_noise;
					//cv::Mat eT = calib.T + T_noise;
					//calib.pointCloudRender(reconstructPoint_camera, projPoint_camera, cam2, std::string("viewer"), calib.R, calib.T);


					key = cv::waitKey(0);
					if(key=='w')
					{
						viewpoint.y+=step;
					}
					if(key=='s')
					{
						viewpoint.y-=step;
					}
					if(key=='a')
					{
						viewpoint.x+=step;
					}
					if(key=='d')
					{
						viewpoint.x-=step;
					}
					if(key=='z')
					{
						viewpoint.z+=step;
					}
					if(key=='x')
					{
						viewpoint.z-=step;
					}
					if(key=='q')
					{
						break;
					}
				}

			} else {
				std::cout << "キャリブレーションデータがありません" << std::endl;
			}

			break;


		case 'w':
			prjWhite = !prjWhite;
			break;

		default:
			exit(0);
			break;
		}
		printf("\n");
		cv::destroyAllWindows();
	}
}


// オイラー角を行列に変換
void eular2rot(double yaw,double pitch, double roll, cv::Mat& dest)
{
    double theta = yaw/180.0*CV_PI;
    double pusai = pitch/180.0*CV_PI;
    double phi = roll/180.0*CV_PI;
 
    double datax[3][3] = {{1.0,0.0,0.0}, 
    {0.0,cos(theta),-sin(theta)}, 
    {0.0,sin(theta),cos(theta)}};
    double datay[3][3] = {{cos(pusai),0.0,sin(pusai)}, 
    {0.0,1.0,0.0}, 
    {-sin(pusai),0.0,cos(pusai)}};
    double dataz[3][3] = {{cos(phi),-sin(phi),0.0}, 
    {sin(phi),cos(phi),0.0}, 
    {0.0,0.0,1.0}};

    cv::Mat Rx(3,3,CV_64F,datax);
    cv::Mat Ry(3,3,CV_64F,datay);
    cv::Mat Rz(3,3,CV_64F,dataz);
    cv::Mat rr=Rz*Rx*Ry;

    rr.copyTo(dest);
}