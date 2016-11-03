#include "Graycode.h"

GRAYCODE::GRAYCODE()
{
	GC = "Graycode";
	MP = "Measure";
	delay = 150; //ここを小さくして、frameをいっぱい読み込む(新しいのを読む)と、グレイコード投影が早くなる
	g = new Graycode();
	c = new correspondence();
	c->code_map = new std::map<int, cv::Point>();
	// 構造体の初期化
	c->g.h_bit = (int)ceil( log(PRJ_HEIGHT+1) / log(2) );
	c->g.w_bit = (int)ceil( log(PRJ_WIDTH+1) / log(2) );
	c->g.all_bit = c->g.h_bit + c->g.w_bit;
	createDirs();
}

GRAYCODE::~GRAYCODE()
{
}

void GRAYCODE::createDirs()
{
	_mkdir("./GrayCodeImage");
	// グレイコード撮影画像
	_mkdir("./GrayCodeImage/CaptureImage");
	// グレイコード生画像
	_mkdir("./GrayCodeImage/ProjectionGrayCode");
	// グレイコード撮影画像の二値化した画像
	_mkdir("./GrayCodeImage/ThresholdImage");
}

/***************************
** グレイコードの作成関連 **
****************************/

// ビット数の計算とグレイコードの作成
void GRAYCODE::initGraycode()
{
	int bin_code_h[PRJ_HEIGHT];  // 2進コード（縦）
	int bin_code_w[PRJ_WIDTH];   // 2進コード（横）
	int graycode_h[PRJ_HEIGHT];  // グレイコード（縦）
	int graycode_w[PRJ_WIDTH];   // グレイコード（横）
	//int *graycode_h =  new int[c->g.h_bit];  // グレイコード（縦）
	//int *graycode_w =  new int[c->g.w_bit];  // グレイコード（横）

	/***** 2進コード作成 *****/
	// 行について
	for( int y = 0; y < PRJ_HEIGHT; y++ )
		bin_code_h[y] = y + 1;
	// 列について
	for( int x = 0; x < PRJ_WIDTH; x++ )
		bin_code_w[x] = x + 1;

	/***** グレイコード作成 *****/
	// 行について
	for( int y = 0; y < PRJ_HEIGHT; y++ )
		graycode_h[y] = bin_code_h[y] ^ ( bin_code_h[y] >> 1 );
	// 列について
	for( int x = 0; x < PRJ_WIDTH; x++ )
		graycode_w[x] = bin_code_w[x] ^ ( bin_code_w[x] >> 1 );
	// 行列を合わせる（行 + 列）
	for( int y = 0; y < PRJ_HEIGHT; y++ ) {
		for( int x = 0; x < PRJ_WIDTH; x++ )
			c->g.graycode[y][x] = ( graycode_h[y] << c->g.w_bit) | graycode_w[x];
	}
}

// パターンコード画像作成（一度作ればプロジェクタの解像度が変わらない限り作り直す必要はない）
void GRAYCODE::makeGraycodeImage()
{
	std::cout << "投影用グレイコード作成中" << std::endl;
	//initGraycode();
	cv::Mat posi_img ( PRJ_HEIGHT, PRJ_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0) );
	cv::Mat nega_img ( PRJ_HEIGHT, PRJ_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0) );
	int bit = c->g.all_bit-1;
	std::stringstream *Filename_posi = new std::stringstream[c->g.all_bit];  // 書式付入出力
	std::stringstream *Filename_nega = new std::stringstream[c->g.all_bit];  // 書式付入出力

	// ポジパターンコード画像作成
	for( unsigned int z = 0; z < c->g.all_bit; z++) {
		for( int y = 0; y < PRJ_HEIGHT; y++ ) {
			for( int x = 0; x < PRJ_WIDTH; x++ ) {
				if( ( (c->g.graycode[y][x] >> (bit-z)) & 1 ) == 0 ) {  // 最上位ビットから順に抽出し，そのビットが0だった時
					posi_img.at<cv::Vec3b>( y, x )[0] = 0;  // B
					posi_img.at<cv::Vec3b>( y, x )[1] = 0;  // G
					posi_img.at<cv::Vec3b>( y, x )[2] = 0;  // R
				}else if( ( (c->g.graycode[y][x] >> (bit-z)) & 1 ) == 1 ) {
					posi_img.at<cv::Vec3b>( y, x )[0] = 255;  // B
					posi_img.at<cv::Vec3b>( y, x )[1] = 255;  // G
					posi_img.at<cv::Vec3b>( y, x )[2] = 255;  // R
				}
			}
		}
		// 連番でファイル名を保存（文字列ストリーム）
		Filename_posi[z] << "./GrayCodeImage/ProjectionGrayCode/posi" << std::setw(2) << std::setfill('0') << z << ".bmp"; 
		cv::imwrite(Filename_posi[z].str(), posi_img);
		Filename_posi[z] << std::endl;
	}

	// ネガパターンコード画像作成
	for( unsigned int z = 0; z < c->g.all_bit; z++) {
		for( int y = 0; y < PRJ_HEIGHT; y++ ) {
			for( int x = 0; x < PRJ_WIDTH; x++ ) {
				if( ( (c->g.graycode[y][x] >> (bit-z)) & 1 ) == 1 ) {
					nega_img.at<cv::Vec3b>( y, x )[0] = 0;  // B
					nega_img.at<cv::Vec3b>( y, x )[1] = 0;  // G
					nega_img.at<cv::Vec3b>( y, x )[2] = 0;  // R
				}else if( ( (c->g.graycode[y][x] >> (bit-z)) & 1 ) == 0 ) {
					nega_img.at<cv::Vec3b>( y, x )[0] = 255;  // B
					nega_img.at<cv::Vec3b>( y, x )[1] = 255;  // G
					nega_img.at<cv::Vec3b>( y, x )[2] = 255;  // R
				}
			}
		}
		// 連番でファイル名を保存（文字列ストリーム）
		Filename_nega[z] << "./GrayCodeImage/ProjectionGrayCode/nega" << std::setw(2) << std::setfill('0') << z << ".bmp"; 
		cv::imwrite(Filename_nega[z].str(), nega_img);
		Filename_nega[z] << std::endl;
	}

	delete[] Filename_posi;
	delete[] Filename_nega;
}

// パターンコード投影 & 撮影
void GRAYCODE::code_projection()
{
	// 定数
	typedef enum flag{
		POSI = true,
		NEGA = false,
		VERTICAL = true,
		HORIZONTAL = false,
	} flag;

	Graycode *g = new Graycode();
	//TPGROpenCV	pgrOpenCV;

	//初期化&カメラ起動
	initGraycode();
	//pgrOpenCV.init( FlyCapture2::PIXEL_FORMAT_BGR );
	//pgrOpenCV.setShutterSpeed(pgrOpenCV.getShutter_h());
	pgrOpenCV.start();

	cv::Mat *posi_img = new cv::Mat[c->g.all_bit];  // ポジパターン用
	cv::Mat *nega_img = new cv::Mat[c->g.all_bit];  // ネガパターン用

	// 書式付入出力（グレイコード読み込み用）
	std::stringstream *Filename_posi = new std::stringstream[c->g.all_bit]; 
	std::stringstream *Filename_nega = new std::stringstream[c->g.all_bit];
	// 書式付入出力（撮影画像書き込み用）
	std::stringstream *Filename_posi_cam = new std::stringstream[c->g.all_bit]; 
	std::stringstream *Filename_nega_cam = new std::stringstream[c->g.all_bit];

	// 連番でファイル名を読み込む（文字列ストリーム）
	std::cout << "投影用グレイコード画像読み込み中" << std::endl;
	for( unsigned int i = 0; i < c->g.all_bit; i++ ) {
		Filename_posi[i] << "./GrayCodeImage/ProjectionGrayCode/posi" << std::setw(2) << std::setfill('0') << i << ".bmp";
		Filename_nega[i] << "./GrayCodeImage/ProjectionGrayCode/nega" << std::setw(2) << std::setfill('0') << i << ".bmp";
		// 読み込み
		posi_img[i] = cv::imread(Filename_posi[i].str(), 1);
		nega_img[i] = cv::imread(Filename_nega[i].str(), 1);
		Filename_posi[i] << std::endl;
		Filename_nega[i] << std::endl;
		// 読み込む枚数が足りなかったらグレイコード画像を作り直す
		if(posi_img[i].empty() || nega_img[i].empty()){
			std::cout << "ERROR(1)：投影用のグレイコード画像が不足しています。" << std::endl;
			std::cout << "ERROR(2)：グレイコード画像を作成します。" << std::endl;
			makeGraycodeImage();
			code_projection();
			return;
		}
	}

	/***** グレイコード投影 & 撮影 *****/
	/*  全画面表示用ウィンドウの作成  */
	cv::namedWindow(GC, 0);
	Projection::MySetFullScrean(DISPLAY_NUMBER, GC);

	// ポジパターン投影 & 撮影
	//start capturing

	std::cout << "ポジパターン撮影中" << std::endl;
	for( unsigned int i = 0; i < c->g.all_bit; i++ ) {
		// 投影
		cv::imshow(GC, posi_img[i]);
		// 遅延待ち
		cv::waitKey(2.0*delay);
		// 撮影
		pgrOpenCV.queryFrame();
		// 撮影画像をMat型に格納
		cv::Mat cap = pgrOpenCV.getVideo();
		// 撮影の様子をチェック
		pgrOpenCV.showCapImg(cap);

		// ポジパターン撮影結果を保存
		// 横縞
		if(i < c->g.h_bit)
			Filename_posi_cam[i] << "./GrayCodeImage/CaptureImage/CameraImg" << HORIZONTAL << "_" << std::setw(2) << std::setfill('0') << i+1 << "_" << POSI << ".bmp"; 
		// 縦縞
		else
			Filename_posi_cam[i] << "./GrayCodeImage/CaptureImage/CameraImg" << VERTICAL << "_" << std::setw(2) << std::setfill('0') << i-c->g.h_bit+1 << "_" << POSI << ".bmp"; 
		// 保存
		cv::imwrite(Filename_posi_cam[i].str(), pgrOpenCV.getVideo());
		Filename_posi_cam[i] << std::endl;
	}

	// ネガパターン投影 & 撮影
	std::cout << "ネガパターン撮影中" << std::endl;
	for( unsigned int i = 0; i < c->g.all_bit; i++ ) {
		// 投影
		cv::imshow(GC, nega_img[i]);
		// 遅延待ち
		cv::waitKey(2*delay);
		// 撮影
		pgrOpenCV.queryFrame();
		// 撮影画像をMat型に格納
		cv::Mat cap = pgrOpenCV.getVideo();
		// 撮影の様子をチェック
		pgrOpenCV.showCapImg(cap);
		// ポジパターン撮影結果を保持
		// 横縞
		if(i < c->g.h_bit)
			Filename_nega_cam[i] << "./GrayCodeImage/CaptureImage/CameraImg" << HORIZONTAL << "_" << std::setw(2) << std::setfill('0') << i+1 << "_" << NEGA << ".bmp"; 
		// 縦縞
		else
			Filename_nega_cam[i] << "./GrayCodeImage/CaptureImage/CameraImg" << VERTICAL << "_" << std::setw(2) << std::setfill('0') << i-c->g.h_bit+1 << "_" << NEGA << ".bmp"; 
		//Filename_nega_cam[i] << "./output/Camera_nega" << std::setw(2) << std::setfill('0') << i << ".bmp";
		cv::imwrite(Filename_nega_cam[i].str(), pgrOpenCV.getVideo());
		Filename_nega_cam[i] << std::endl;
	}
	/***** 投影 & 撮影終了 *****/


	cv::Mat src = cv::Mat(PROJECTOR_WIDTH, PROJECTOR_HEIGHT, CV_8UC3, cv::Scalar(255, 255, 255));
	cv::imshow(GC, src);
	cv::waitKey(2*delay);
	pgrOpenCV.queryFrame();
	cv::imwrite("./cap.jpg", pgrOpenCV.getVideo());

	// カメラ終了処理
	pgrOpenCV.stop();

	/**** 終了 *****/

	// メモリの開放
	delete[] posi_img;
	delete[] nega_img;
	delete[] Filename_posi;
	delete[] Filename_nega;
	delete[] Filename_posi_cam;
	delete[] Filename_nega_cam;
}


/***************
** 二値化関連 **
****************/

// カメラ撮影画像を読み込む関数
void GRAYCODE::loadCam(cv::Mat &mat, int div_bin, bool vh, bool pn)
{
	char buf[256];
	sprintf_s(buf, "./GrayCodeImage/CaptureImage/CameraImg%d_%02d_%d.bmp", vh, div_bin, pn);
	mat = cv::imread(buf, 0);
}

// マスクを作成するインタフェース
void GRAYCODE::makeMask(cv::Mat &mask)
{
	cv::Mat posi_img;
	cv::Mat nega_img;

	// マスク画像生成
	cv::Mat mask_vert, mask_hor;
	static int useImageNumber = 6;
	// y方向のグレイコード画像読み込み
	loadCam(posi_img, useImageNumber, 0, 1);
	loadCam(nega_img, useImageNumber, 0, 0);

	// 仮のマスク画像Y生成
	makeMaskFromCam(posi_img, nega_img, mask_vert);

	// x方向のグレイコード画像読み込み
	loadCam(posi_img, useImageNumber, 1, 1);
	loadCam(nega_img, useImageNumber, 1, 0);

	// 仮のマスク画像X生成
	makeMaskFromCam(posi_img, nega_img, mask_hor);

	// XとYのORを取る
	// マスク外はどちらも黒なので黒
	// マスク内は（理論的には）必ず一方が白でもう一方が黒なので、白になる
	// 実際はごま塩ノイズが残ってしまう
	cv::bitwise_or(mask_vert, mask_hor, mask);

	// 残ったごま塩ノイズを除去（白ゴマか黒ゴマかで適用順が逆になる）
	dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 5);
	erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 5);

	cv::imwrite("./GrayCodeImage/mask.bmp", mask);
}

// グレイコードの画像を利用してマスクを生成する関数
// ポジとネガの差分を取ってthresholdValue以上の輝度のピクセルを白にする
void GRAYCODE::makeMaskFromCam(cv::Mat &posi, cv::Mat &nega, cv::Mat &result, int thresholdValue)
{
	result = cv::Mat::zeros(cv::Size(CMR_WIDTH,CMR_HEIGHT), CV_8UC1);

	for(int j=0; j<CMR_HEIGHT; j++){
		for(int i=0; i<CMR_WIDTH; i++){
			int posi_i = posi.at<uchar>(j, i);
			int nega_i = nega.at<uchar>(j, i);

			if (abs(posi_i - nega_i) > thresholdValue){
				result.at<uchar>(j, i) = 255;
			}else{
				result.at<uchar>(j, i) = 0;
			}
		}
	}
}

// 撮影画像の2値化をするインタフェース
void GRAYCODE::make_thresh()
{
	cv::Mat posi_img;
	cv::Mat nega_img;
	cv::Mat Geometric_thresh_img;  // 2値化された画像
	cv::Mat mask;

	// マスクを生成
	makeMask(mask);

	int h_bit = (int)ceil( log(PRJ_HEIGHT+1) / log(2) );
	int w_bit = (int)ceil( log(PRJ_WIDTH+1) / log(2) );
	int all_bit = h_bit + w_bit;

	std::cout << "二値化開始" << std::endl;
	// 連番でファイル名を読み込む
	for( int i = 0; i < h_bit; i++ ) {
		// 読み込み
		char buf[256];
		// ポジパターン読み込み
		loadCam(posi_img, i+1, 0, 1);
		// ネガパターン読み込み
		loadCam(nega_img, i+1, 0, 0);

		// 2値化
		cv::Mat masked_img;
		thresh( posi_img, nega_img, Geometric_thresh_img, 0 );
		// マスクを適用して2値化
		Geometric_thresh_img.copyTo( masked_img, mask );
		sprintf_s(buf, "./GrayCodeImage/ThresholdImage/Geometric_thresh%02d.bmp", i);
		cv::imwrite(buf, masked_img);

		std::cout << i << ", ";
	}
	for( int i = 0; i < w_bit; i++ ) {
		// 読み込み
		char buf[256];
		// ポジパターン読み込み
		loadCam(posi_img, i+1, 1, 1);
		// ネガパターン読み込み
		loadCam(nega_img, i+1, 1, 0);

		// 2値化
		cv::Mat masked_img;
		thresh( posi_img, nega_img, Geometric_thresh_img, 0 );
		// マスクを適用して2値化
		Geometric_thresh_img.copyTo( masked_img, mask );
		sprintf_s(buf, "./GrayCodeImage/ThresholdImage/Geometric_thresh%02d.bmp", i+h_bit);
		cv::imwrite(buf, masked_img);

		std::cout << i+h_bit << ", ";
	}
	std::cout << std::endl;
	std::cout << "二値化終了" << std::endl;
}

// 実際の2値化処理 
void GRAYCODE::thresh( cv::Mat &posi, cv::Mat &nega, cv::Mat &thresh_img, int thresh_value )
{
	thresh_img = cv::Mat(posi.rows, posi.cols, CV_8UC1);
	for( int y = 0; y < posi.rows; y++ ) {
		for(int x = 0; x < posi.cols; x++ ) {
			int posi_pixel = posi.at<uchar>( y, x );
			int nega_pixel = nega.at<uchar>( y, x );

			// thresh_valueより大きいかどうかで二値化
			if( posi_pixel - nega_pixel >= thresh_value )
				thresh_img.at<uchar>( y, x ) = 255;
			else
				thresh_img.at<uchar>( y, x ) = 0;
		}
	}
}

/***********************************
** プロジェクタとカメラの対応付け **
************************************/

// 2値化コード復元
void GRAYCODE::code_restore()
{
	// 2値化コード復元
	for( unsigned int i = 0; i < c->g.all_bit; i++ ) {
		char buf[256];
		sprintf_s(buf, "./GrayCodeImage/ThresholdImage/Geometric_thresh%02d.bmp", i);
		cv::Mat a = cv::imread(buf, 0);

		for( int y = 0; y < CMR_HEIGHT; y++ ) {
			for( int x = 0; x < CMR_WIDTH; x++ ) {
				if( a.at<uchar>( y, x ) == 255)
					c->graycode[y][x] = ( 1 << (c->g.all_bit-i-1) ) | c->graycode[y][x]; 
			}
		}
	}

	c->code_map->clear();

	// 連想配列でグレイコードの値の場所に座標を格納
	for( int y = 0; y < CMR_HEIGHT; y++ ) {
		for( int x = 0; x < CMR_WIDTH; x++ ) {
			int a = c->graycode[y][x];
			if( a != 0 )
				(*c->code_map)[a] = cv::Point(x, y);

			// 初期化
			c->ProCam[y][x] = cv::Point(-1, -1);
		}
	}

	// 0番目は使わない
	(*c->code_map)[0] = cv::Point(-1, -1);

	// プロジェクタとカメラの対応付け
	for( int y = 0; y < PRJ_HEIGHT; y++ ) {
		for( int x = 0; x < PRJ_WIDTH; x++ ) {
			// グレイコード取得
			int a = c->g.graycode[y][x];
			// map内に存在しないコード（カメラで撮影が上手くいかなかった部分）の場所にはエラー値-1を格納
			if ( (*c->code_map).find(a) == (*c->code_map).end() ) {
				c->CamPro[y][x] = cv::Point(-1, -1);
			}
			// 存在する場合は、対応するグレイコードの座標を格納
			else {
				c->CamPro[y][x] = (*c->code_map)[a];
				c->ProCam[(*c->code_map)[a].y][(*c->code_map)[a].x] = cv::Point(x, y);
			}
		}
	}
}

// 画素補間手法
// 右側の画素を持ってくる
//cv::Point GRAYCODE::getInterpolatedPoint1(int y, int x, cv::Point CamPro[PRJ_HEIGHT][PRJ_WIDTH])
//{
//	for (int i = x + 1; i < PRJ_WIDTH; i++) {
//		if (CamPro[y][i].x != -1) {
//			return CamPro[y][i];
//		}
//	}
//
//	return cv::Point(-1, -1);
//}

// 隣接する画素から持ってくる
cv::Point GRAYCODE::getInterpolatedPoint2(int y, int x, cv::Point CamPro[PRJ_HEIGHT][PRJ_WIDTH])
{
	const int MAX_RADIUS = 100;

	for (int radius = 1; radius <= MAX_RADIUS; radius++){

		for (int j = -radius; j <= radius; j++) {
			for (int i = -radius; i <= radius; i++) {
				int yj = j + y;
				int xi = i + x;

				if (0 <= yj && yj < PRJ_HEIGHT && 0 <= xi && xi < PRJ_WIDTH) {
					if ((yj > y) || (yj == y && xi > x)) {
						if (c->CamPro[yj][xi].x != -1) {
							return c->CamPro[yj][xi];
						}
					}
				}
			}
		}

	}

	return cv::Point(-1, -1);
}

// 画素補間
void GRAYCODE::interpolation()
{
	for (int y = 0; y < PRJ_HEIGHT; y++) {
		for (int x = 0; x < PRJ_WIDTH; x++) {
			if (c->CamPro[y][x].x == -1) {
				c->CamPro[y][x] = getInterpolatedPoint2(y, x, c->CamPro);
			}
		}
	}
}

// プロジェクタ - カメラ構造体初期化
void GRAYCODE::initCorrespondence()
{
	initGraycode();

	for( int y = 0; y < CMR_HEIGHT; y++ ) {
		for( int x = 0; x < CMR_WIDTH; x++ ){
			c->graycode[y][x] = 0;
		}
	}
}

// 対応付けを行うインターフェース
void GRAYCODE::makeCorrespondence()
{
	initCorrespondence();
	code_restore();
	//	補完処理on off
	//interpolation();
}


/***********************************
** その他（用途不明な過去の遺物） **
************************************/

// 画像変形・処理
// カメラ撮影領域からプロジェクタ投影領域を切り出し
void GRAYCODE::transport_camera_projector(cv::Mat &src, cv::Mat &dst)
{
	cv::Mat src_resize;  // リサイズした画像
	resize( src, src_resize, cv::Size(CMR_WIDTH, CMR_HEIGHT) );

	dst = cv::Mat( PRJ_HEIGHT, PRJ_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0) );  // 幾何補正された画像（投影画像）

	for( int y = 0; y < PRJ_HEIGHT; y++ ) {
		for( int x = 0; x < PRJ_WIDTH; x++ ) {
			cv::Point p = c->CamPro[y][x];
			if( p.x != -1 ) {
				if(src_resize.at<uchar>( p.y, p.x ) != 0){
				//if(src_resize.at<uchar>( p.y, 3*p.x ) != 0 && src_resize.at<uchar>( p.y, 3*p.x+1 ) != 0 && src_resize.at<uchar>( p.y, 3*p.x+2 ) != 0){
					//printf("x:%d, y:%d, p.x:%d, p.y:%d\n", x, y, p.x, p.y);
					//dst.at<uchar>( y, 3*x ) = src_resize.at<uchar>( p.y, 3*p.x );      // B
					//dst.at<uchar>( y, 3*x+1 ) = src_resize.at<uchar>( p.y, 3*p.x+1 );  // G
					//dst.at<uchar>( y, 3*x+2 ) = src_resize.at<uchar>( p.y, 3*p.x+2 );  // R
					//GRAY
					dst.at<uchar>( y, 3*x ) = src_resize.at<uchar>( p.y, p.x );   // B
					dst.at<uchar>( y, 3*x+1 ) = src_resize.at<uchar>( p.y, p.x);  // G
					dst.at<uchar>( y, 3*x+2 ) = src_resize.at<uchar>( p.y, p.x);  // R
				}
			}
		}
	}
}

// 入力画像をカメラ撮影領域に変形
void GRAYCODE::transport_projector_camera(cv::Mat &src, cv::Mat &dst)
{
	cv::Mat src_resize;  // リサイズした画像
	resize( src, src_resize, cv::Size(PRJ_WIDTH, PRJ_HEIGHT) );

	dst = cv::Mat( CMR_HEIGHT, CMR_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0) );  // 幾何補正された画像（投影画像）

	for( int y = 0; y < PRJ_HEIGHT; y++ ) {
		for( int x = 0; x < PRJ_WIDTH; x++ ) {
			cv::Point p = c->CamPro[y][x];
			if( p.x != -1 ) {
				//printf("x:%d, y:%d, p.x:%d, p.y:%d\n", x, y, p.x, p.y);
				dst.at<uchar>( p.y, 3*p.x ) = src_resize.at<uchar>( y, 3*x );      // B
				dst.at<uchar>( p.y, 3*p.x+1 ) = src_resize.at<uchar>( y, 3*x+1 );  // G
				dst.at<uchar>( p.y, 3*p.x+2 ) = src_resize.at<uchar>( y, 3*x+2 );  // R
			}
		}
	}
}
//
//// 全画面表示
//void GRAYCODE::displayFullScreen(cv::Mat &image)
//{
//	/*  全画面表示用ウィンドウの作成  */
//	cv::namedWindow(GC, 0);
//	HWND windowHandle = ::FindWindowA(NULL, GC);
//	// ウィンドウスタイル変更（メニューバーなし、最前面)
//	SetWindowLongPtr(windowHandle,  GWL_STYLE, WS_POPUP);
//	SetWindowLongPtr(windowHandle, GWL_EXSTYLE, WS_EX_TOPMOST);
//	// 最大化する
//	ShowWindow(windowHandle, SW_MAXIMIZE);
//	cv::setWindowProperty(GC, CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN );
//	// ディスプレイサイズを取得
//	//int mainDisplayWidth = GetSystemMetrics(SM_CXSCREEN);
//	//int mainDisplayHeight = GetSystemMetrics(SM_CYSCREEN);
//	// クライアント領域をディスプレイに合わせる
//	SetWindowPos(windowHandle, NULL,
//		PRJ_X, PRJ_Y, PRJ_WIDTH, PRJ_HEIGHT, SWP_FRAMECHANGED | SWP_NOZORDER);
//
//	cv::imshow(GC, image);
//	cv::waitKey(0);
//}



/***********************************
** 追加 **
************************************/


// カメラ座標に対するプロジェクタの対応点を返す
void GRAYCODE::getCorrespondProjPoints(std::vector<cv::Point2f> &projPoint, const std::vector<cv::Point2f> &imagePoint)
{
	for (int i=0; i < imagePoint.size(); ++i)
	{
		cv::Point2f point = c->ProCam[int(imagePoint[i].y+0.5f)][int(imagePoint[i].x+0.5f)];

		if( point.x != -1.0f) {
			projPoint.emplace_back(point);
		}
	}
}


// カメラ座標に対するプロジェクタの対応点を返す(高精度版)
void GRAYCODE::getCorrespondSubPixelProjPoints(std::vector<cv::Point2f> &projPoint, const std::vector<cv::Point2f> &imagePoint, int size)
{
	for (int i=0; i < imagePoint.size(); ++i)
	{
		std::vector<cv::Point2f> iPoints, pPoints;
		if(imagePoint[i].x > size && imagePoint[i].x+size < CAMERA_WIDTH && imagePoint[i].y > size && imagePoint[i].y+size < CAMERA_HEIGHT)
		{
			// 領域毎の対応点
			for( float h = imagePoint[i].y-size; h < imagePoint[i].y+size; h+=1.0f){
				for( float w = imagePoint[i].x-size; w < imagePoint[i].x+size; w+=1.0f){
					cv::Point2f point = c->ProCam[int(h+0.5f)][int(w+0.5f)];
					if( point.x != -1.0f) {
						iPoints.emplace_back(cv::Point2f(w, h));
						pPoints.emplace_back(point);
					}
				}
			}

			// 対応点同士でHomographyの計算
			cv::Mat H = cv::findHomography(iPoints, pPoints, CV_RANSAC, 2.0);
			// Homographyを使ってチェッカーパターンの交点を射影
			cv::Point3d Q = cv::Point3d(cv::Mat(H * cv::Mat(cv::Point3d(imagePoint[i].x,imagePoint[i].y,1.0))));
			projPoint.emplace_back(cv::Point2f(Q.x/Q.z, Q.y/Q.z));
		}
		else
		{
			cv::Point2f point = c->ProCam[int(imagePoint[i].y+0.5f)][int(imagePoint[i].x+0.5f)];

			if( point.x != -1.0f) {
				projPoint.emplace_back(point);
			}
		}
	}
}


// 対応のとれた点を全て返す
void GRAYCODE::getCorrespondAllPoints(std::vector<cv::Point2f> &projPoint, std::vector<cv::Point2f> &imagePoint, std::vector<int> &flag) 
{
	for( int y = 0; y < PRJ_HEIGHT; y++ ) {
		for( int x = 0; x < PRJ_WIDTH; x++ ) {
			cv::Point p = c->CamPro[y][x];
			if( p.x != -1 ) {
				flag.emplace_back(1);
				projPoint.emplace_back(cv::Point2f(x, y));
				imagePoint.emplace_back(cv::Point2f(p.x, p.y));
			}
			else
			{
				flag.emplace_back(0);
				projPoint.emplace_back(cv::Point2f(-1, -1));
				imagePoint.emplace_back(cv::Point2f(-1, -1));
			}
		}
	}
}

void GRAYCODE::getCorrespondAllPoints_ProCam(std::vector<cv::Point2f> &projPoint, std::vector<cv::Point2f> &imagePoint, std::vector<int> &flag) 
{
	for( int y = 0; y < CAMERA_HEIGHT; y++ ) {
		for( int x = 0; x < CAMERA_WIDTH; x++ ) {
			cv::Point p = c->ProCam[y][x];
			if( p.x != -1 ) {
				flag.emplace_back(1);
				projPoint.emplace_back(cv::Point2f(p.x, p.y));
				imagePoint.emplace_back(cv::Point2f(x, y));
			}
			else
			{
				flag.emplace_back(0);
				projPoint.emplace_back(cv::Point2f(-1, -1));
				imagePoint.emplace_back(cv::Point2f(-1, -1));
			}
		}
	}
}