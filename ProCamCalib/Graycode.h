#ifndef GRAYCODE_H
#define GRAYCODE_H

#pragma once

// GrayCodeを用いた幾何補正

// グレイコードは，縦，横別に作成し，後で合成
// パターン画像はビット演算を用いて作成（文字列char型は使わない）
// パターン画像は1枚ずつ書き込んで保存
#include <iostream>
#include <sstream>
#include <fstream>
#include <iomanip>  // 文字列ストリーム
#include <direct.h> // create dir
#include "Header.h"
#include "PGROpenCV.h"


extern TPGROpenCV pgrOpenCV;

/** 
@brief GrayCodeを用いた幾何補正<br>
対応付けられなかった画素には近傍の画素をコピーして補間<br>
最終的なプロジェクタとカメラを対応付けた配列はc->CamProに格納される。
*/
class GRAYCODE{
public:
	// プロジェクタ解像度
	static const int PRJ_WIDTH = PROJECTOR_WIDTH;
	static const int PRJ_HEIGHT = PROJECTOR_HEIGHT;
	static const int CMR_WIDTH = CAMERA_WIDTH;
	static const int CMR_HEIGHT = CAMERA_HEIGHT;
	static const int PRJ_X = DISPLAY_WIDTH;
	static const int PRJ_Y = DISPLAY_HEIGHT;

	// グレイコード作成に必要な構造体
	typedef struct _Graycode {
		int graycode[PRJ_HEIGHT][PRJ_WIDTH];  // グレイコード（プロジェクタ解像度[高さ][幅]）
		unsigned int h_bit, w_bit;     // 高さ，幅の必要ビット数
		unsigned int all_bit;          // 合計ビット数（h_bit + w_bit）
	} Graycode;

	// プロジェクタ - カメラ対応に必要な構造体
	typedef struct _correspondence {
		int graycode[CMR_HEIGHT][CMR_WIDTH];  // 2値化コードを復元したものをカメラ画素に格納
		cv::Point CamPro[PRJ_HEIGHT][PRJ_WIDTH];  // プロジェクタ画素に対するカメラ対応画素
		cv::Point ProCam[CMR_HEIGHT][CMR_WIDTH];  // カメラ画素に対するプロジェクタ対応画素
		std::map<int, cv::Point> *code_map;
		Graycode g;
	} correspondence;

	correspondence *c;

	GRAYCODE();
	~GRAYCODE();
	// パターンコード投影 & 撮影
	void code_projection();
	// 2値化
	void make_thresh();
	// 初期化
	void makeCorrespondence();

	//// 画像変形・処理
	//// カメラ撮影領域からプロジェクタ投影領域を切り出し
	void transport_camera_projector(cv::Mat &src, cv::Mat &dst);
	//// 入力画像をカメラ撮影領域に変形
	void transport_projector_camera(cv::Mat &src, cv::Mat &dst);

	// カメラ座標に対するプロジェクタの対応点を返す
	void getCorrespondProjPoints(std::vector<cv::Point2f> &projPoint, const std::vector<cv::Point2f> &imagePoint);
	// カメラ座標に対するプロジェクタの対応点を返す(高精度版)
	void getCorrespondSubPixelProjPoints(std::vector<cv::Point2f> &projPoint, const std::vector<cv::Point2f> &imagePoint, int size = 20);

	// 対応のとれた点を全て返す
	void getCorrespondAllPoints(std::vector<cv::Point2f> &projPoint, std::vector<cv::Point2f> &imagePoint, std::vector<int> &flag);

	void getCorrespondAllPoints_ProCam(std::vector<cv::Point2f> &projPoint, std::vector<cv::Point2f> &imagePoint, std::vector<int> &flag);


private:
	// ウィンドウネーム
	char* GC;
	char* MP;
	float SHUTTER;	// シャッター速度
	double delay;

	Graycode *g;

	/// グレイコードの作成関連
	// カメラの初期化
	void initCamera();
	// グレイコード作成
	void initGraycode();
	// パターンコード画像作成
	void makeGraycodeImage();
	// ディレクトリの作成
	void createDirs();
	/// 二値化関連
	// カメラ撮影画像を読み込む関数
	void loadCam(cv::Mat &mat, int div_bin, bool flag, bool pattern);
	// 最終的に使用するマスクを生成する関数
	void makeMask(cv::Mat &mask);
	// グレイコードの画像を利用してマスクを生成する関数
	// ポジとネガの差分を取ってMASK_THRESH以上の輝度のピクセルを白にする
	void makeMaskFromCam(cv::Mat &posi, cv::Mat &nega, cv::Mat &result, int thresholdValue = 25);
	// 2値化処理関数 
	void thresh( cv::Mat &posi, cv::Mat &nega, cv::Mat &thresh_img, int thresh_value );

	/// その他
	// プロジェクタ - カメラ構造体初期化
	void initCorrespondence();
	// 2値化コード復元
	void code_restore();
	//// 画素補間手法
	//// 右側の画素を持ってくる
	//cv::Point getInterpolatedPoint1(int y, int x, cv::Point CamPro[PRJ_HEIGHT][PRJ_WIDTH]);
	// 隣接する画素から持ってくる
	cv::Point getInterpolatedPoint2(int y, int x, cv::Point CamPro[PRJ_HEIGHT][PRJ_WIDTH]);
	// 画素補間
	void interpolation();

	//// 全画面表示
	//void displayFullScreen(cv::Mat &image);
};


#endif