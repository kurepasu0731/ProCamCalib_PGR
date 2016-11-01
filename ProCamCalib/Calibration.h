#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <opencv2\opencv.hpp>


class Calibration
{
public:
	Calibration(int _checkerRow, int _checkerCol, float _checkerSize)
		: checkerPattern (cv::Size(_checkerRow, _checkerCol))
		, checkerSize (_checkerSize)
		, calib_flag (false)
	{
		// 世界座標におけるチェッカーパターンの交点座標を決定
		for( int i = 0; i < checkerPattern.area(); ++i ) {
			worldPoint.push_back( cv::Point3f(	static_cast<float>( i % checkerPattern.width * checkerSize ),
													static_cast<float>( i / checkerPattern.width * checkerSize ), 0.0 ) );
		}
	};

	~Calibration(){};


	// 画像からチェッカーパターンの交点を取得
	bool getCheckerCorners(std::vector<cv::Point2f> &imagePoint, const cv::Mat &image, cv::Mat &draw_image);		// 画像からチェッカーパターンの交点を取得

	// 再投影誤差の計算
	void calcReprojectionError(const std::vector<std::vector<cv::Point3f>> &worldPoints, const std::vector<std::vector<cv::Point2f>> &cameraPoints, const std::vector<std::vector<cv::Point2f>> &projectorPoints,double &cam_error, double &proj_error);

	// プロジェクタとカメラのキャリブレーション
	void proCamCalibration(const std::vector<std::vector<cv::Point3f>> &worldPoints, const std::vector<std::vector<cv::Point2f>> &cameraPoints, const std::vector<std::vector<cv::Point2f>> &projectorPoints,
							const cv::Size &camSize, const cv::Size &projSize);

	// キャリブレーション結果の読み込み
	void loadCalibParam(const std::string &fileName);

	// 透視投影変換行列の取得(カメラ)
	cv::Mat getCamPerspectiveMat();
	// 透視投影変換行列の取得(プロジェクタ)
	cv::Mat getProjPerspectiveMat();

	// カメラ位置をワールド座標とした際の対象物体の位置の取得
	void getCameraWorldPoint(std::vector<cv::Point3f> &camWorldPoint, const std::vector<cv::Point2f> &imagePoint);

	// 3次元復元
	void reconstruction(std::vector<cv::Point3f> &reconstructPoint, const std::vector<cv::Point2f> &projPoint, const std::vector<cv::Point2f> &imagePoint, const std::vector<int> &flag);

	// 3次元点群の描画
	void pointCloudRender(const std::vector<cv::Point3f> &reconstructPoint, const std::vector<cv::Point2f> &imagePoint, const cv::Mat &image, std::string &windowName, const cv::Mat& R, const cv::Mat& t);

	/***** メンバ変数 *****/
	cv::Size checkerPattern;		// チェッカーパターンの交点の数
	float checkerSize;				// チェッカーパターンのマス目のサイズ(mm)

	std::vector<cv::Point3f> worldPoint;		// チェッカー交点座標と対応する世界座標の値を格納する行列

	// カメラ
	cv::Mat cam_K;					// 内部パラメータ行列
	cv::Mat cam_dist;				// レンズ歪み
	std::vector<cv::Mat> cam_R;		// 回転ベクトル
	std::vector<cv::Mat> cam_T;		// 平行移動ベクトル

	// プロジェクタ
	cv::Mat proj_K;					// 内部パラメータ行列
	cv::Mat proj_dist;				// レンズ歪み
	std::vector<cv::Mat> proj_R;	// 回転ベクトル
	std::vector<cv::Mat> proj_T;	// 平行移動ベクトル

	// ステレオパラメータ
	cv::Mat R;						// カメラ-プロジェクタ間の回転行列
	cv::Mat T;						// カメラ-プロジェクタ間の並進ベクトル
	cv::Mat E;						// 基本行列
	cv::Mat F;						// 基礎行列

	// フラグ
	bool calib_flag;
};


#endif