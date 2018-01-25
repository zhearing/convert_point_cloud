//#ifndef CONVERT_CONVERT_HPP_
//#define CONVERT_CONVERT_HPP_

#include <iostream>
#include <opencv2/opencv.hpp>

const int kDistance = 20; //grid size
const int kImgSizeHeight = 760; //rows .y
const int kImgSizeWidth = 500; //cols .x
const int bins_height = kImgSizeWidth / kDistance;
const int bins_width = kImgSizeHeight / kDistance;
int threshold = 10; //z threshold of point cloud  

cv::Point3f PointConvertCoordinate(float dpt_x, float dpt_y, float dpt_z);
void ComputeVote(cv::Point3f coord, int bin_max[bins_height][bins_width], int bin_min[bins_height][bins_width]);
void ZValueFilter(cv::Mat bins, int threshold, int bin_max[bins_height][bins_width], int bin_min[bins_height][bins_width]);
void SetPixelValue(cv::Mat bins);


int main(void)
{
	// TODO(zyzhong): use pcl::transformPoint() to rotate the coordinate system in ROS

	//Test Case
	float a[4][3] = { { 0, 0, 3.56f }, { 0, 2.49f, 1.56f }, { -1.2f, 0, 0.05f }, { -1.3f, -1.4323f, 1.56f } };

	cv::Mat bins(bins_height, bins_width, CV_32SC1, cv::Scalar(0));  //set as CV_16SC1, it will overflow
	int bin_max[bins_height][bins_width] = { 0 };
	int bin_min[bins_height][bins_width] = { threshold };

	for (int i = 0; i < 4; i++)
	{
		cv::Point3f coord;
		coord = PointConvertCoordinate(a[i][0], a[i][1], a[i][2]);  //for test case
		//coord = PointConvertCoordinate(dpt.x, dpt.y, dpt.z);  //for point cloud
		//coord = PointConvertCoordinate(0, 0, 3.56f);  //for single point
		ComputeVote(coord, bin_max, bin_min);
	}
	ZValueFilter(bins, threshold, bin_max, bin_min);
	SetPixelValue(bins);

	cvWaitKey(0);
    return 0;
}

cv::Point3f PointConvertCoordinate(float dpt_x, float dpt_y, float dpt_z)
{
	int half_img_size_x = kImgSizeWidth / 2;
	int half_img_size_y = kImgSizeHeight / 2;
	cv::Point3f coord;
	coord.x = 100 * dpt_x + half_img_size_x;
	coord.y = -100 * dpt_y + half_img_size_y;
	coord.z = 100 * dpt_z;
	//printf("%f %f %f \n", coord.x, coord.y, coord.z);
	return coord;
}

/**
* @brief Calculate the number of votes in each bin
*
*/
void ComputeVote(cv::Point3f coord, int bin_max[bins_height][bins_width], int bin_min[bins_height][bins_width])
{
	int bin_x = int(coord.x / kDistance);
	int bin_y = int(coord.y / kDistance);
	if (coord.z >= bin_max[bin_x][bin_y])
		bin_max[bin_x][bin_y] = int(coord.z);
	if (coord.z <  bin_min[bin_x][bin_y])
		bin_min[bin_x][bin_y] = int(coord.z);
	//printf("%d\n", bin_max[12][19]);
	//printf("%d\n", bin_min[12][19]);
}

void ZValueFilter(cv::Mat bins, int threshold, int bin_max[bins_height][bins_width], int bin_min[bins_height][bins_width])
{
	// TODO(zyzhong): Rewrite in Eigen or another way to speed up
	for (int i = 0; i < bins.rows; i++)
		for (int j = 0; j < bins.cols; j++)
		{
			//printf("%d %d\n", i, j);
			//printf("%d\n", bins.at<long int>(i, j));
			bins.at<int>(i, j) = bin_max[i][j] - bin_min[i][j];
			if (bins.at<int>(i, j) < threshold)
				bins.at<int>(i, j) = 0;
		}
	
}

void SetPixelValue(cv::Mat bins)
{
	cv::Mat grayimage(kImgSizeHeight, kImgSizeWidth, CV_8UC1, cv::Scalar(255));

	// TODO(zyzhong): Rewrite in Eigen or another way to speed up
	for (int i = 0; i < bins.rows; i++)
		for (int j = 0; j < bins.cols; j++)
		{
			if (bins.at<int>(i, j) != 0)
			{
				//printf("%d %d\n", i,j);
				for (int k = kDistance * i; k < kDistance * (i + 1); k++)
					for (int l = kDistance * j; l < kDistance * (j + 1); l++)
						grayimage.at<uchar>(l, k) = 0;  //important: exchange l and k

			}
		}

	cv::imshow("top_view", grayimage);
	cv::imwrite("E:\\top_view.jpg", grayimage);

}

//#endif // CONVERT_CONVERT_HPP_