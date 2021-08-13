#include <strategy/edge_detection.h>

void Edge_detection::Catch_image(const sensor_msgs::ImageConstPtr& msg)
{
    if(!checkRealImage)
    {
        cv_bridge::CvImagePtr cv_ptr;
        checkImageSource = true;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        orign_img = cv_ptr->image;
    }
    else
    {
        ROS_ERROR("Please Close Real Image and restart the node.");
    }
}

string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}




int main(int argc, char** argv)
{
    ros::init(argc, argv, "Edge_detection");
    ros::NodeHandle nh;
    Edge_detection Edge_detection(nh);

    ros::Rate loop_rate(30);

    while(nh.ok())
    {
        Edge_detection.strategymain();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

void Edge_detection::strategymain()
{
    if(!orign_img.empty() && checkImageSource)
    {
        orign_img = imread("/home/ching/git/test.png");
        imshow("view", orign_img);
        waitKey(30);
        Canny(orign_img, edge, 50, 150, 3);
        imshow("edge", edge);
        waitKey(30);
        cvtColor(edge, frame, cv::COLOR_GRAY2BGR);
        edgeimage_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        edgeimage_Publisher.publish(edgeimage_msg);
        ROS_INFO("width %d , height %d", edge.rows, edge.cols);

        string ty =  type2str( edge.type() );
        printf("Matrix: %s %dx%d \n", ty.c_str(), edge.cols, edge.rows );
        for(int i = 0; i<edge.cols/2; i++)
        {
            for(int j = 0; j<edge.rows/2; j++)
            {
                printf("%d", (bool)frame.at<uchar>(i,j));
            }
            printf("\n");
        }
        printf("\n\n\n");
        // vector<Vec4f> plines;//保存霍夫变换检测到的直线
        // Mat dst;
        // cvtColor(edge, dst, CV_GRAY2BGR);
        // // HoughLinesP(edge, plines, 1, CV_PI / 180, 10, 0, 10);//提取边缘时，会造成有些点不连续，所以maxLineGap设大点
        // HoughLines(edge, plines, 1, CV_PI/180, 100, 0, 0 );
        // //5. 显示检测到的直线
        // Scalar color = Scalar(0, 0, 255);//设置颜色
        // for (size_t i = 0; i < plines.size(); i++)
        // {
        //     Vec4f hline = plines[i];
        //     line(dst, Point(hline[0], hline[1]), Point(hline[2], hline[3]), color, 3, LINE_AA);//绘制直线
        // }
        // imshow("plines", dst);
        waitKey(30);

//         Mat dst, cdst;
//  Canny(orign_img, dst, 50, 200, 3);//用Canny運算元對影象進行邊緣檢測
//  cvtColor(dst, cdst, CV_GRAY2BGR);

//  #if 0
//   vector<Vec2f> lines;
//   HoughLines(dst, lines, 1, CV_PI/180, 100, 0, 0 );

//   for( size_t i = 0; i < lines.size(); i++ )
//   {
//      float rho = lines[i][0], theta = lines[i][1];
    //  Point pt1, pt2;
    //  double a = cos(theta), b = sin(theta);
    //  double x0 = a*rho, y0 = b*rho;
    //  pt1.x = cvRound(x0 + 1000*(-b));
    //  pt1.y = cvRound(y0 + 1000*(a));
    //  pt2.x = cvRound(x0 - 1000*(-b));
    //  pt2.y = cvRound(y0 - 1000*(a));
    //  line( cdst, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
//   }
//  #else
//   vector<Vec4i> lines;//儲存著檢測到的直線的引數對 (X1, Y1, X2, Y2) 的容器，也就是線段的兩個端點
//   HoughLinesP(dst, lines, 1, CV_PI/180, 50, 50, 10 );
//   //dst:邊緣檢測的輸出影象. 它應該是個灰度圖,作為我們的輸入影象
//   //１：引數極徑 r 以畫素值為單位的解析度. 我們使用 1 畫素.
//   //CV_PI/180:引數極角 \theta 以弧度為單位的解析度. 我們使用 1度 (即CV_PI/180)
//   //50:要”檢測” 一條直線所需最少的的曲線交點
//   //50:能組成一條直線的最少點的數量. 點數量不足的直線將被拋棄.
//   //10:線段上最近兩點之間的閾值,也就是能被認為在一條直線上的亮點的最大距離.

//   //通過畫出檢測到的直線來顯示結果.
//   for( size_t i = 0; i < lines.size(); i++ )
//   {
//     Vec4i l = lines[i];//Vec4i 就是Vec<int, 4>，裡面存放４個int
//     line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
//     //要劃的線所在的影象：cdst，　起點：Point(l[0], l[1])，　終點：Point(l[2], l[3])，　顏色：Scalar(0,0,255)，
//   }
//  #endif
//  imshow("source", orign_img);
//  imshow("detected lines", cdst);


    vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
    vector<Point3i> qq;
	findContours(edge,contours,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE,Point());
	Mat imageContours=Mat::zeros(edge.size(),CV_8UC1);
	Mat Contours=Mat::zeros(edge.size(),CV_8UC1);  //绘制
	for(int i=0;i<contours.size();i++)
	{
		//contours[i]代表的是第i个轮廓，contours[i].size()代表的是第i个轮廓上所有的像素点数
		for(int j=0;j<contours[i].size();j++) 
		{
			//绘制出contours向量内所有的像素点
			Point P=Point(contours[i][j].x,contours[i][j].y);
			Contours.at<uchar>(P)=255;
            printf("(i,j) = (%d, %d) , (x,y) = (%d, %d)\n", i, j, contours[i][j].x, contours[i][j].y);
            qq.push_back(Point3i(contours[i][j].x,contours[i][j].y, 0));
            // Computational_geometryInstance->equal(P, P);
		}
    
		//输出hierarchy向量内容
		char ch[256];
		sprintf(ch,"%d",i);
		string str=ch;
		cout<<"向量hierarchy的第" <<str<<" 个元素内容为："<<endl<<hierarchy[i]<<endl<<endl;
        ROS_INFO("qq? %d", Computational_geometry->isPointInPolygon(qq, Point3i(190, 150, 0)));
 
		//绘制轮廓
		drawContours(imageContours,contours,i,Scalar(255),1,8,hierarchy);
	}
	imshow("Contours Image",imageContours); //轮廓
	imshow("Point of Contours",Contours);   //向量contours内保存的所有轮廓点集
    waitKey(30);
    


        for(int i = 0;i<100000000; i++);
    }
    checkImageSource = false;
}