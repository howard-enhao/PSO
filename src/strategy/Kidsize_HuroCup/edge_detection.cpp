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

void Edge_detection::stepcheck_callback(const std_msgs::Bool& msg)
{
    odd_step = msg.data;
    Footstepack = false;
}

void Edge_detection::Footstepack_callback(const std_msgs::Bool& msg)
{
    Footstepack = msg.data;
    usleep(2000*1000);
    checkImageSource = false;
}

// string type2str(int type)
// {
//     string r;

//     uchar depth = type & CV_MAT_DEPTH_MASK;
//     uchar chans = 1 + (type >> CV_CN_SHIFT);

//     switch ( depth ) {
//         case CV_8U:  r = "8U"; break;
//         case CV_8S:  r = "8S"; break;
//         case CV_16U: r = "16U"; break;
//         case CV_16S: r = "16S"; break;
//         case CV_32S: r = "32S"; break;
//         case CV_32F: r = "32F"; break;
//         case CV_64F: r = "64F"; break;
//         default:     r = "User"; break;
//     }

//     r += "C";
//     r += (chans+'0');

//     return r;
// }

void Edge_detection::save_image(const Mat orignimage)
{
    char save_path[200] = "";
    printf("%d\n", name_cnt);
    string tmp = to_string(name_cnt);
    tmp = "/origin_image"+tmp+".png";
    strcat(save_path,path_2);
    strcat(save_path,tmp.c_str());
    imwrite(save_path, orignimage);
}

void Edge_detection::initial()
{
    checkImageSource = false;
    Footstepack = false;
    name_cnt = 0;
    odd_step = false;
    height_2 = 69;
    // path_2 = "/home/iclab/Desktop/PSO";
    // path_1 = "/home/iclab/Desktop/PSO/finalimage.png";
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Edge_detection");
    ros::NodeHandle nh;
    Edge_detection Edge_detection(nh);

    ros::Rate loop_rate(30);
    Edge_detection.initial();
    while(nh.ok())
    {
        Edge_detection.strategymain(nh);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

void Edge_detection::strategymain(ros::NodeHandle nh)
{
    // if(!odd_step)
    //     name_cnt = 0;
    if(!orign_img.empty()&& checkImageSource && Footstepack && odd_step)
    {
        save_image(orign_img);
        name_cnt++;
        // orign_img = imread("/home/iclab/Desktop/PSO/test.png");
        // orign_img = imread("/home/ching/git/PSO/test.png");
        // imshow("view", orign_img);
        // waitKey(30);
        Mat frame_img = Mat::zeros(orign_img.size(),CV_8UC1);
        if(now_step % 2 == 1)
        {
            reachable_region.now_step = now_step;
            reachable_region.x = 185;
            reachable_region.y = 130;
            reachable_region.Width = 115;
            reachable_region.Height = 109;
            reachable_region.c_x = 220;
            reachable_region.c_y = 205;
        }
        else
        {
            reachable_region.now_step = now_step;
            reachable_region.x = 90;
            reachable_region.y = 130;
            reachable_region.Width = 115;
            reachable_region.Height = 109;
            reachable_region.c_x = 170;
            reachable_region.c_y = 205;
        }
            // reachable_region.x = 90;
            // reachable_region.y = 130;
            // reachable_region.Width = 210;
            // reachable_region.Height = 109;
        // if(pre_now_step != now_step)
        // {
            Rect rect(reachable_region.x, reachable_region.y, reachable_region.Width, reachable_region.Height);
            Rect rect2(reachable_region.x, reachable_region.y, reachable_region.Width, height_2);
            Mat ROI = orign_img(rect2);
            Mat invert_tmp = Mat::zeros(rect.size(),CV_8UC1);
            // invert_tmp = Scalar(255);
            // Mat ROI_tmp = orign_img(rect2);
            // erode(orign_img, orign_img, Mat());
            // dilate(orign_img, orign_img, Mat());

            GaussianBlur(orign_img, orign_img, Size(17,17),0.6);
            erode(orign_img, orign_img, Mat());
            dilate(orign_img, orign_img, Mat());
            Canny(orign_img, edge, 120, 240, 3);
            Reachable_region_pub.publish(reachable_region);
            // cvtColor(edge, frame, cv::COLOR_GRAY2BGR);
            // edgeimage_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            // edgeimage_Publisher.publish(edgeimage_msg);

            // string edge_type =  type2str( edge.type() );
            // printf("Matrix: %s %dx%d \n\n\n\n", edge_type.c_str(), edge.cols, edge.rows );

            vector<Point3i> edge_point;

            /*  繪製原始輪廓與顯示輪廓點集 */
            // vector<vector<Point>> contours;
            // vector<Vec4i> hierarchy;
            // findContours(edge,contours,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE,Point());
            // Mat imageContours=Mat::zeros(edge.size(),CV_8UC1);
            // Mat Contours=Mat::zeros(edge.size(),CV_8UC1);

            /*  縮小物件輪廓 */
            vector<vector<Point>> ring_contours;
            vector<Vec4i> ring_hierarchy;
            Mat ROI_frame;
            Mat dist=Mat::zeros(edge.size(),CV_32FC1);
            Mat Shrink=Mat::zeros(edge.size(),CV_8UC1);
            Mat ring = Mat::zeros(edge.size(),CV_8UC1);
            cvtColor(ROI, ROI_frame, cv::COLOR_BGR2GRAY);
            Mat edge_1;
            Canny(ROI_frame, edge_1, 50, 150, 3);
            edge_1 = ~edge_1;
            invert_tmp = ~invert_tmp;
            Mat temp = frame_img(rect);
            invert_tmp.copyTo(temp);
            Mat temp_2 = frame_img(rect2);
            edge_1.copyTo(temp_2);
            distanceTransform(frame_img, dist, DIST_L2, DIST_MASK_PRECISE);
            inRange(dist, 9, 10, ring);
            threshold(ring, ring, 1, 255, CV_THRESH_BINARY);
            findContours(ring,ring_contours,ring_hierarchy,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE,Point());
            
            strategy::EdgePointList edgepointlist;
            for(int i=0;i<ring_contours.size();i++)
            {
                bool PointWithinObs = false;
                int InObs = -1;
                bool CCRisInObs = false;
                geometry_msgs::Point32 point;
                strategy::EdgePointArray polygon;
                
                // contours[i]代表的是第i個輪廓，contours[i].size()代表的是第i個輪廓上所有的像素點數
                // for(int j=0;j<contours[i].size();j++) 
                // {
                //     // 繪製出contours向量內所有的像素點
                //     Point P=Point(contours[i][j].x,contours[i][j].y);
                //     Contours.at<uchar>(P)=255;
                //     // edge_point.push_back(Point3i(contours[i][j].x,contours[i][j].y, 0));
                //     printf("(i,j) = (%d, %d) , (x,y) = (%d, %d)\n", i, j, contours[i][j].x, contours[i][j].y);
                
                // }

                for(int j=0;j<ring_contours[i].size();j++) 
                {
                    edge_point.push_back(Point3i(ring_contours[i][j].x,ring_contours[i][j].y, 0));  //將shrink的邊緣點存入edge_point
                    printf("ring_(i,j) = (%d, %d) , (x,y) = (%d, %d)\n", i, j, ring_contours[i][j].x, ring_contours[i][j].y);
                    point.x = ring_contours[i][j].x;
                    point.y = ring_contours[i][j].y;
                    point.z = 0;
                    polygon.points.push_back(point);
                    // printf("%f, %f, %f\n", point.x, point.y, point.z);
                }
                // printf("size = %d \n", polygon.points.size());
                bool enough_space_flag;
                enough_space_flag = Computational_geometry->areaOfPolygon(edge_point);
                if(enough_space_flag)
                    edgepointlist.Edgepointlist.push_back(polygon);
                polygon.points.clear();
                // 輸出hierarchy向量内容
                char ch[256];
                sprintf(ch,"%d",i);
                string str=ch;
                cout<<"向量hierarchy的第" <<str<<" 個元素内容為:"<<ring_hierarchy[i]<<endl<<endl;
                printf("\nok = %d\n", Computational_geometry->areaOfPolygon(edge_point));
                // printf("point  %d\n", Computational_geometry->isPointInPolygon(edge_point, Point3i(200, 120, 0)));
                // printf("point = %d\n", Computational_geometry->isCircleInPolygon(edge_point, Point3i(200, 120, 0), 5));
                // CCRisInObs = Computational_geometry->isCircleInPolygon(edge_point, Point3i(220, 188, 0), 15);
                // PointWithinObs = Computational_geometry->isPointInPolygon(edge_point, Point3i(200, 120, 0));
                edge_point.clear();

                // drawContours(imageContours,contours,i,Scalar(255),1,8,hierarchy);    // 繪製輪廓
                if(enough_space_flag)
                    drawContours(Shrink,ring_contours,i,Scalar(255),1,8,ring_hierarchy);    // 繪製輪廓
                
                if(PointWithinObs || CCRisInObs)
                {
                    InObs = stoi(str);
                    printf("可踏步在第 %d 個障礙物內\n", InObs);
                    // break;
                }
                cout<<endl<<endl<<endl;
            }
            edgepoint_pub.publish(edgepointlist);
            edgepointlist.Edgepointlist.clear();
            addWeighted(Shrink, 1, edge, 0.3, 0, Shrink);  //合成edgeimg與shrinkimg
            rectangle(Shrink, Rect(140,161,50,68), Scalar(150), 1, 8, 0);
            rectangle(Shrink, Rect(200,161,50,68), Scalar(150), 1, 8, 0);
            rectangle(Shrink, rect, Scalar(200), 1, 8, 0);
            // imshow("dist", dist);
            // imshow("ring", ring);
            // imshow("Shrink", Shrink);  //縮小後的輪廓
            // imshow("Contours Image",imageContours); //輪廓
            // imshow("Point of Contours",Contours);   //向量contours内保存的所有輪廓點集
            waitKey(30);
            imwrite(path_1, Shrink);
            // imwrite("/home/ching/git/PSO/finalimage.png", Shrink);
            cvtColor(Shrink, frame, cv::COLOR_GRAY2BGR);
            edgeimage_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            printf("now_step = %d\n", now_step);
            pre_now_step = now_step;
            now_step++;
        // }
        edgeimage_Publisher.publish(edgeimage_msg);
        Footstepack = false;
        usleep(500*1000);
    }
    checkImageSource = false;
    if(!odd_step)
    {
        // Footstepack = true;
        // odd_step = false;
        now_step = 1;
        name_cnt = 0;
    }
}