#include <cmath>
#include <ros/ros.h>
#include "hungarian.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl/impl/point_types.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <pcl/filters/extract_indices.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pointcloud_msgs/PointCloud2_Segments.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>


int marker_flag;

visualization_msgs::Marker marker_sphere;
visualization_msgs::Marker marker_line;


std::vector<float> red = {0, 0, 1, 1, 1, 102.0/255, 102.0/255, 204.0/255, 0, 1};
std::vector<float> green = {0, 1.0, 0, 1, 1, 102.0/255, 102.0/255, 0, 1, 152.0/255};
std::vector<float> blue = {1.0, 0, 0, 0, 1, 152.0/255, 52.0/255, 152.0/255, 1, 52.0/255};


//-------------------------------------------------------------------




class Centroid_tracking{
public:

    int max_id ;

    pointcloud_msgs::PointCloud2_Segments base_msg;

    Centroid_tracking ( pointcloud_msgs::PointCloud2_Segments& base_msg, int max_id ) 
    {
        this->base_msg = base_msg ;
        this->max_id = max_id ;
    }

    void track ( pointcloud_msgs::PointCloud2_Segments& msg ) {

        std::vector<Eigen::Vector4f> msg_centroid_vec;
        std::vector<Eigen::Vector4f> base_centroid_vec;

        //first frame 
        for (int i=0; i < base_msg.clusters.size(); i++)
        {
            pcl::PointXYZ centroidpoint ;
            pcl::PCLPointCloud2 pc2;
            pcl_conversions::toPCL ( base_msg.clusters[i] , pc2 );

            pcl::PointCloud<pcl::PointXYZ> cloud2;
            pcl::fromPCLPointCloud2 ( pc2 , cloud2 );

            Eigen::Vector4f base_centroid;
            pcl::compute3DCentroid ( cloud2 , base_centroid);
            base_centroid_vec.push_back( base_centroid );

          //  std::cout << "base_centroid_vec= " << base_centroid_vec[i] << std::endl;
        }
         
     //   std::cout << "size of base_centroid_vec= " << base_centroid_vec.size() << std::endl;


        //second frame
        for (int i=0; i < msg.clusters.size(); i++)
        {
            pcl::PointXYZ centroidpoint ;
            pcl::PCLPointCloud2 pc2;
            pcl_conversions::toPCL ( msg.clusters[i] , pc2 );

            pcl::PointCloud<pcl::PointXYZ> cloud2;
            pcl::fromPCLPointCloud2 ( pc2 , cloud2 );
            Eigen::Vector4f base_centroid;
            pcl::compute3DCentroid ( cloud2 , base_centroid);

            msg_centroid_vec.push_back( base_centroid );
            msg.cluster_id[i] = -1;
        }

      //  std::cout <<"size of msg_centroid_vec= " << msg_centroid_vec.size() << std::endl;

        size_t size_old = base_centroid_vec.size();
        size_t size_new = msg_centroid_vec.size();
        unsigned totalsz = size_old + size_new;
        std::vector<std::vector<int> > dists(totalsz, std::vector<int>(totalsz , 10000));// TODO currently, 10000 is the maximum (2d) int distance with a 10 meter laser scanner. Initial value represents a point connected to bottom.

        for(unsigned i=0; i < size_old + size_new; i++){
            for(unsigned j=0; j < size_new + size_old; j++){
                if(i < size_old and j < size_new){
                    dists[i][j] = 1000 * sqrt(pow(base_centroid_vec[i][0]-msg_centroid_vec[j][0], 2) + pow(base_centroid_vec[i][1]-msg_centroid_vec[j][1], 2) + pow(base_centroid_vec[i][2]-msg_centroid_vec[j][2], 2));
                }
                else if(i >= size_old and j >= size_new){
                    dists[i][j] = 0; // connecting bottom to bottom is free!
                }
            }
        }

        Hungarian hungarian(dists , totalsz, totalsz, HUNGARIAN_MODE_MINIMIZE_COST) ;
        // fprintf(stderr, "cost-matrix:");
        // hungarian.print_cost();
        hungarian.solve();
        // fprintf(stderr, "assignment:");
        // hungarian.print_assignment();
        // std::cout << "size_old = " << size_old << ", size_new = " << size_new << std::endl;

        dists = hungarian.assignment();

        if(marker_flag==1) {

            marker_line.points.clear();        
            marker_sphere.points.clear();
            marker_sphere.colors.clear();
            marker_line.colors.clear();
        }


        for(unsigned j=0; j < size_new; j++){
            for(unsigned i=0; i < size_old; i++){
                if (dists[i][j] == 1){

                    msg.cluster_id[j] = base_msg.cluster_id[i];


                    if(marker_flag==1) {

                        uint mod = msg.cluster_id[j] % 10;

                        
                        std_msgs::ColorRGBA c;
                        
                        c.r = red[mod];
                        c.g = green[mod];
                        c.b=blue[mod];
                        c.a=0.7;


                        geometry_msgs::Point p;
                       
                        p.x = base_centroid_vec[i][0];
                        p.y = base_centroid_vec[i][1];
                        p.z = base_centroid_vec[i][2];

                                           
                        marker_line.points.push_back(p);
                        marker_sphere.points.push_back(p);


                        geometry_msgs::Point pp;                    

                        pp.x = msg_centroid_vec[j][0];
                        pp.y = msg_centroid_vec[j][1];
                        pp.z = msg_centroid_vec[j][2];

                       
                        marker_line.points.push_back(pp);
                        marker_sphere.points.push_back(pp);


                        marker_line.colors.push_back(c);
                        marker_line.colors.push_back(c);
                        marker_sphere.colors.push_back(c);
                        c.a=0.3;
                        marker_sphere.colors.push_back(c);
                    }
                    break;
                }
            }
            msg.cluster_id[j] = msg.cluster_id[j] == -1 ? ++max_id : msg.cluster_id[j];
        }

        // for (unsigned j=0; j < size_new; j++){
        //     std::cout << "cluster#" << j << ", clusterID:" << msg.cluster_id[j] << std::endl;
        // }
    }

};



//------------------------------------------------------------------



ros::Publisher pub;
ros::Subscriber sub;
ros::Publisher marker_pub;

bool b = true;
int size, max_id, method;
double overlap, offset ;


std::vector<pointcloud_msgs::PointCloud2_Segments> v_;
std::vector<pointcloud_msgs::PointCloud2_Segments> new_v(2);


visualization_msgs::MarkerArray marker;



std::pair<double,double> overlap_range (const pointcloud_msgs::PointCloud2_Segments& cls){

    double z_max = 0 ;
    double height_after ,height_before ;
    double z_min = std::numeric_limits<double>::max();

    std::vector<pointcloud_msgs::PointCloud2_Segments> vec;

    vec.push_back(cls);

    if (vec.size() > size){
        vec.erase(vec.begin());
    }

    for (unsigned i=0; i < vec.size(); i++)
    {
        double offset;

        if ( i > 0 ){

            offset = ( 1.0 - overlap ) * (double)( ros::Duration( vec[i].first_stamp - vec[0].first_stamp ).toSec()) * (double)( cls.factor );

            for (unsigned j=0; j < vec[i].clusters.size(); j++)
            {
                sensor_msgs::PointCloud cloud;
                sensor_msgs::convertPointCloud2ToPointCloud( vec[i].clusters[j] , cloud );

                for (unsigned k=0; k < cloud.points.size(); k++)
                {
                    cloud.points[k].z += offset ;
                    if (cloud.points[k].z < z_min){
                        z_min = cloud.points[k].z ;
                    }
                }
            }
        }
        else {

            offset = 0.0;

            for (unsigned j=0; j < vec[0].clusters.size(); j++){
                sensor_msgs::PointCloud cloud;
                sensor_msgs::convertPointCloud2ToPointCloud( vec[0].clusters[j] , cloud );

                for ( unsigned l=0; l < cloud.points.size(); l++){
                    if ( cloud.points[l].z > z_max ){
                        z_max = cloud.points[l].z;
                    }
                }
            }
        }

    }

    return std::make_pair( z_max , z_min );

}

pointcloud_msgs::PointCloud2_Segments clusters_in_overlap (const pointcloud_msgs::PointCloud2_Segments& cl , double z_overlap_min , double z_overlap_max ){

    sensor_msgs::PointCloud2 pc1;
    pointcloud_msgs::PointCloud2_Segments output;

    for ( unsigned i=0; i < cl.clusters.size(); i++){

        sensor_msgs::PointCloud cloud;
        sensor_msgs::convertPointCloud2ToPointCloud( cl.clusters[i] , cloud );

        pcl::PCLPointCloud2 pc2 ;
        sensor_msgs::PointCloud2 cl2 ;

        sensor_msgs::convertPointCloudToPointCloud2( cloud , cl2 );
        pcl_conversions::toPCL ( cl2 , pc2 );

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2 ( pc2 , *cloud2 );
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::ExtractIndices<pcl::PointXYZ> extract ;

        for (int i=0; i < (*cloud2).size(); i++){
            pcl::PointXYZ pt(cloud2->points[i].x , cloud2->points[i].y , cloud2->points[i].z);
            if (pt.z <= z_overlap_max and pt.z >= z_overlap_min){
                inliers->indices.push_back(i);
            }
        }
        extract.setInputCloud(cloud2);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*cloud2);
        pcl::PCLPointCloud2 pcl_cloud;
        pcl::toPCLPointCloud2(*cloud2, pcl_cloud);
        pcl_conversions::fromPCL(pcl_cloud, pc1);
    
        output.clusters.push_back(pc1);
    }
    return output;
}



void callback (const pointcloud_msgs::PointCloud2_Segments& msg ){


    if(marker_flag==1) {

        marker_sphere.ns = "shapeOfsphere";
        marker_sphere.id = 0;
        marker_sphere.type = 7;
        marker_sphere.action = visualization_msgs::Marker::ADD;
        marker_sphere.pose.orientation.w = 1.0;
        marker_sphere.color.a = 1.0;

        marker_sphere.scale.x = 0.3;
        marker_sphere.scale.y = 0.3;
        marker_sphere.scale.z = 0.3;

        marker_line.ns = "shapeOfline";
        marker_line.id = 1;    
        marker_line.type = 5;
        marker_line.action = visualization_msgs::Marker::ADD;
        marker_line.pose.orientation.w = 1.0;
        marker_line.color.a = 1.0;

        marker_line.scale.x = 0.1;
        marker_line.scale.y = 0.1;
        marker_line.scale.z = 0.1;



        marker.markers.push_back(marker_sphere);
        marker.markers.push_back(marker_line);

        marker_sphere.header.frame_id = "base_link";
        marker_sphere.header.stamp = msg.header.stamp;
        marker_sphere.lifetime = ros::Duration();


        marker_line.header.frame_id = "base_link";
        marker_line.header.stamp = msg.header.stamp;
        marker_line.lifetime = ros::Duration();
    }

    double overlap_height_min , overlap_height_max ;

    sensor_msgs::PointCloud cloud;
    pointcloud_msgs::PointCloud2_Segments c_;

    if ( method == 1 ){
        std::pair<double,double> z_height = overlap_range(msg);
        overlap_height_min = z_height.first;
        overlap_height_max = z_height.second;
    }

    v_.push_back(msg);

    Centroid_tracking* t;

    if(v_.size()>3) return;


    if (v_.size() > size){
        v_.erase(v_.begin());
    }

    if(v_.size()>=2) {

        if ( b ){

            for (unsigned i=0; i < v_[0].clusters.size(); i++){
                v_[0].cluster_id.push_back(i);
            }
            b = false;
        }

       

        if (method == 1 ){


             new_v[0]=v_[0];

            new_v[1] = clusters_in_overlap(v_[1] , overlap_height_min , overlap_height_max);

            for (int i=0; i < v_[1].clusters.size(); i++){
           
                new_v[1].cluster_id.push_back(i);
            }

        }        
            
        if ( method == 2) {

            for (int i=0; i < v_[1].clusters.size(); i++){

                v_[1].cluster_id.push_back(i);
            }
        }

        if (method == 1 ){
            for (unsigned i=0; i < new_v[0].cluster_id.size(); i++){
                if ( new_v[0].cluster_id[i] > max_id){
                    max_id = new_v[0].cluster_id[i];
                }
            }

            t = new Centroid_tracking( new_v[0] , max_id );
        }
        else if (method == 2){
           for (unsigned i=0; i < v_[0].cluster_id.size(); i++){
                if (v_[0].cluster_id[i] > max_id){
                    max_id = v_[0].cluster_id[i];
                }
            }

            t = new Centroid_tracking( v_[0] , max_id );
        }
    }
    else {
        t = NULL;
    }

    //if (b==false){
    if(t != NULL) {
        if ( method == 1 ){
            t-> track( new_v[1] );
            //track();

            for (unsigned i=0; i < new_v[1].cluster_id.size(); i++){
                v_[1].cluster_id.push_back(new_v[1].cluster_id[i]);
            }
        }
        else if ( method == 2){
            t-> track( v_[1]);
          //  track();
        }
    }


    for (unsigned i=0; i < v_.size(); i++)
    {
        double offset=0.0;

     //     if ( i > 0 ){
     //       offset = ( 1.0 - overlap ) * (double)( ros::Duration( v_[i].first_stamp - v_[0].first_stamp ).toSec()) * (double)( msg.factor );
       // }
      //  else {
        //    offset = 0.0;
      //  }

        for (unsigned j=0; j < v_[i].clusters.size(); j++)
        {
            sensor_msgs::PointCloud cloud;
            sensor_msgs::convertPointCloud2ToPointCloud( v_[i].clusters[j] , cloud );

            for (unsigned k=0; k < cloud.points.size(); k++){
                cloud.points[k].z += offset;
            }

            sensor_msgs::PointCloud2 pc2;
            sensor_msgs::convertPointCloudToPointCloud2( cloud , pc2 );
            c_.clusters.push_back( pc2 );
        }

        for (int k=0; k < v_[i].cluster_id.size(); k++){
            c_.cluster_id.push_back(v_[i].cluster_id[k]);
        }
    }    

    c_.header.stamp = ros::Time::now();
    c_.header.frame_id = msg.header.frame_id;
    c_.factor = msg.factor ;
    c_.overlap = msg.overlap ;
    c_.num_scans = msg.num_scans ;
    c_.first_stamp = msg.first_stamp ;
    c_.angle_min = msg.angle_min;
    c_.angle_max = msg.angle_max;
    c_.angle_increment = msg.angle_increment;
    c_.range_min = msg.range_min;
    c_.range_max = msg.range_max;
    c_.scan_time = msg.scan_time;
    c_.rec_time = msg.rec_time;
    

    
    pub.publish(c_);


    if(marker_flag==1) {

        if ( b==false ) marker_pub.publish(marker);

        marker.markers.clear();
    }
}


int main(int argc, char** argv){

    ros::init(argc, argv, "pointcloud2_cluster_tracking");
    ros::NodeHandle n_;

    std::string out_topic;
    std::string input_topic;
    std::string marker_topic;


    n_.param("pointcloud2_cluster_tracking/size", size , 2);
    n_.param("pointcloud2_cluster_tracking/method", method , 1);
    n_.param("pointcloud2_cluster_tracking/overlap", overlap , 0.2);
    n_.param("pointcloud2_cluster_tracking/marker_flag", marker_flag , 0);

    n_.param("pointcloud2_cluster_tracking/out_topic", out_topic , std::string("/pointcloud2_cluster_tracking/clusters"));
    n_.param("pointcloud2_cluster_tracking/input_topic", input_topic , std::string("pointcloud2_clustering/clusters"));

    if(marker_flag==1) {

        n_.param("pointcloud2_cluster_tracking/marker_topic", marker_topic , std::string("visualization_marker"));
        marker_pub = n_.advertise<visualization_msgs::MarkerArray>(marker_topic, 1);
    }


    sub = n_.subscribe( input_topic, 1 , callback);
    pub = n_.advertise<pointcloud_msgs::PointCloud2_Segments>( out_topic, 1);

    

    ros::spin();
}