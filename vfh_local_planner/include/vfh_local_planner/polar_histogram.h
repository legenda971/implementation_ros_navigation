#include <vfh_local_planner/histogram.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <cmath>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define robotRadius 0.25
#define robotSafe 0.1

namespace vfh_local_planner
{
    struct Valley
    {
        unsigned int idx_l;
        unsigned int idx_r;

        Valley(unsigned int idx_l, unsigned int idx_r) : idx_l(idx_l), idx_r(idx_r) {}
    };

    class Polar_histogram
    {
    private:
        Histogram<double> histogram_;
        costmap_2d::Costmap2DROS *costmap_ros_;
        std::vector<int> valleys_;

    public:
        Polar_histogram(unsigned int number_of_sector_, costmap_2d::Costmap2DROS *costmap_ros) : histogram_(number_of_sector_), costmap_ros_(costmap_ros) {}

        bool calculate_magnitude()
        {
            // Get the current robot position and orientation
            geometry_msgs::PoseStamped current_pose;
            costmap_ros_->getRobotPose(current_pose);
            costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();

            unsigned int n = histogram_.getNumberOfSector();
            double alfa = 2.0 * M_PI / n;
            // ROS_INFO("Current pose: x %f, y %f, theta %f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.orientation.z);

            /* Calculate constant */
            // double dMax = center;
            // double rRS = robotRadius + robotSafe;
            // double a = dMax / (dMax - 0.25);
            // double b = a / dMax;

            /* Calculate magnitude */
            for (unsigned int _x = 0; _x < costmap->getSizeInCellsX(); _x++)
            {
                for (unsigned int _y = 0; _y < costmap->getSizeInCellsY(); _y++)
                {
                    double wx, wy;
                    costmap->mapToWorld(_x, _y, wx, wy);
                    double dx = wx - current_pose.pose.position.x;
                    double dy = wy - current_pose.pose.position.y;

                    double distance = sqrt(pow(dx, 2) + pow(dy, 2));
                    double angle = std::atan2(dy, dx);
                    if (angle < 0)
                        angle += 2 * M_PI;

                    unsigned char cost = costmap->getCost(_x, _y);
                    if (cost == costmap_2d::FREE_SPACE)
                        continue;

                    // unsigned int k = angle / alfa;
                    int bin = static_cast<int>((angle) / (2 * M_PI) * n) % n;
                    histogram_[bin] += 1;
                }
            }

            calculate_valleys();

            
        }

        void calculate_gradient()
        {
            unsigned int n = histogram_.getNumberOfSector();
            for (unsigned int i = 0; i < n; i++)
            {
                histogram_[i] = histogram_[(i + 1) % n] - histogram_[i];
            }
        }

        void calculate_valleys()
        {
            unsigned int n = histogram_.getNumberOfSector();
            for (unsigned int i = 0; i < n; i++)
            {
                if (histogram_[i] > 0)
                {
                    unsigned int idx_l = i;
                    unsigned int idx_r = (i + 1) % n;
                    while (histogram_[idx_r] > 0)
                    {
                        idx_r = (idx_r + 1) % n;
                    }
                    valleys_.push_back(idx_l);
                    valleys_.push_back(idx_r);
                    i = idx_r;
                }
            }
        }

        void threshold(double treshold)
        {
            for (unsigned int i = 0; i < histogram_.getNumberOfSector(); i++)
            {
                if (histogram_[i] < treshold)
                {
                    histogram_[i] = 0;
                }
            }
        }

        void print()
        {
            unsigned int n = histogram_.getNumberOfSector();
            std::cout << "Sectors" << std::endl;
            for (int i = 0; i < n; i++)
            {
                std::cout << histogram_[i] << " ";
            }

            std::cout << std::endl;
        }

        std_msgs::ColorRGBA getColorRGBA(double r, double g, double b, double a)
        {
            std_msgs::ColorRGBA temp;
            temp.r = r;
            temp.g = g;
            temp.b = b;
            temp.a = a;

            return temp;
        }

        geometry_msgs::Point getPoint(int x, int y, int z)
        {
            geometry_msgs::Point p;
            p.x = x;
            p.y = y;
            p.z = z;

            return p;
        }

        geometry_msgs::Point getPoint(double x, double y, double z)
        {
            geometry_msgs::Point p;
            p.x = x;
            p.y = y;
            p.z = z;

            return p;
        }

        geometry_msgs::Vector3 getVector3(int x, int y, int z)
        {
            geometry_msgs::Vector3 temp;
            temp.x = x;
            temp.y = y;
            temp.z = z;

            return temp;
        }

        void drawHistogram(ros::Publisher &marker_pub)
        {
            // Get the current robot position and orientation
            geometry_msgs::PoseStamped current_pose;
            costmap_ros_->getRobotPose(current_pose);
            costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();

            double max_distance = costmap->getSizeInMetersX() / 2.0;

            unsigned int n = histogram_.getNumberOfSector();
            double alfa = 2.0 * M_PI / n;

            visualization_msgs::Marker marker;
            marker.header.frame_id = current_pose.header.frame_id;
            marker.header.stamp = ros::Time();
            marker.id = 500;
            marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale = getVector3(1, 1, 1);
            marker.color = getColorRGBA(0, 0, 0, 1);
            // marker.lifetime = ros::Duration(0.1);

            for (unsigned int i = 0; i < n; i++)
            {
                std_msgs::ColorRGBA color;
                if (histogram_[i] == 0)
                    color = getColorRGBA(0, 1, 0, 1);
                else
                    color = getColorRGBA(1, 0, 0, 1);

                for (unsigned int j = 0; j < valleys_.size(); j++)
                {
                    if (i == valleys_[j])
                    {
                        color = getColorRGBA(0, 0, 1, 1);
                    }
                }
                marker.colors.push_back(color);

                marker.points.push_back(getPoint(current_pose.pose.position.x,
                                                 current_pose.pose.position.y,
                                                 current_pose.pose.position.z));
                for (int o = 0; o < 2; o++)
                {
                    double alfa_ = (i + o) * alfa;

                    double c = cos(alfa_);
                    double s = sin(alfa_);
                    double x, y;
                    if (abs(s) < abs(c))
                    {
                        x = ((c >= 0) - (c < 0)) * max_distance;
                        y = (c == 0) ? 1 : (s / c) * x;
                    }
                    else
                    {
                        y = ((s >= 0) - (s < 0)) * max_distance;
                        x = (s == 0) ? 1 : (c / s) * y;
                    }

                    marker.points.push_back(getPoint(x + current_pose.pose.position.x,
                                                     y + current_pose.pose.position.y,
                                                     current_pose.pose.position.z));
                }
            }

            marker_pub.publish(marker);
        }
    };

}