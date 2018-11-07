#include <iostream>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_representation.h>

#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/PointIndices.h>
#include <inttypes.h>
#include <stdint.h>

using namespace pcl;
using namespace octree;
using namespace io;
using namespace std;

typedef PointXYZRGB PointT;
typedef PointXYZRGBL PointL;

int anzahlClouds;

int main(int argc, char** argv)
{
	// Einlesen der Dateinamen der 5 Punktewolken
	vector<int> filenames;
	filenames = console::parse_file_extension_argument(argc, argv, ".ply");

	vector<PointCloud<PointL>::Ptr, Eigen::aligned_allocator<PointCloud<PointL>::Ptr>> clouds;
	// Lade die clouds in einen cloud-vektor
	for (int i = 0; i < filenames.size(); ++i)
	{
		anzahlClouds = filenames.size();
		PointCloud<PointL>::Ptr cloud(new PointCloud<PointL>());
		if (loadPLYFile(argv[filenames[i]], *cloud) < 0) {
			cout << "Fehler beim Laden der Cloud " << argv[filenames[i]] << endl << endl;
			return -1;
		}
		clouds.push_back(cloud);
		/*cout << "test" << i << endl;*/
	}

	PointCloud<PointL>::Ptr cloud1(new PointCloud<PointL>());
	PointCloud<PointL>::Ptr cloud2(new PointCloud<PointL>());
	PointCloud<PointL>::Ptr cloud3(new PointCloud<PointL>());
	PointCloud<PointL>::Ptr cloud4(new PointCloud<PointL>());

	PointCloud<PointL>::Ptr cloudAlle(new PointCloud<PointL>());

	*cloud1 = *clouds[0];
	/**cloud2 = *clouds[1];
	*cloud3 = *clouds[2];
	*cloud4 = *clouds[3];*/

	// Initialisiere Label
	/*uint32_t label1 = 1;
	uint32_t label2 = 2;
	uint32_t label3 = 3;
	uint32_t label4 = 4;*/

	/*cout << "vor pointcloud label shiet" << endl;
	for (auto &p1 : cloud1->points) {
		p1.label = label1;
	}
	for (auto &p2 : cloud2->points) {
		p2.label = label2;
	}
	for (auto &p1 : cloud3->points) {
		p1.label = label3;
	}
	for (auto &p2 : cloud4->points) {
		p2.label = label4;
	}*/

	/*stringstream ss;
	ss << "label261.ply";
	savePLYFile(ss.str(), *cloudLabel1, true);
	stringstream ss2;
	ss2 << "label2.ply";
	savePLYFile(ss2.str(), *cloudLabel2, true);*/

	/**cloudAlle = *cloud1 + *cloud2;
	*cloudAlle = *cloudAlle + *cloud3;
	*cloudAlle = *cloudAlle + *cloud4;*/

	vector<int> counter = { 0, 0, 0, 0 };

	for (auto &p2 : cloud1->points) {
		if (p2.label == 1)
		{
			counter[0] += 1;
		}
		if (p2.label == 2)
		{
			counter[1] += 1;
		}
		if (p2.label == 3)
		{
			counter[2] += 1;
		}
		if (p2.label == 4)
		{
			counter[3] += 1;
		}
	}
	cout << " label1: " << counter[0] << endl;
	cout << " label2: " << counter[1] << endl;
	cout << " label3: " << counter[2] << endl;
	cout << " label4: " << counter[3] << endl;
	cout << "         break           " << endl;

	/*stringstream ss3;
	ss3 << "labelTestDaniel.ply";
	savePLYFile(ss3.str(), *cloudAlle, true);

	vector<int> counter2 = { 0, 0, 0, 0 };

	for (auto &p2 : cloudAlle->points) {
		if (p2.label == 1)
		{
			counter2[0] += 1;
		}
		if (p2.label == 2)
		{
			counter2[1] += 1;
		}
		if (p2.label == 3)
		{
			counter2[2] += 1;
		}
		if (p2.label == 4)
		{
			counter2[3] += 1;
		}
	}
	cout << " label1: " << counter2[0] << endl;
	cout << " label2: " << counter2[1] << endl;
	cout << " label3: " << counter2[2] << endl;
	cout << " label4: " << counter2[3] << endl;*/
	
	/*uint8_t r1 = 255, g1 = 0, b1 = 0;
	uint32_t rgb1 = ((uint32_t)r1 << 16 | (uint32_t)g1 << 8 | (uint32_t)b1);
	uint8_t r2 = 0, g2 = 255, b2 = 0;
	uint32_t rgb2 = ((uint32_t)r2 << 16 | (uint32_t)g2 << 8 | (uint32_t)b2);
	uint8_t r3 = 0, g3 = 0, b3 = 255;
	uint32_t rgb3 = ((uint32_t)r3 << 16 | (uint32_t)g3 << 8 | (uint32_t)b3);
	uint8_t r4 = 255, g4 = 255, b4 = 0;
	uint32_t rgb4 = ((uint32_t)r4 << 16 | (uint32_t)g4 << 8 | (uint32_t)b4);
	uint8_t r5 = 255, g5 = 0, b5 = 255;
	uint32_t rgb5 = ((uint32_t)r5 << 16 | (uint32_t)g5 << 8 | (uint32_t)b5);

	for (size_t i = 0; i < cloud1->points.size(); ++i) {
		switch (cloud1->points[i].label) {
		case 1: {
			cloud1->points[i].rgb = *reinterpret_cast<float*>(&rgb1);
			break;
		}
		case 2: {
			cloud1->points[i].rgb = *reinterpret_cast<float*>(&rgb2);
			break;
		}
		case 3: {
			cloud1->points[i].rgb = *reinterpret_cast<float*>(&rgb3);
			break;
		}
		case 4: {
			cloud1->points[i].rgb = *reinterpret_cast<float*>(&rgb4);
			break;
		}
		case 5: {
			cloud1->points[i].rgb = *reinterpret_cast<float*>(&rgb5);
			break;
		}
		}
	}

	stringstream ss3;
	ss3 << "FarbKonvertierung2.ply";
	savePLYFile(ss3.str(), *cloud1, true);*/
}

