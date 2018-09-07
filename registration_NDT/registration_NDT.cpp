#include <iostream>

#include <pcl/io/ply_io.h>
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

// Abkuerzungen
typedef PointXYZRGB PointT;
typedef PointXYZRGBL PointL;

// Hilfefunktion bei falscher Eingabe
void
showHelp(char * program_name)
{
	cout << endl;
	cout << "Bedienung: " << program_name << " cloud_filename.ply" << endl;
	cout << "-h:  Diese Hilfefunktion anzeigen." << endl;
}



int
main(int argc, char** argv)
{
	// Hilfe Funktion
	if (console::find_switch(argc, argv, "-h") || console::find_switch(argc, argv, "--help")) {
		showHelp(argv[0]);
		return 0;
	}

	// Einlesen der Dateinamen der 6 Punktewolken
	vector<int> filenames;
	filenames = console::parse_file_extension_argument(argc, argv, ".ply");

	// Cloud 1-6 sind die Punktewolken der 6 Scan-Standorte
	PointCloud<PointT>::Ptr cloud1(new PointCloud<PointT>());
	PointCloud<PointT>::Ptr cloud2(new PointCloud<PointT>());
	PointCloud<PointT>::Ptr cloud3(new PointCloud<PointT>());
	PointCloud<PointT>::Ptr cloud4(new PointCloud<PointT>());
	PointCloud<PointT>::Ptr cloud5(new PointCloud<PointT>());
	PointCloud<PointT>::Ptr cloud6(new PointCloud<PointT>());
	PointCloud<PointL>::Ptr cloudLabel1(new PointCloud<PointL>());
	PointCloud<PointL>::Ptr cloudLabel2(new PointCloud<PointL>());
	PointCloud<PointL>::Ptr cloudLabelRB1(new PointCloud<PointL>());
	PointCloud<PointL>::Ptr cloudLabelRB2(new PointCloud<PointL>());

	// Lade die Wolken
	if (loadPLYFile(argv[filenames[0]], *cloud1) < 0) {
		cout << "Fehler beim Laden der Cloud " << argv[filenames[0]] << endl << endl;
		showHelp(argv[0]);
		return -1;
	}
	if (loadPLYFile(argv[filenames[1]], *cloud2) < 0) {
		cout << "Fehler beim Laden der Cloud " << argv[filenames[1]] << endl << endl;
		showHelp(argv[0]);
		return -1;
	}
	if (filenames.size() == 6) {
		if (io::loadPLYFile(argv[filenames[2]], *cloud3) < 0) {
			cout << "Fehler beim Laden der Cloud " << argv[filenames[2]] << endl << endl;
			showHelp(argv[0]);
			return -1;
		}
		if (io::loadPLYFile(argv[filenames[3]], *cloud4) < 0) {
			cout << "Fehler beim Laden der Cloud " << argv[filenames[3]] << endl << endl;
			showHelp(argv[0]);
			return -1;
		}
		if (io::loadPLYFile(argv[filenames[4]], *cloud5) < 0) {
			cout << "Fehler beim Laden der Cloud " << argv[filenames[5]] << endl << endl;
			showHelp(argv[0]);
			return -1;
		}
		if (io::loadPLYFile(argv[filenames[5]], *cloud6) < 0) {
			cout << "Fehler beim Laden der Cloud " << argv[filenames[5]] << endl << endl;
			showHelp(argv[0]);
			return -1;
		}
	}
	// temporaer: konvertiere zwei (beliebige) eingelesenen Clouds in (neue) XYZRGBL Clouds
	copyPointCloud(*cloud1, *cloudLabel1);
	copyPointCloud(*cloud2, *cloudLabel2);

	// Initialisiere RotBlau Clouds
	*cloudLabelRB1 = *cloudLabel1;
	*cloudLabelRB2 = *cloudLabel2;

	// Iteriere ueber beide Clouds und gebe jedem Punkt jeweils Label 1 oder 2, je nach Zugehhoerigkeit
	uint32_t label1 = 1;
	uint8_t r1 = 255, g1 = 0, b1 = 0;
	uint32_t rgb1 = ((uint32_t)r1 << 16 | (uint32_t)g1 << 8 | (uint32_t)b1);

	// Labels und Farben setzten
	for (auto &p1 : cloudLabel1->points) {
		p1.label = label1;
	}
	for (auto &p1 : cloudLabelRB1->points) {
		p1.label = label1;
		p1.rgb = *reinterpret_cast<float*>(&rgb1);
	}

	uint32_t label2 = 2;
	uint8_t r2 = 0, g2 = 0, b2 = 255;
	uint32_t rgb2 = ((uint32_t)r2 << 16 | (uint32_t)g2 << 8 | (uint32_t)b2);
	for (auto &p2 : cloudLabel2->points) {
		p2.label = label2;
	}
	for (auto &p2 : cloudLabelRB2->points) {
		p2.label = label2;
		p2.rgb = *reinterpret_cast<float*>(&rgb2);
	}

	Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity();

	// Align Methode
	// output = die registrierte Ergebnis cloud
	// anhand src und tgt wird die Traffomatrix gewonnen
	// dabei wird src immer weiter transformiert bis die übereinstimmung mit tgt einen gewissen grad überschreitet
	// die traffos werden jeweils in einer Matrix festgehalten (s.u. gleich) indem sie in jeder Iteration an die matrix
	// ursprungstraffomatrix multipliziert werden
	// am ende wird tgt=cloudLabel2 anhang der inversen der endtraffomatrix transformiert (also auf source draufgelegt)
	PointCloud<PointL>::Ptr output(new PointCloud<PointL>);
	PointCloud<PointL>::Ptr src(new PointCloud<PointL>);
	PointCloud<PointL>::Ptr tgt(new PointCloud<PointL>);
	
	// sehr grober filter, so wird viel seltener im lokalen maximum festgesteckt da die anfängliche traffo nur
	// viel weniger punkte genau matchen muss
	// verringert auch den rechenaufwand (DUH)
	VoxelGrid<PointL> grid;
	grid.setLeafSize(0.05, 0.05, 0.05);
	grid.setInputCloud(cloudLabel1);
	grid.filter(*src);

	grid.setInputCloud(cloudLabel2);
	grid.filter(*tgt);
	
	// die gefilterten neuen Clouds
	src = cloudLabel1;
	tgt = cloudLabel2;
	
	// Die ICP Instanz
	IterativeClosestPointNonLinear<PointL, PointL> reg;
	// Konvergenzkriterum
	reg.setTransformationEpsilon(1e-6);
	// maximaler abstand 10cm zwischen Datenpunkten (LOL)
	reg.setMaxCorrespondenceDistance(0.1);
	
	// setzt initially intern die input_ variable auf src
	reg.setInputSource(src);
	// setzt target_ intern auf tgt damit die traffo bestimmt werden kann
	reg.setInputTarget(tgt);

	// einige platzhalter matrizzen, alle matrizzen sind für die traffo
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource, final_transform;
	// platzhalter wolke
	PointCloud<PointL>::Ptr reg_result = src;
	// interne maxIterationen stark limitieren, sorgt für erhöhte kontrolle
	reg.setMaximumIterations(2);
	// hier manuell dafür großer loop
	for (int i = 0; i < 30; ++i)
	{
		cout << i << endl;
		// temp src speichern
		src = reg_result;

		// input source aktualisieren
		reg.setInputSource(src);
		cout << "nach reg.input..(..)" << endl;
		// align ruft computeTransformation auf (hier wird die traffo berechnet) und verwendet hier E_4 als startguess
		reg.align(*reg_result);
		cout << "nach reg.align(..)" << endl;
		// in jeder iteration die globale traffo aktualisieren
		Ti = reg.getFinalTransformation() * Ti;

		// zu beginn wird durch maxCorrespondanceDistance=0.1 garantiert dass kein lokales Maximum zum stopp führt
		// also große, gröbere bewegungen der wolke
		// sollte die verschiebung in einem schritt das traffoEpsilon unterschreiten, wird maxCorrespondanceDistance jeweils slighty
		// reduziert um genauere, kleine bewegungen zu ermöglichen
		if (fabs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())
			reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.001);
		cout << "bei prev = shitnstuff" << endl;
		prev = reg.getLastIncrementalTransformation();

		// zu testzwecken am Ende jeder Iteration einmal die aktuell registrierten wolken anschauen
		targetToSource = Ti.inverse();
		transformPointCloud(*cloudLabel2, *output, targetToSource);
		*output += *cloudLabel1;

		std::stringstream ss;
		ss << "registrationTry"<<i<<".ply";
		savePLYFile(ss.str(), *output, true);
	}

	// Get the transformation from target to source
	targetToSource = Ti.inverse();

	// Transform target back in source frame
	transformPointCloud(*cloudLabel2, *output, targetToSource);

	//add the source to the transformed target
	*output += *cloudLabel1;

	final_transform = targetToSource;
	// Ende align

	////transform current pair into the global transform
	//transformPointCloud(*output, *result, GlobalTransform);

	////update the global transform
	//GlobalTransform = GlobalTransform * pairTransform;

	//save aligned pair, transformed into the first cloud's frame
	std::stringstream ss2;
	ss2 << "registrationTryEnd.ply";
	savePLYFile(ss2.str(), *output, true);

}