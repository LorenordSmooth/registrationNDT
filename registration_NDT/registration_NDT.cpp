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

// Bonn paper
#include <pcl/features/fpfh.h>
#include <pcl/features/multiscale_feature_persistence.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/transformation_estimation_point_to_plane_weighted.h>
#include <pcl/registration/default_convergence_criteria.h>
// Ende

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
typedef boost::shared_ptr<MultiscaleFeaturePersistence<PointL, FPFHSignature33> > Ptr;
typedef boost::shared_ptr<const MultiscaleFeaturePersistence<PointL, FPFHSignature33> > ConstPtr;
typedef pcl::PointCloud<FPFHSignature33> FeatureCloud;
typedef typename pcl::PointCloud<FPFHSignature33>::Ptr FeatureCloudPtr;
typedef typename pcl::Feature<PointL, FPFHSignature33>::Ptr FeatureEstimatorPtr;
typedef boost::shared_ptr<const pcl::PointRepresentation <FPFHSignature33> > FeatureRepresentationConstPtr;

// globals
int anzahlClouds = 1;
int anzahlIterationen = 30;
bool NormalDistributionTrans = false;
bool IterativeClosestPoints = true;
bool LeafnodeIterator = true;
bool registrierungFertig = false;

int ladeClouds(int argc, char** argv, vector<PointCloud<PointT>::Ptr, 
	Eigen::aligned_allocator<PointCloud<PointT>::Ptr>> &sourceClouds)
{
	// Einlesen der Dateinamen der 5 Punktewolken
	vector<int> filenames;
	filenames = console::parse_file_extension_argument(argc, argv, ".ply");

	// Lade die clouds in einen cloud-vektor
	for (int i = 0; i < anzahlClouds; ++i)
	{
		PointCloud<PointT>::Ptr cloud(new PointCloud<PointT>());
		if (loadPLYFile(argv[filenames[i]], *cloud) < 0) {
			cout << "Fehler beim Laden der Cloud " << argv[filenames[i]] << endl << endl;
			return -1;
		}
		sourceClouds.push_back(cloud);
		cout << "test" << i << endl;
	}
}

void registerNDT(const PointCloud<PointL>::Ptr cloud_src, const PointCloud<PointL>::Ptr cloud_tgt, 
	PointCloud<PointL>::Ptr output)
{
	//PointCloud<PointL>::Ptr output(new PointCloud<PointL>);
	PointCloud<PointL>::Ptr src(new PointCloud<PointL>);
	PointCloud<PointL>::Ptr tgt(new PointCloud<PointL>);

	VoxelGrid<PointL> grid;
	grid.setLeafSize(0.05, 0.05, 0.05);
	grid.setInputCloud(cloud_src);
	grid.filter(*src);

	grid.setInputCloud(cloud_tgt);
	grid.filter(*tgt);
	cout << "Filtered cloud contains " << tgt->size() << endl;

	// Initializing Normal Distributions Transform (NDT).
	NormalDistributionsTransform<PointL, PointL> ndt;

	// Setting scale dependent NDT parameters
	// Setting minimum transformation difference for termination condition.
	ndt.setTransformationEpsilon(0.01);
	// Setting maximum step size for More-Thuente line search.
	ndt.setStepSize(0.1);
	//Setting Resolution of NDT grid structure (VoxelGridCovariance).
	ndt.setResolution(1.0);

	// Setting max number of registration iterations.
	ndt.setMaximumIterations(35);

	// Setting point cloud to be aligned.
	ndt.setInputSource(src);
	// Setting point cloud to be aligned to.
	ndt.setInputTarget(tgt);

	// Set initial alignment estimate found using robot odometry.
	Eigen::AngleAxisf init_rotation(0, Eigen::Vector3f::UnitZ());
	Eigen::Translation3f init_translation(0, 0, 0);
	Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

	cout << "init_guess " << init_guess << endl;

	// Calculating required rigid transform to align the input cloud to the target cloud.
	ndt.align(*output, init_guess);


	/*cout << "Normal Distributions Transform has converged:" << ndt.hasConverged()
	<< " score: " << ndt.getFitnessScore() << endl;*/

	// Transforming unfiltered, input cloud using found transform.
	transformPointCloud(*cloud_tgt, *output, ndt.getFinalTransformation());

	*output += *cloud_src;
}

void initialTransformMatrixCompute(PointCloud<FPFHSignature33>::Ptr feat_src, 
	boost::shared_ptr<const PointCloud<PointL>> key_src, PointCloud<FPFHSignature33>::Ptr feat_tgt,
	boost::shared_ptr<const PointCloud<PointL>> key_tgt, Eigen::Matrix4f transform)
{
	/*CorrespondencesPtr correspondences(new Correspondences);
	registration::CorrespondenceEstimation<FPFHSignature33, FPFHSignature33> cest;

	cest.setInputSource(feat_src);
	cest.setInputTarget(feat_tgt);
	cest.determineCorrespondences(*correspondences);
	Correspondences corr_filtered;
	registration::CorrespondenceRejectorSampleConsensus<PointL> rejector;
	rejector.setInputSource(key_src);
	rejector.setInputTarget(key_tgt);
	rejector.setInlierThreshold(2.5);
	rejector.setMaximumIterations(1000000);
	rejector.setRefineModel(false);
	rejector.setInputCorrespondences(correspondences);;
	rejector.getCorrespondences(corr_filtered);
	registration::TransformationEstimationSVD<PointL, PointL> trans_est;
	trans_est.estimateRigidTransformation(key_src, key_tgt, corr_filtered, transform);*/
}

void keypointAndDescriptorCompute(const PointCloud<PointL>::Ptr cloud, PointCloud<FPFHSignature33>::Ptr features, 
	boost::shared_ptr<vector<int>> keypoints)
{
	// erstmal normals berechnen kk ty
	//NormalEstimation<PointL, Normal> ne;
	//ne.setInputCloud(cloud);

	//search::KdTree<PointL>::Ptr tree(new search::KdTree<PointL>());
	//ne.setSearchMethod(tree);

	//PointCloud<Normal>::Ptr cloud_normals(new PointCloud<Normal>);

	//ne.setRadiusSearch(0.03);
	//ne.compute(*cloud_normals);

	//// Bonn paper vorberechnungen fuer besseres initial alignement
	//FPFHEstimation<PointL, Normal, FPFHSignature33>::Ptr fest(new FPFHEstimation<PointL, Normal, FPFHSignature33>);
	//fest->setInputCloud(cloud);
	//fest->setInputNormals(cloud_normals);
	//MultiscaleFeaturePersistence<PointL, FPFHSignature33> fper;

	//vector<int> keypoints;
	//vector<float> scale_values = { 0.5f, 1.0f, 1.5f };

	//Feature<PointL, FPFHSignature33>::Ptr fpfh_estimation;
	////vector<int> keypoints;
	////boost::shared_ptr<MultiscaleFeaturePersistence<PointL, FPFHSignature33>> keypoints;
	//fper.setScalesVector(scale_values);
	//fper.setAlpha(1.3f);
	//fper.setFeatureEstimator(fpfh_estimation);
	//fper.setDistanceMetric(pcl::CS);
	//// verstehe nicht welchen datentyp er haben moechte fuer keypoints :(
	//fper.determinePersistentFeatures(*features, keypoints);
}

void initialTransformCrude(const PointCloud<PointL>::Ptr cloud_src, const PointCloud<PointL>::Ptr cloud_tgt, 
	Eigen::Matrix4f &finalTransform, CorrespondencesPtr &corresps_filtered)
{
	// better transform initial saved here
	Eigen::Matrix4f transformInitial;

	CorrespondencesPtr corresps(new Correspondences);
	registration::CorrespondenceEstimation<PointL, PointL> est;
	est.setInputSource(cloud_src);
	est.setInputTarget(cloud_tgt);
	double max_dist = 0.1;
	est.determineCorrespondences(*corresps, max_dist);

	registration::CorrespondenceRejectorDistance rejector;
	rejector.setInputSource<PointL>(cloud_src);
	rejector.setInputTarget<PointL>(cloud_tgt);
	rejector.setInputCorrespondences(corresps);
	rejector.setMaximumDistance(max_dist);
	rejector.getCorrespondences(*corresps_filtered);

	registration::TransformationEstimationSVD<PointL, PointL, float>::Matrix4 R;
	registration::TransformationEstimationSVD<PointL, PointL, float> te;
	//registration::TransformationEstimationPointToPlaneWeighted<PointL, PointL, double> te;
	//te.setWeights(correspondence_weights);
	te.estimateRigidTransformation(*cloud_src, *cloud_tgt, *corresps_filtered, transformInitial);

	cout << "transform: " << transformInitial << endl;
}

void registerICP2(const PointCloud<PointL>::Ptr cloud_src, const PointCloud<PointL>::Ptr cloud_tgt, 
	const Eigen::Matrix4f transformInitial,
	const CorrespondencesPtr corresps_filtered ,PointCloud<PointL>::Ptr output)
{	
	PointCloud<PointL>::Ptr src(new PointCloud<PointL>);
	PointCloud<PointL>::Ptr tgt(new PointCloud<PointL>);

	VoxelGrid<PointL> grid;
	grid.setLeafSize(0.05, 0.05, 0.05);
	grid.setInputCloud(cloud_src);
	grid.filter(*src);

	grid.setInputCloud(cloud_tgt);
	grid.filter(*tgt);
	cout << "Filtered cloud contains " << tgt->size() << endl;

	IterativeClosestPointNonLinear<PointL, PointL> reg;
	///*reg.setTransformationEpsilon(1e-6);
	//reg.setMaxCorrespondenceDistance(0.1);*/

	reg.setInputSource(src);
	reg.setInputTarget(tgt);

	int iterations = 0;

	// der konstruktor kriegt die per reference geschickt, heißt das die werden zwischen durch updated? (muss ja eigl..)
	registration::DefaultConvergenceCriteria<float> conv_crit(iterations, transformInitial, *corresps_filtered);
	
	conv_crit.setMaximumIterations(30);
	conv_crit.setMaximumIterationsSimilarTransforms(3);
	conv_crit.setTranslationThreshold(5e-3);
	conv_crit.setRotationThreshold(cos(0.5 * M_PI / 180.0));
	conv_crit.setRelativeMSE(0.01);
	int i = 0;
	do
	{
		cout << "iteration" << i << endl;
		cout << "transform: " << endl;
		cout << transformInitial << endl;
		++i;
		reg.align(*output, transformInitial);
	} while (!conv_crit.hasConverged());
	registration::DefaultConvergenceCriteria<float>::ConvergenceState why_conv = conv_crit.getConvergenceState();
	cout << "konvergiert wegen: " << why_conv << endl;
}

// input: 1.src, 2.tgt, 3.startTraffo, 4.start?corresp, 5.resultat der methode, 6.resultat der traffo dieser methode als zwischenergebnis
void registerICP(const PointCloud<PointL>::Ptr cloud_src, const PointCloud<PointL>::Ptr cloud_tgt,
	Eigen::Matrix4f transformInitial,
	const CorrespondencesPtr corresps_filtered, PointCloud<PointL>::Ptr output, Eigen::Matrix4f &finalTransform)
{
	Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity();

	// Align Methode
	// output = die registrierte Ergebnis cloud
	// anhand src und tgt wird die Traffomatrix gewonnen
	// dabei wird src immer weiter transformiert bis die übereinstimmung mit tgt einen gewissen grad überschreitet
	// die traffos werden jeweils in einer Matrix festgehalten (s.u. gleich) indem sie in jeder Iteration an die matrix
	// ursprungstraffomatrix multipliziert werden
	// am ende wird tgt=cloudLabel2 anhang der inversen der endtraffomatrix transformiert (also auf source draufgelegt)
	//PointCloud<PointL>::Ptr result(new PointCloud<PointL>);
	PointCloud<PointL>::Ptr src(new PointCloud<PointL>);
	PointCloud<PointL>::Ptr tgt(new PointCloud<PointL>);

	// sehr grober filter, so wird viel seltener im lokalen maximum festgesteckt da die anfängliche traffo nur
	// viel weniger punkte genau matchen muss
	// verringert auch den rechenaufwand (DUH)
	VoxelGrid<PointL> grid;
	grid.setLeafSize(0.05, 0.05, 0.05);
	grid.setInputCloud(cloud_src);
	grid.filter(*src);

	grid.setInputCloud(cloud_tgt);
	grid.filter(*tgt);
	cout << "Filtered cloud contains " << tgt->size() << endl;

	// die ungefilterten Clouds
	/*src = cloudLabelRB1;
	tgt = cloudLabelRB2;*/

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
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;

	// platzhalter wolke
	PointCloud<PointL>::Ptr reg_result = src;
	// interne maxIterationen stark limitieren, sorgt für erhöhte kontrolle
	reg.setMaximumIterations(2);
	// hier manuell dafür großer loop
	for (int i = 0; i < anzahlIterationen; ++i)
	{
		cout << i << endl;
		// temp src speichern
		src = reg_result;

		// input source aktualisieren
		reg.setInputSource(src);
		cout << "nach reg.input..(..)" << endl;
		// align ruft computeTransformation auf (hier wird die traffo berechnet) und verwendet hier E_4 als startguess
		// die traffo(Matrix) wird komplett intern berechnet getFinalTransformation() liefert die jeweilige Matrix im
		// aktuellen schritt
		// wenn man einen start guess verwendet, wird dieser nur einmalig am anfang auf die wolke angewendet, dient
		// nicht als basis der weiteren traffo
		reg.align(*reg_result);
		cout << "nach reg.align(..)" << endl;
		// in jeder iteration die globale traffo aktualisieren
		Ti = reg.getFinalTransformation() * Ti;

		/*cout << "transform: getFinal" << endl;
		cout << reg.getFinalTransformation() << endl;*/

		// zu beginn wird durch maxCorrespondanceDistance=0.1 garantiert dass kein lokales Maximum zum stopp führt
		// also große, gröbere bewegungen der wolke
		// sollte die verschiebung in einem schritt das traffoEpsilon unterschreiten, wird maxCorrespondanceDistance jeweils slighty
		// reduziert um genauere, kleine bewegungen zu ermöglichen
		if (fabs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())
			reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.001);
		cout << "bei prev = shitnstuff" << endl;
		prev = reg.getLastIncrementalTransformation();

		// zu testzwecken am Ende jeder Iteration einmal die aktuell registrierten wolken anschauen
		// targetToSource ist die transformation
		targetToSource = Ti.inverse();
		/*cout << "transform: targetToSource" << endl;
		cout << targetToSource << endl;*/
		transformPointCloud(*cloud_tgt, *output, targetToSource);
		*output += *cloud_src;

		/*if (i = 5 || 10 || 20) {
			std::stringstream ss;
			ss << "registrationTry" << i << ".ply";
			savePLYFile(ss.str(), *output, true);
		}*/
		
	}

	// Get the transformation from target to source
	targetToSource = Ti.inverse();

	// Transform target back in source frame
	// transformiere cloud_tgt anhand targetToSource, speichere resultat in output
	transformPointCloud(*cloud_tgt, *output, targetToSource);

	//add the source to the transformed target
	// fuege untouched cloud_src zu transformiert cloud_tgt(=output) hinzu
	*output += *cloud_src;

	finalTransform = targetToSource;
}

int main(int argc, char** argv)
{
	// Base Clouds
	PointCloud<PointT>::Ptr cloud1(new PointCloud<PointT>());
	PointCloud<PointT>::Ptr cloud2(new PointCloud<PointT>());
	PointCloud<PointT>::Ptr cloud3(new PointCloud<PointT>());
	PointCloud<PointT>::Ptr cloud4(new PointCloud<PointT>());
	PointCloud<PointT>::Ptr cloud5(new PointCloud<PointT>());
	// Clouds mit Label
	PointCloud<PointL>::Ptr cloudLabel1(new PointCloud<PointL>());
	PointCloud<PointL>::Ptr cloudLabel2(new PointCloud<PointL>());
	PointCloud<PointL>::Ptr cloudLabel3(new PointCloud<PointL>());
	PointCloud<PointL>::Ptr cloudLabel4(new PointCloud<PointL>());
	PointCloud<PointL>::Ptr cloudLabel5(new PointCloud<PointL>());
	// Clouds mit Label in RotBlauGruenGelb Darstellung
	PointCloud<PointL>::Ptr cloudLabelRB1(new PointCloud<PointL>());
	PointCloud<PointL>::Ptr cloudLabelRB2(new PointCloud<PointL>());
	PointCloud<PointL>::Ptr cloudLabelRB3(new PointCloud<PointL>());
	PointCloud<PointL>::Ptr cloudLabelRB4(new PointCloud<PointL>());
	PointCloud<PointL>::Ptr cloudLabelRB5(new PointCloud<PointL>());
	// resultat der registrierung von cloud 1&2&3&4
	PointCloud<PointL>::Ptr result12(new PointCloud<PointL>());
	PointCloud<PointL>::Ptr result123(new PointCloud<PointL>());
	PointCloud<PointL>::Ptr result1234(new PointCloud<PointL>());

	// lade die clouds in einen vector von clouds
	vector<PointCloud<PointT>::Ptr, Eigen::aligned_allocator<PointCloud<PointT>::Ptr>> clouds;
	ladeClouds(argc, argv, clouds);

	// initialisiere die arbeits clouds (uebersichtlicher)
	switch (anzahlClouds) {
		case 1: {
			*cloud1 = *clouds[0];
			// wenn wir nur eine Cloud laden, ist dies die bereits fertig registrierte
			registrierungFertig = true;
			break;
		}
		case 2: {
			*cloud1 = *clouds[0];
			*cloud2 = *clouds[1];
			break;
		}
		case 3: {
			*cloud1 = *clouds[0];
			*cloud2 = *clouds[1];
			*cloud3 = *clouds[2];
			break;
		}
		case 4: {
			*cloud1 = *clouds[0];
			*cloud2 = *clouds[1];
			*cloud3 = *clouds[2];
			*cloud4 = *clouds[3];
			break;
		}
		case 5: {
			*cloud1 = *clouds[0];
			*cloud2 = *clouds[1];
			*cloud3 = *clouds[2];
			*cloud4 = *clouds[3];
			*cloud5 = *clouds[4];
			break;
		}
	}

	// wenn registrierungFertig true, ueberspringe das Labeln und die RegistrierungsMethoden, direkt zu LeafIter
	if (!registrierungFertig)
	{

		cout << "vor pointcloud copy shiet" << endl;
		// temporaer: konvertiere zwei (beliebige) eingelesenen Clouds in (neue) XYZRGBL Clouds
		copyPointCloud(*cloud1, *cloudLabel1);
		copyPointCloud(*cloud2, *cloudLabel2);
		copyPointCloud(*cloud3, *cloudLabel3);
		copyPointCloud(*cloud4, *cloudLabel4);
		copyPointCloud(*cloud5, *cloudLabel5);
		cout << "Nach pointcloud copy shiet" << endl;

		// Initialisiere RotBlau Clouds
		*cloudLabelRB1 = *cloudLabel1;
		*cloudLabelRB2 = *cloudLabel2;
		*cloudLabelRB3 = *cloudLabel3;
		*cloudLabelRB4 = *cloudLabel4;
		*cloudLabelRB5 = *cloudLabel5;

		// Initialisiere Label
		uint32_t label1 = 1;
		uint32_t label2 = 2;
		uint32_t label3 = 3;
		uint32_t label4 = 4;
		uint32_t label5 = 5;

		cout << "vor pointcloud label shiet" << endl;
		for (auto &p1 : cloudLabel1->points) {
			p1.label = label1;
		}
		for (auto &p2 : cloudLabel2->points) {
			p2.label = label2;
		}
		for (auto &p3 : cloudLabel3->points) {
			p3.label = label3;
		}
		for (auto &p4 : cloudLabel4->points) {
			p4.label = label4;
		}
		for (auto &p5 : cloudLabel5->points) {
			p5.label = label5;
		}
		cout << "nach pointcloud label shiet" << endl;

		// rufe registrierung per ICP auf
		if (IterativeClosestPoints)
		{
			// source features n keypoints kk
			//PointCloud<FPFHSignature33>::Ptr features1; 
			////vector<int> keypoints1;
			//boost::shared_ptr<vector<int>> keypoints1;
			//// target same same
			//PointCloud<FPFHSignature33>::Ptr features2;
			//boost::shared_ptr<vector<int>> keypoints2;

			// vorberechnungen fuer besser alignment etc
			// eingabe: uno cloud, ausgabe: features&keypoints hell yeah
			/*keypointAndDescriptorCompute(cloudLabel1, features1, keypoints1);
			keypointAndDescriptorCompute(cloudLabel2, features2, keypoints2);

			initialTransformMatrixCompute(features1, keypoints1, features2, keypoints2, transformInitial);*/

			Eigen::Matrix4f startTransform;
			CorrespondencesPtr corrFiltered(new Correspondences);

			Eigen::Matrix4f result12Transform;
			Eigen::Matrix4f result123Transform;
			Eigen::Matrix4f result1234Transform;
			initialTransformCrude(cloudLabel1, cloudLabel2, startTransform, corrFiltered);
			registerICP(cloudLabel1, cloudLabel2, startTransform, corrFiltered, result12, result12Transform);

			std::stringstream ss3;
			ss3 << "registration12.ply";
			savePLYFile(ss3.str(), *result12, true);

			// ergebnis erste registrierung wird neue src cloud
			registerICP(result12, cloudLabel3, result12Transform, corrFiltered, result123, result123Transform);

			std::stringstream ss4;
			ss4 << "registration123.ply";
			savePLYFile(ss4.str(), *result123, true);

			// ergebnis zweite registrierung wird neue src cloud
			registerICP(result123, cloudLabel4, result123Transform, corrFiltered, result1234, result1234Transform);

			std::stringstream ss5;
			ss5 << "registration1234.ply";
			savePLYFile(ss5.str(), *result1234, true);

			// Bonn Paper ICP
			//registerICP2(cloudLabel1, cloudLabel2, result);

			//// transform current pair into the global transform
			//transformPointCloud(*output, *result, GlobalTransform);

			//// update the global transform
			//GlobalTransform = GlobalTransform * final_transform;
		}

		// rufe registrierung per NDT auf
		else if (NormalDistributionTrans)
		{
			registerNDT(cloudLabel1, cloudLabel2, result12);
		}

		/* Legende: Cloud 1 rot
					Cloud 2 gruen
					Cloud 3 blau
					Cloud 4 gelb
					Cloud 5 lila */

		uint8_t r1 = 255, g1 = 0, b1 = 0;
		uint32_t rgb1 = ((uint32_t)r1 << 16 | (uint32_t)g1 << 8 | (uint32_t)b1);
		uint8_t r2 = 0, g2 = 255, b2 = 0;
		uint32_t rgb2 = ((uint32_t)r2 << 16 | (uint32_t)g2 << 8 | (uint32_t)b2);
		uint8_t r3 = 0, g3 = 0, b3 = 255;
		uint32_t rgb3 = ((uint32_t)r3 << 16 | (uint32_t)g3 << 8 | (uint32_t)b3);
		uint8_t r4 = 255, g4 = 255, b4 = 0;
		uint32_t rgb4 = ((uint32_t)r4 << 16 | (uint32_t)g4 << 8 | (uint32_t)b4);
		uint8_t r5 = 255, g5 = 0, b5 = 255;
		uint32_t rgb5 = ((uint32_t)r5 << 16 | (uint32_t)g5 << 8 | (uint32_t)b5);

		for (size_t i = 0; i < result1234->points.size(); ++i) {
			switch (result1234->points[i].label) {
			case 1: {
				result1234->points[i].rgb = *reinterpret_cast<float*>(&rgb1);
				break;
			}
			case 2: {
				result1234->points[i].rgb = *reinterpret_cast<float*>(&rgb2);
				break;
			}
			case 3: {
				result1234->points[i].rgb = *reinterpret_cast<float*>(&rgb3);
				break;
			}
			case 4: {
				result1234->points[i].rgb = *reinterpret_cast<float*>(&rgb4);
				break;
			}
			case 5: {
				result1234->points[i].rgb = *reinterpret_cast<float*>(&rgb5);
				break;
			}
			}
		}

		// Registrierte Cloud in RotBlauGruenGelb Darstellung
		std::stringstream ss6;
		ss6 << "registration1234RB.ply";
		savePLYFile(ss6.str(), *result1234, true);

	}

	// Registrierung ist fertig, d.h. wir definieren result1234 als clouds[0]=cloud1
	else
	{
		copyPointCloud(*cloud1, *result1234);
		/*for (int i = 0; i < 30000; ++i)
		{
			cout << "farbe aus reg wolke: " << result1234->points[i].rgb << endl;
		}*/

		/*uint8_t r1 = 255, g1 = 0, b1 = 0;
		uint32_t rgb1 = ((uint32_t)r1 << 16 | (uint32_t)g1 << 8 | (uint32_t)b1);
		uint8_t r2 = 0, g2 = 255, b2 = 0;
		uint32_t rgb2 = ((uint32_t)r2 << 16 | (uint32_t)g2 << 8 | (uint32_t)b2);
		uint8_t r3 = 0, g3 = 0, b3 = 255;
		uint32_t rgb3 = ((uint32_t)r3 << 16 | (uint32_t)g3 << 8 | (uint32_t)b3);
		uint8_t r4 = 255, g4 = 255, b4 = 0;
		uint32_t rgb4 = ((uint32_t)r4 << 16 | (uint32_t)g4 << 8 | (uint32_t)b4);
		uint8_t r5 = 255, g5 = 0, b5 = 255;
		uint32_t rgb5 = ((uint32_t)r5 << 16 | (uint32_t)g5 << 8 | (uint32_t)b5);*/

		/*for (size_t i = 0; i < result1234->points.size(); ++i) {
			switch (result1234->points[i].label) {
			case 1: {
				cloudFilteredFarbig->points[i].rgb = *reinterpret_cast<float*>(&rgb1);
				break;
			}
			case 2: {
				cloudFilteredFarbig->points[i].rgb = *reinterpret_cast<float*>(&rgb2);
				break;
			}
			case 3: {
				cloudFilteredFarbig->points[i].rgb = *reinterpret_cast<float*>(&rgb3);
				break;
			}
			case 4: {
				cloudFilteredFarbig->points[i].rgb = *reinterpret_cast<float*>(&rgb4);
				break;
			}
			case 5: {
				cloudFilteredFarbig->points[i].rgb = *reinterpret_cast<float*>(&rgb5);
				break;
			}
			}
		}*/

	}

	PointCloud<PointXYZRGBL>::Ptr cloudMergedFarbig(new PointCloud<PointXYZRGBL>());
	PointCloud<PointXYZRGBL>::Ptr cloudMergedRotBlau(new PointCloud<PointXYZRGBL>());
	PointCloud<PointXYZRGBL>::Ptr cloudFilteredFarbig(new PointCloud<PointXYZRGBL>());
	PointCloud<PointXYZRGBL>::Ptr cloudFilteredRotBlau(new PointCloud<PointXYZRGBL>());
	if (LeafnodeIterator)
	{
		*cloudMergedFarbig = *result1234;

		// Tiefes des Baumes (standard scheint m, moeglicherweise immer im Bezug auf Quelldaten)
		float resolution = 0.03f;

		// Octree auf gemergte Pointcloud
		OctreePointCloud<PointXYZRGBL> octreeFarbig(resolution);

		//octree.max_objs_per_leaf_ = (size_t)50000;

		octreeFarbig.setInputCloud(cloudMergedFarbig);
		// BoundingBox muss vor addPoints ausgelesen werden, Begruendung unklar aber durch Tests bestaetigt
		// sowohl defineBoudingBox() als auch octree.defineBoundingBox(10.0f) liefern gewuenschte Resultate
		octreeFarbig.defineBoundingBox();
		//octree.defineBoundingBox(10.0f);
		octreeFarbig.addPointsFromInputCloud();

		OctreePointCloud<PointXYZRGBL>::LeafNodeIterator iterFarbig(&octreeFarbig);

		// Vector speichert Indizes von Punkten im aktuellen Leafnode
		vector<int> indexVector;
		// besser wieder array von vectoren obv (later, feinarbeit)
		vector<int> indexVectorCloud1;
		vector<int> indexVectorCloud2;
		vector<int> indexVectorCloud3;
		vector<int> indexVectorCloud4;
		vector<int> indexVectorCloud5;
		// Aus jedem Leafnode werden die relevanten Nodes in gesamtIndices gespeichert
		vector<int> gesamtIndices;

		// Iteriere ueber alle Leafnodes
		for (iterFarbig = octreeFarbig.leaf_begin(); iterFarbig != octreeFarbig.leaf_end(); ++iterFarbig)
		{

			indexVector = iterFarbig.getLeafContainer().getPointIndicesVector();

			// Ueberpruefe bei jedem Punkt im Leafnode, welches Label er hat
			// Die Pointcloud mit mehr Punkten im Leafnode wird praeferiert
			// Zu Testzwecken erstmal einfach Ausgabe der Punkte im Vektor
			for (size_t i = 0; i < indexVector.size(); ++i)
			{
				// bessere implementierung (schoener): array[0] = counterCloud1 bis array[4] = counterCloud4, dann schleife ueber array.size
				int counterCloud1 = 0;
				int counterCloud2 = 0;
				int counterCloud3 = 0;
				int counterCloud4 = 0;
				int counterCloud5 = 0;

				if (cloudMergedFarbig->points[indexVector[i]].label == 1) {
					counterCloud1++;
					// jeden index aus Cloud1 in indexVectorCloud1 hinzufuegen
					indexVectorCloud1.push_back(indexVector[i]);
				}
				else if (cloudMergedFarbig->points[indexVector[i]].label == 2) {
					counterCloud2++;
					// jeden index aus Cloud1 in indexVectorCloud1 hinzufuegen
					indexVectorCloud2.push_back(indexVector[i]);
				}
				else if (cloudMergedFarbig->points[indexVector[i]].label == 3) {
					counterCloud3++;
					// jeden index aus Cloud1 in indexVectorCloud1 hinzufuegen
					indexVectorCloud3.push_back(indexVector[i]);
				}
				else if (cloudMergedFarbig->points[indexVector[i]].label == 4) {
					counterCloud4++;
					// jeden index aus Cloud1 in indexVectorCloud1 hinzufuegen
					indexVectorCloud4.push_back(indexVector[i]);
				}
				else {
					counterCloud5++;
					// analog fuer Cloud2
					indexVectorCloud5.push_back(indexVector[i]);
				}
			}

			// fuege neue Punkte hinten an
			int temp = gesamtIndices.size();

			// die Cloud mit der hoechsten Menge an Punkten soll die Punkte "stellen"
			int maxIndexSize = max({indexVectorCloud1.size(),indexVectorCloud2.size(),indexVectorCloud3.size(),indexVectorCloud4.size(),indexVectorCloud5.size()});

			// (unschoen) finde heraus zu welcher cloud maxIndexSize gehoert, fuege alle diese Punkte zum relevanten Punkte Pool hinzu
			if (maxIndexSize == indexVectorCloud1.size()) {
				for (int i = 0; i < indexVectorCloud1.size(); ++i) {
					gesamtIndices.push_back(indexVectorCloud1[i]);
				}
			}	
			else if (maxIndexSize == indexVectorCloud2.size()) {
				for (int i = 0; i < indexVectorCloud2.size(); ++i) {
					gesamtIndices.push_back(indexVectorCloud2[i]);
				}
			}
			else if (maxIndexSize == indexVectorCloud3.size()) {
				for (int i = 0; i < indexVectorCloud3.size(); ++i) {
					gesamtIndices.push_back(indexVectorCloud3[i]);
				}
			}
			else if (maxIndexSize == indexVectorCloud4.size()) {
				for (int i = 0; i < indexVectorCloud4.size(); ++i) {
					gesamtIndices.push_back(indexVectorCloud4[i]);
				}
			}
			else {
				for (int i = 0; i < indexVectorCloud5.size(); ++i) {
					gesamtIndices.push_back(indexVectorCloud5[i]);
				}
			}
			indexVectorCloud1.clear();
			indexVectorCloud2.clear();
			indexVectorCloud3.clear();
			indexVectorCloud4.clear();
			indexVectorCloud5.clear();
		}

		// gesamtIndices erhaelt nun die gefilterte cloudMergedFarbig, diese Indices gilt es nun wieder in eine Cloud zu ueberfuehren
		for (size_t i = 0; i < gesamtIndices.size(); ++i) {
			cloudFilteredFarbig->push_back(cloudMergedFarbig->points[gesamtIndices[i]]);
		}
	}

	// Registrierte Cloud nach LeafIteration Filter
	std::stringstream ss7;
	ss7 << "registration1234LeafIter.ply";
	savePLYFile(ss7.str(), *cloudFilteredFarbig, true);

	/* Legende: Cloud 1 rot
				Cloud 2 gruen
				Cloud 3 blau
				Cloud 4 gelb
				Cloud 5 lila */

	uint8_t r1 = 255, g1 = 0, b1 = 0;
	uint32_t rgb1 = ((uint32_t)r1 << 16 | (uint32_t)g1 << 8 | (uint32_t)b1);
	uint8_t r2 = 0, g2 = 255, b2 = 0;
	uint32_t rgb2 = ((uint32_t)r2 << 16 | (uint32_t)g2 << 8 | (uint32_t)b2);
	uint8_t r3 = 0, g3 = 0, b3 = 255;
	uint32_t rgb3 = ((uint32_t)r3 << 16 | (uint32_t)g3 << 8 | (uint32_t)b3);
	uint8_t r4 = 255, g4 = 255, b4 = 0;
	uint32_t rgb4 = ((uint32_t)r4 << 16 | (uint32_t)g4 << 8 | (uint32_t)b4);
	uint8_t r5 = 255, g5 = 0, b5 = 255;
	uint32_t rgb5 = ((uint32_t)r5 << 16 | (uint32_t)g5 << 8 | (uint32_t)b5);
	
	for (size_t i = 0; i < cloudFilteredFarbig->points.size(); ++i) {
		switch (cloudFilteredFarbig->points[i].label) {
		case 1: {
			cloudFilteredFarbig->points[i].rgb = *reinterpret_cast<float*>(&rgb1);
			break;
		}
		case 2: {
			cloudFilteredFarbig->points[i].rgb = *reinterpret_cast<float*>(&rgb2);
			break;
		}
		case 3: {
			cloudFilteredFarbig->points[i].rgb = *reinterpret_cast<float*>(&rgb3);
			break;
		}
		case 4: {
			cloudFilteredFarbig->points[i].rgb = *reinterpret_cast<float*>(&rgb4);
			break;
		}
		case 5: {
			cloudFilteredFarbig->points[i].rgb = *reinterpret_cast<float*>(&rgb5);
			break;
		}
		}
	}

	// Registrierte Cloud nach LeafIteration Filter in RotBlauGruenGelb Darstellung
	std::stringstream ss8;
	ss8 << "registration1234LeafIterRB.ply";
	savePLYFile(ss8.str(), *cloudFilteredFarbig, true);
}