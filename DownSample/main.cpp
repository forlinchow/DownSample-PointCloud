#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/file_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <time.h>
//#include "stuff.h"
#include "FYDataReader.h"
using std::cout;
//向后继续读取字符串
inline void cutStrEnd(char* source, FILE* instream, size_t readnum) {
	char b;
	while (!feof(instream)) {
		fread(&b, sizeof(char), 1, instream);
		source[readnum++] = b;
		if (b == '\n') {
			source[readnum] = '\0';
			break;
		}
	}
}

template <typename PointT>
void ReadTxtFile(std::string path, boost::shared_ptr<pcl::PointCloud<PointT>> cloud) {
	LARGE_INTEGER FileSize = forlin::_getFileSize(path);
	if (FileSize.QuadPart < 100) {
		throw "the file is too small to include point clouds";
		exit(-3);
	}

	int cpus = forlin::_getNumOfProcessors() <= 2 ? 1 : forlin::_getNumOfProcessors();

	//打开文件流
	std::cout << "opening file " << path << std::endl;
	FILE *in = fopen(path.c_str(), "r");
	if (in == NULL) {
		exit(1);
	}
	std::cout << "opening success." << std::endl;

	std::vector<char*> buffer(cpus);
	long long each_datasize = ceil(FileSize.QuadPart / (cpus));
	size_t readnum;
	//数据读入处理
	for (int cc = 0; cc < buffer.size(); cc++) {
		buffer[cc] = (char*)malloc(each_datasize + 100);
		readnum = fread(buffer[cc], sizeof(char), each_datasize, in);
		buffer[cc][readnum] = '\0';
		cutStrEnd(buffer[cc], in, readnum);
	}
	fclose(in);
	std::cout << "data has read ino buffer" << std::endl;

	for (int cc = 0; cc < buffer.size(); cc++) {
		size_t bufferlen = strlen(buffer[cc]);
		int linelens = 0;
		char* line = (char*)malloc(100);
		pcl::PointXYZRGB t;
		for (size_t _read = 0; _read < bufferlen;) {
			linelens = strcspn(buffer[cc], "\n");
			strncpy(line, buffer[cc], linelens);
			if (strlen(line) < 1)
			{
				break;
			}
			line[linelens] = '\0';
			if (sscanf(line, "%f,%f,%f,%hhd,%hhd,%hhd", &t.x, &t.y, &t.z, &t.r, &t.g, &t.b) == 0)
			{
				break;
			}
			cloud->points.push_back(t);
			buffer[cc] = buffer[cc] + linelens + 1;
			_read += linelens + 1;
		}
	}
}

template <typename PointT>
void SorFilter(float stdv, int points, boost::shared_ptr<pcl::PointCloud<PointT>> cloud, boost::shared_ptr<pcl::PointCloud<PointT>> cloud_filtered, boost::shared_ptr<pcl::PointCloud<PointT>> cloud_outlies) {
	std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
		<< " data points (" << pcl::getFieldsList(*cloud) << ").";

	pcl::StatisticalOutlierRemoval<PointT> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(points);
	sor.setStddevMulThresh(stdv);
	sor.filter(*cloud_filtered);
	std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
		<< " data points (" << pcl::getFieldsList(*cloud_filtered) << ").";

	sor.setNegative(true);
	sor.filter(*cloud_outlies);
}


template <typename PointT>
void DownSample(float SpaceDistance, boost::shared_ptr<pcl::PointCloud<PointT>> cloud, boost::shared_ptr<pcl::PointCloud<PointT>> cloud_downsample) {
	std::cerr << "PointCloud before filtering: " << cloud->points.size()
		<< " data points (" << pcl::getFieldsList(*cloud) << ").";
	//建立kdtree
	pcl::KdTreeFLANN<PointT> kdtree;
	kdtree.setInputCloud(cloud);

	float radius = SpaceDistance;

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	std::vector<int> idxForPointCloud(cloud->points.size());

	for (int i = 0; i < idxForPointCloud.size(); i++)
	{
		if (idxForPointCloud[i] == 0) {
			cloud_downsample->points.push_back(cloud->points[i]);
			if (kdtree.radiusSearch(cloud->points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
			{
				for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j)
					idxForPointCloud[pointIdxRadiusSearch[j]] = 1;
			}
		}
	}
	std::cerr << "PointCloud after filtering: " << cloud_downsample->width * cloud_downsample->height
		<< " data points (" << pcl::getFieldsList(*cloud_downsample) << ").";
}


//打印使用方法
void printusage() {
	cout << "Develop by forlin\n"
		"Usage: DownSample.exe PoinCloud_filepath OutputPath SapceParameter\n"
		"eg: DownSample.exe  D:/origin D:/downSample 0.001" << std::endl;
}


int
main(int argc, char** argv)
{
	//1207FYData抽稀测试
	//forlin::FYDataReader<forlin::_XYZRGB> tfy(argv[1], argv[2], atof(argv[3]));
	
	//2019.05.31修改内部代码
	forlin::FYDataReader<forlin::_XYZRGB> tfy(argv[1], argv[2], 0.001);

	tfy.MultiThreadRead1();
	//system("pause");

// 	if (argc<5)
// 	{
// 		cout << " Please input the specific parameters" << std::endl;
// 		printusage();
// 		return -1;
// 	}
// 
// 	if (!forlin::iEndsWith(argv[2],"ply")&&!forlin::iEndsWith(argv[3],"ply")&&!forlin::iEndsWith(argv[2],"txt"))
// 	{
// 		cout << "only support ply and txt file in this version." << std::endl;
// 		return -1;
// 	}
// 	if (atoi(argv[1])==1 && (atof(argv[4])<0.001 || atof(argv[4])>0.5))
// 	{
// 		cout << "Please set the SpaceParameter between 0.001 and 0.5" << std::endl;
// 		return -1;
// 	}
// 	if (atoi(argv[1])==2 && (atoi(argv[5])<9 || atoi(argv[5])>50) && (atoi(argv[6])<0.1 || atoi(argv[6])>1))
// 	{
// 		cout << "parameter 5 should be 9 to 50, while parameter 6 should be 0.1 to 1" << std::endl;
// 	}
// 
// 	clock_t start_t = clock();
// 	if (atoi(argv[1])==1)
// 	{
// 		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
// 		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsample(new pcl::PointCloud<pcl::PointXYZRGB>);
// 
// 		if (forlin::iEndsWith(argv[2], "ply")) {
// 			pcl::PLYReader reader_ply;
// 			reader_ply.read<pcl::PointXYZRGB>(argv[2], *cloud);
// 		}
// 		else if (forlin::iEndsWith(argv[2],"txt")){
// 			ReadTxtFile(argv[2], cloud);
// 		}
// 		DownSample<pcl::PointXYZRGB>(atof(argv[4]), cloud, cloud_downsample);
// 
// 		pcl::PLYWriter writer;
// 		writer.write<pcl::PointXYZRGB>(argv[3], *cloud_downsample, true);
// 
// 		std::cout << "time consume: " << (clock() - start_t) / CLOCKS_PER_SEC << std::endl;
// 	}
// 	
// 	if (atoi(argv[1])==2)
// 	{
// 		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
// 		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
// 		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_outlie(new pcl::PointCloud<pcl::PointXYZRGB>);
// 
// 		pcl::PLYReader reader_ply;
// 		reader_ply.read<pcl::PointXYZRGB>(argv[2], *cloud);
// 		SorFilter<pcl::PointXYZRGB>(atof(argv[6]), atoi(argv[5]), cloud, cloud_filtered,cloud_outlie);
// 		pcl::PLYWriter writer;
// 		writer.write<pcl::PointXYZRGB>(argv[3], *cloud_filtered, true);
// 		writer.write<pcl::PointXYZRGB>(argv[4], *cloud_outlie, true);
// 
// 
// 		std::cout << "time consume: " << (clock() - start_t) / CLOCKS_PER_SEC << std::endl;
// 	}
	return (0);
}