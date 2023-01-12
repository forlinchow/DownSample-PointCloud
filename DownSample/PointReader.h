#pragma once
#include <vector>
#include <list>
#include <algorithm>
#include <string>

#include "definitions.h"
namespace forlin {

	//格式文件头，记录创建时间和版本号等数据
	struct _Header
	{
		std::string Version;				//格式版本号
		std::string CreateTime;				//创建时间
		std::string LastestTime;			//最新修改时间
		std::string AttributeTable;			//属性表
		size_t numOfPoints;					//点数量
		float minx, miny, minz, maxx, maxy, maxz;

	};

	struct _XYZRGB
	{
		float x, y, z;
		unsigned char r, g, b;
		POINTTYPE type() {
			return POINTTYPE::XYZRGB;
		}
	};

	struct _XYZ
	{
		float x, y, z;
		POINTTYPE type() {
			return POINTTYPE::XYZ;
		}
	};

	struct _XYZRGBI
	{
		float x, y, z;
		unsigned char r, g, b,i;
		POINTTYPE type() {
			return POINTTYPE::XYZRGBI;
		}
	};

	template <typename PointT>
	struct subData
	{
		std::list<PointT> data;
	};

	template <typename PointT>
	struct BlockData {
		std::vector<subData<PointT>> _allData;
		std::vector<std::string> files;
	};

	//将vector里的点云转换为Block数据
	template<typename PointT>
	void TransferPointsToBlock(std::vector<PointT>* datas, forlin::BlockData<PointT>* data, std::string outFolder) {
		std::string outdir;
		forlin::Converter<PointT> tC;
		tC.setOutFolder(outFolder);

		int _iterator;
		size_t _end = datas->size();
		for (size_t i = 0; i < _end; i++)
		{
			outdir = tC.PointToIndexOfFYData((*datas)[i].x, (*datas)[i].y, (*datas)[i].z);
			auto index = std::find(data->files.begin(), data->files.end(), outdir);
			if (index != data->files.end())
			{
				//found
				_iterator = std::distance(data->files.begin(), index);
				//data._allData[i].data.push_back(k);
				data->_allData[_iterator].data.push_back((*datas)[i]);
			}
			else {
				//not found
				data->files.push_back(outdir);
				forlin::subData<PointT> newsub;
				data->_allData.push_back(newsub);
				data->_allData[data->_allData.size() - 1].data.push_back((*datas)[i]);
			}
		}
		//std::cout << "processing done!!!!!!!!!!!!" << std::endl;
		std::cout << "buffer data process done!" << std::endl;
	}

	//将block数据写入到FYData中
	template <typename PointT>
	void writeBlockToFYData(BlockData<PointT> * data)
	{
		std::cout << "正在写数据" << std::endl;
		for (int j = 0; j < data->files.size(); j++) {
			FILE *out = fopen(data->files[j].c_str(), "ab+");
			if (out == NULL)
			{
				std::cout << "open "<< data->files[j]<<" error" << std::endl;
				continue;
			}

			for (auto k = data->_allData[j].data.begin(); k != data->_allData[j].data.end(); k++)
			{
				fwrite(&(*k), sizeof(PointT), 1, out);
			}
			fclose(out);
		}
	}

	template<typename PointT>
	class PointReader
	{
	public:
		virtual ~PointReader() {};

		//判断可读取的数据块大小
		virtual int BlockReady() = 0;

		virtual int LeftBlock() = 0;

		virtual int MovedBlocks() = 0;

		virtual int readfile(std::string path, std::string savepath)=0;
		//获取一个数据块
		
		virtual std::vector<PointT> getOneBlock() = 0;

	protected:
		

	};
}