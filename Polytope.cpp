#include "Polytope.h"


namespace vsg {
	
	void Polytope::add(const vsg::dplane& plane)
	{
		_planeList.emplace_back(plane);
		
	}
	void Polytope::setAndTransformProvidingInverse(const vsg::dmat4& matrix)
	{	
		for (int i = 0; i < _planeList.size(); ++i)
		{
			_planeList.at(i) =_planeList.at(i)*matrix;
			
		}

	}
	/** Check whether a vertex is contained within clipping set.*/
	bool Polytope::contains(const vsg::dvec3& v) const
	{
		if (_planeList.empty())return false;
		for (int i=0;i< _planeList.size();i++)
		{
			if (vsg::distance(_planeList.at(i), v) < 0.0f)
			{
				
				return false;
			}
		}
		return true;
	}
	/*Check whether a line is contained or cross the plane no parallel*/
	bool Polytope::contains(const vsg::dvec3& v1, const vsg::dvec3& v2,uint32_t &firstIndex, uint32_t&secondIndex)
	{
		if (_planeList.empty())return false;
		for (int i = 0; i < _planeList.size(); i++)
		{
			if (vsg::distance(_planeList.at(i), v1) < 0.0f&& vsg::distance(_planeList.at(i), v2)<0.0f)
			{
				return false;
			}
		}
		_containVertexIndexList.emplace_back(firstIndex);
		_containVertexIndexList.emplace_back(secondIndex);
		_containVertexList.emplace_back(dvec3(v1));
		_containVertexList.emplace_back(dvec3(v2));
		return true;
	}

	bool Polytope::contains(const vsg::dvec3& v1, const vsg::dvec3& v2, const uint16_t& firstIndex,const uint16_t& secondIndex)
	{
		if (_planeList.empty())return false;
		for (int i = 0; i < _planeList.size(); i++)
		{
			if (vsg::distance(_planeList.at(i), v1) < 0.0f && vsg::distance(_planeList.at(i), v2) < 0.0f)
			{
				return false;
			}
		}
		_containVertexIndexList.emplace_back(firstIndex);
		_containVertexIndexList.emplace_back(secondIndex);
		_containVertexList.emplace_back(dvec3(v1));
		_containVertexList.emplace_back(dvec3(v2));
		return true;
	}

	bool Polytope::contains(const vsg::dvec3& v1, const vsg::dvec3& v2, const vsg::dvec3& v3)
	{
		if (_planeList.empty())return false;
		auto Triangle_vert = vsg::dvec3Array::create({ v1,v2,v3 });
		bool suc;
		for (int j = 0; j < Triangle_vert->size(); j++)
		{
			bool inPlane=checkInPlanlist(Triangle_vert->at(j));
			if (inPlane)
			{
				_containVertexList.emplace_back(v1);
				_containVertexList.emplace_back(v2);
				_containVertexList.emplace_back(v3);
				return true;
			}
		}
		
		
	}
	/** Check whether any part of vertex list is contained within clipping set.*/
	bool Polytope::contains(ref_ptr<const vec3Array>vertices,bool checkAll)
	{
		if (_planeList.empty())return false;
		//could get the Index of vertices
		for (int i =0; i<vertices->size();++i)
		{
			const vsg::vec3& v = vertices->at(i);
			bool outside = false;
			
			for (int j = 0; j < _planeList.size();++j)
			{
				auto dis = distance(_planeList.at(j), dvec3(v));
				
				if (dis < 0.0f) outside = true;
			}

			if (!outside)
			{
				_containVertexIndexList.emplace_back(i);
				_containVertexList.emplace_back(dvec3(v));
				/*check one*/
				if (!checkAll)
				{
					return true;
				}
			}

		}
		return !_containVertexList.empty();
	}

	bool Polytope::contains(const vsg::dsphere& dsp)
	{
		if (_planeList.empty())return false;

		for (auto itr = _planeList.begin(); itr != _planeList.end(); itr++)
		{
			if (vsg::distance(*itr, dsp.center) > -dsp.radius)return false;

		}
		return true;

		
	}



}