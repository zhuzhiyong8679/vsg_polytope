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
				_containVertexIndexList.push_back(i);
				_containVertexList.push_back(dvec3(v));
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
		return vsg::intersect(_planeList, dsp);
	}
}