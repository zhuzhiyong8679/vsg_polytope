#include "Polytope.h"


namespace vsg {

	void Polytope::add(const vsg::dplane &plane)
	{
		_planeList.emplace_back(plane);
	}

	void Polytope::setToUnitFrustum(bool withNear, bool withFar)
	{
		_planeList.clear();
		_planeList.emplace_back(dplane(1.0, 0.0, 0.0, 1.0)); // left plane.
		_planeList.emplace_back(dplane(-1.0, 0.0, 0.0, 1.0)); // right plane.
		_planeList.emplace_back(dplane(0.0, 1.0, 0.0, 1.0)); // bottom plane.
		_planeList.emplace_back(dplane(0.0, -1.0, 0.0, 1.0)); // top plane.
		if (withNear) _planeList.emplace_back(dplane(0.0, 0.0, 1.0, 1.0)); // near plane
		if (withFar) _planeList.emplace_back(dplane(0.0, 0.0, -1.0, 1.0)); // far plane
	}

	void Polytope::setToBoundingBox(const vsg::ComputeBounds &bb)
	{
		_planeList.clear();
		_planeList.emplace_back(dplane(1.0, 0.0, 0.0, -bb.bounds.min.x)); // left plane.
		_planeList.emplace_back(dplane(-1.0, 0.0, 0.0, bb.bounds.max.y)); // right plane.
		_planeList.emplace_back(dplane(0.0, 1.0, 0.0, -bb.bounds.min.y)); // bottom plane.
		_planeList.emplace_back(dplane(0.0, -1.0, 0.0, bb.bounds.max.y)); // top plane.
		_planeList.emplace_back(dplane(0.0, 0.0, 1.0, -bb.bounds.min.z)); // near plane
		_planeList.emplace_back(dplane(0.0, 0.0, -1.0, bb.bounds.max.z)); // far plane
	}

	void Polytope::setAndTransformProvidingInverse(const vsg::dmat4 &matrix)
	{
		for (int i = 0; i < _planeList.size(); ++i)
		{
			_planeList.at(i) = _planeList.at(i) * matrix;
		}
	}
	/** Check whether a vertex is contained within clipping set.*/
	bool Polytope::contains(const vsg::dvec3 &v) const
	{
		if (_planeList.empty())
			return false;

		for (int i = 0; i < _planeList.size(); i++)
		{
			if (vsg::distance(_planeList.at(i), v) < 0.0f)
			{
				return false;
			}
		}
		return true;
	}

	bool Polytope::contains(const vsg::dvec3 &v1, const vsg::dvec3 &v2)
	{
		if (_planeList.empty())
			return false;

		for (int i = 0; i < _planeList.size(); i++)
		{
			if (vsg::distance(_planeList.at(i), v1) < 0.0f)
			{
				return false;
			}
		}
		return false;
	}

	/** Check whether any part of vertex list is contained within clipping set.*/
	bool Polytope::contains(ref_ptr<const vec3Array>vertices, bool iscatchOne)
	{
		if (_planeList.empty()) return false;
		//could get the Index of vertices
		for (int i = 0; i < vertices->size(); ++i)
		{
			const vsg::vec3 &v = vertices->at(i);
			bool outside = false;

			for (int j = 0; j < _planeList.size(); ++j)
			{
				auto dis = distance(_planeList.at(j), dvec3(v));

				if (dis < 0.0f) outside = true;
			}

			if (!outside)
			{
				_containVertexIndexList.push_back(i);
				_containVertexList.push_back(dvec3(v));
				/*check one*/
				if (iscatchOne)
				{
					return true;
				}
			}

		}
		return !_containVertexList.empty();
	}

	//包围验证无误
	bool Polytope::contains(const vsg::dsphere &dsp)
	{
		if (_planeList.empty())
			return false;

		return vsg::intersect(_planeList, dsp);
	}
	int Polytope::intersect(const dplane &plane, const dsphere &dsp) const
	{
		double d = distance(plane, dsp.center);

		if (d > dsp.radius) return 1;
		else if (d < -dsp.radius) return -1;
		else return 0;
	}
}