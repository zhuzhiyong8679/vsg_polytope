#pragma once
#include <vsg/maths/plane.h>
#include <vsg/maths/mat4.h>
#include <iostream>
#include <vsg/app/Camera.h>
#include <vector>
#include <vsg/utils/ComputeBounds.h>
#include <vsg/all.h>

namespace vsg {

	class Polytope
	{
	public:
		using PlaneList = std::vector<dplane>;

		Polytope() {};
		Polytope(const Polytope &cv) : _planeList(cv._planeList) {};
		Polytope(int x, int y) {};

		void add(const vsg::dplane &plane);
		void setToUnitFrustum(bool withNear = true, bool withFar = true);
		void setToBoundingBox(const vsg::ComputeBounds &bb);

		void set(const PlaneList &pl) { _planeList = pl; }
		std::vector<dplane> getPlaneList() { return _planeList; }
		void setAndTransformProvidingInverse(const vsg::dmat4 &matrix);

		/* Check whether a vertex is contained within clipping set.*/
		bool contains(const vsg::dvec3 &v) const;
		bool contains(const vsg::dvec3 &v1, const vsg::dvec3 &v2);
		/* Check whether a line is contained within clipping set.*/
		/* contain one or contain all */
		bool contains(ref_ptr<const vec3Array> vertices, bool isCatchonPoint);
		bool contains(const vsg::dsphere &dsp);

		std::vector<uint32_t> getVertexIndex() { return _containVertexIndexList; }
		std::vector<dvec3>  getVertexList() { return _containVertexList; }
	protected:
		int intersect(const dplane &plane, const dsphere &dsp) const;
	private:
		PlaneList _planeList;
		std::vector<uint32_t> _containVertexIndexList;
		std::vector<dvec3> _containVertexList;
	};

}