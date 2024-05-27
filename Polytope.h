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
		typedef std::vector<dplane>    PlaneList;
	

		Polytope(){};
		Polytope(const Polytope& cv) :_planeList(cv._planeList){};
		Polytope(int x, int y){};
		//Polytope(const vsg::Camera& camera) {};
		
		void add(const vsg::dplane&plane);
		void setToUnitFrustum(bool withNear = true, bool withFar = true)
		{
			_planeList.clear();
			_planeList.emplace_back(dplane(1.0, 0.0, 0.0, 1.0)); // left plane.
			_planeList.emplace_back(dplane(-1.0, 0.0, 0.0, 1.0)); // right plane.
			_planeList.emplace_back(dplane(0.0, 1.0, 0.0, 1.0)); // bottom plane.
			_planeList.emplace_back(dplane(0.0, -1.0, 0.0, 1.0)); // top plane.
			if (withNear) _planeList.emplace_back(dplane(0.0, 0.0, 1.0, 1.0)); // near plane
			if (withFar) _planeList.emplace_back(dplane(0.0, 0.0, -1.0, 1.0)); // far plane
		}
		void setToBoundingBox(const vsg::ComputeBounds& bb)
		{
			_planeList.clear();
			_planeList.emplace_back(dplane(1.0, 0.0, 0.0, -bb.bounds.min.x)); // left plane.
			_planeList.emplace_back(dplane(-1.0, 0.0, 0.0, bb.bounds.max.y)); // right plane.
			_planeList.emplace_back(dplane(0.0, 1.0, 0.0, -bb.bounds.min.y)); // bottom plane.
			_planeList.emplace_back(dplane(0.0, -1.0, 0.0, bb.bounds.max.y)); // top plane.
			_planeList.emplace_back(dplane(0.0, 0.0, 1.0, -bb.bounds.min.z)); // near plane
			_planeList.emplace_back(dplane(0.0, 0.0, -1.0, bb.bounds.max.z)); // far plane
		}

		 void set(const PlaneList& pl) { _planeList = pl;  }
		 std::vector<dplane>getPlaneList() { return _planeList; }
		 void setAndTransformProvidingInverse(const vsg::dmat4& matrix);
		 
		 /* Check whether a vertex is contained within clipping set.*/
		 bool contains(const vsg::dvec3& v) const;

		 /* Check whether a line is contained within clipping set.*/
		 /* contain one or contain all */
		 bool contains(ref_ptr<const vec3Array> vertices,bool checkAll);

		 bool contains(const vsg::dsphere& dsp);

		 
		// void makeUintLength();
		
		 std::vector<uint32_t> getVertexIndex() {return _containVertexIndexList;}
		 std::vector<dvec3>  getVertexList() { return _containVertexList; }
	protected:
		 int intersect( const dplane &plnae,const dsphere& bs) const
		{
			float d = distance(plnae,bs.center);
			
			if (d > bs.radius) return 1;
			else if (d < -bs.radius) return -1;
			else return 0;
		}
	private:
		
		PlaneList   _planeList;
		//Camera _camera;
		//dvec3Array  _referenceVertexList;
		std::vector<uint32_t> _containVertexIndexList;
		std::vector<dvec3> _containVertexList;
	};
	
}