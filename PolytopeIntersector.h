#pragma once
#include "Polytope.h"
#include <vsg/app/Camera.h>
#include <vsg/utils/Intersector.h>
#include <vsg/nodes/transform.h>
#include <vector>

namespace vsg {
	/*//project Model
        auto width=camera->getViewport().width;
        auto height = camera->getViewport().height;
        auto x = camera->getViewport().x;
        auto y = camera->getViewport().y;
        auto xNormal = 2.0f * (pointxy.x - x) / ( width) - 1.0f;
        auto yNormal = 2.0f * (pointxy.y - y) / (height) - 1.0f ;
        auto intersector = vsg::PolytopeIntersector::create(camera, xNormal -0.05, yNormal -0.05, xNormal +0.05, yNormal +0.05);
        */
	class VSG_DECLSPEC PolytopeIntersector : public Inherit<Intersector, PolytopeIntersector>
	{
	public:
		
		PolytopeIntersector( vsg::Polytope& polytope,Camera*camera);
		
		PolytopeIntersector(Camera*camera,float xMin, float yMin, float xMax, float yMax);
		
		class VSG_DECLSPEC Intersection : public Inherit<Object, Intersection>
		{
		public:
			Intersection() {}
			Intersection(const std::vector<dvec3>& in_localIntersection, const std::vector<dvec3>& in_worldIntersection, const dmat4& in_localToWorld, const NodePath& in_nodePath, const DataList& in_arrays, uint32_t in_instanceIndex, std::vector<uint32_t> index);
			
			std::vector <dvec3> localIntersection;
			std::vector<dvec3> worldIntersection;
			double ratio = 0.0;

			dmat4 localToWorld;
			NodePath nodePath;
			DataList arrays;

			uint32_t instanceIndex = 0;
			std::vector<uint32_t> pointIndex;
			// return true if Intersection is valid
			operator bool() const { return !nodePath.empty(); }
		};
		using Intersections = std::vector<ref_ptr<Intersection>>;
		Intersections intersections;
	
		Intersections add(const std::vector<dvec3>& coord, uint32_t instanceIndex, std::vector<uint32_t>& index);

		Intersections add( vsg::Polytope& _poly, uint32_t instanceIndex);

		void pushTransform(const Transform& transform) override;

		void popTransform() override;

		/// check for intersection with sphere
		bool intersects(const dsphere& bs) override;

		bool intersectDraw(uint32_t firstVertex, uint32_t vertexCount, uint32_t firstInstance, uint32_t instanceCount) override;
		bool intersectDrawIndexed(uint32_t firstIndex, uint32_t indexCount, uint32_t firstInstance, uint32_t instanceCount) override;
		

	protected:

		
		
		vsg::dmat4 getTramsFormCamer(const Camera*_camera);
		std::vector<Polytope>     _PlaneSegmentStack;
		Polytope				  _polytope;
		Camera*					   _camera = nullptr;

	};

	VSG_type_name(vsg::PolytopeIntersector);

}
