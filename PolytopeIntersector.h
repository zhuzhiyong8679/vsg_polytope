#pragma once
#include "Polytope.h"
#include <vsg/app/Camera.h>
#include <vsg/utils/Intersector.h>
#include <vsg/nodes/transform.h>
#include <vector>

namespace vsg {

	/**
	 * @brief This is a class for intersecting a Polytope with a scene graph.
	 * @details This class is used to intersect a Polytope with a scene graph.The Polytope
	 *		is defined by a set of planes, and the scene graph is defined by a set of nodes.
	 *		The intersection is performed by traversing the scene graph and testing each node
	 *		against the Polytope. If the node intersects the Polytope then the node is added to the list of intersections.
	 */
	class VSG_DECLSPEC PolytopeIntersector : public Inherit<Intersector, PolytopeIntersector>
	{
	public:
		PolytopeIntersector(vsg::Polytope &polytope, Camera *camera);
		PolytopeIntersector(Camera *camera, float xMin, float yMin, float xMax, float yMax);

		class VSG_DECLSPEC Intersection : public Inherit<Object, Intersection>
		{
		public:
			Intersection() {}
			Intersection(const std::vector<dvec3> &in_localIntersection, const std::vector<dvec3> &in_worldIntersection, const dmat4 &in_localToWorld, const NodePath &in_nodePath, const DataList &in_arrays, uint32_t in_instanceIndex, std::vector<uint32_t> index);

			std::vector<dvec3> localIntersection;
			std::vector<dvec3> worldIntersection;
			double ratio = 0.0;

			dmat4 localToWorld;
			NodePath nodePath;
			DataList arrays;

			uint32_t m_instanceIndex = 0;
			std::vector<uint32_t> m_pointIndices;
			// return true if Intersection is valid
			operator bool() const { return !nodePath.empty(); }
		};
		using Intersections = std::vector<ref_ptr<Intersection>>;

		Intersections &getIntersections() { return _intersections; }

		Intersections add(const std::vector<dvec3> &coord, uint32_t instanceIndex, std::vector<uint32_t> &index);

		Intersections add(vsg::Polytope &polytope, uint32_t instanceIndex);

		void pushTransform(const Transform &transform) override;

		void popTransform() override;

		/// check for intersection with sphere
		bool intersects(const dsphere &bs) override;

		bool intersectDraw(uint32_t firstVertex, uint32_t vertexCount, uint32_t firstInstance, uint32_t instanceCount) override;
		bool intersectDrawIndexed(uint32_t firstIndex, uint32_t indexCount, uint32_t firstInstance, uint32_t instanceCount) override;

		bool isCatchOnePoint = true;

	protected:
		Intersections             _intersections;
		std::vector<Polytope>     _planeSegmentStack;
		Polytope				  _polytope;
		Camera *_camera = nullptr;
	};

	VSG_type_name(vsg::PolytopeIntersector);
}
