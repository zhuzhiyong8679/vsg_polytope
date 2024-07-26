#include "PolytopeIntersector.h"

namespace vsg {
	PolytopeIntersector::PolytopeIntersector(vsg::Polytope &polytope, Camera *camera) :_camera(camera)
	{
		_polytope.set(polytope.getPlaneList());
		auto viewport = _camera->getViewport();
		auto projectionMatrix = _camera->projectionMatrix->transform();
		auto viewMatrix = _camera->viewMatrix->transform();
		auto pv = projectionMatrix * viewMatrix;
		_polytope.setAndTransformProvidingInverse(pv);

		_planeSegmentStack.push_back(_polytope);

	}

	//input srceen coord from -1 to 1
	PolytopeIntersector::PolytopeIntersector(Camera *camera, float xMin, float yMin, float xMax, float yMax) :_camera(camera)
	{

		double zNear = 1.0;

		_polytope.add(vsg::dplane(1.0, 0.0, 0.0, -xMin));//left
		_polytope.add(vsg::dplane(-1.0, 0.0, 0.0, xMax));//right
		_polytope.add(vsg::dplane(0.0, 1.0, 0.0, -yMin));//bottom
		_polytope.add(vsg::dplane(0.0, -1.0, 0.0, yMax));//top
		_polytope.add(vsg::dplane(0.0, 0.0, 1.0, zNear));//zNear
		auto viewport = _camera->getViewport();
		auto projectionMatrix = _camera->projectionMatrix->transform();
		auto viewMatrix = _camera->viewMatrix->transform();


		auto pv = projectionMatrix * viewMatrix;
		_polytope.setAndTransformProvidingInverse(pv);
		_planeSegmentStack.emplace_back(_polytope);

	}

	PolytopeIntersector::Intersections PolytopeIntersector::add(vsg::Polytope &_poly, uint32_t instanceIndex)
	{
		auto localToWorld = computeTransform(_nodePath);
		std::vector<dvec3>worldCoords;
		for (size_t i = 0; i < _poly.getVertexList().size(); i++)
		{
			worldCoords.emplace_back(localToWorld * _poly.getVertexList().at(i));
		}
		ref_ptr<Intersection> intersection = Intersection::create(_poly.getVertexList(), worldCoords, localToWorld, _nodePath, arrayStateStack.back()->arrays, instanceIndex, _poly.getVertexIndex());
		_intersections.emplace_back(intersection);

		return _intersections;
	}
	PolytopeIntersector::Intersections PolytopeIntersector::add(const std::vector<dvec3> &coord, uint32_t instanceIndex, std::vector<uint32_t> &index)
	{
		auto localToWorld = computeTransform(_nodePath);
		std::vector<dvec3>worldCoords;
		for (size_t i = 0; i < coord.size(); i++)
		{
			worldCoords.emplace_back(localToWorld * coord.at(i));
		}
		ref_ptr<Intersection> intersection = Intersection::create(coord, worldCoords, localToWorld, _nodePath, arrayStateStack.back()->arrays, instanceIndex, index);
		_intersections.emplace_back(intersection);

		return _intersections;
	}

	void PolytopeIntersector::pushTransform(const Transform &mtransform)
	{
		auto &l2wStack = localToWorldStack();
		auto &w2lStack = worldToLocalStack();
		dmat4 localToWorld = mtransform.transform(dmat4{});

		dmat4 worldToLocal = inverse(localToWorld);
		l2wStack.push_back(localToWorld);
		w2lStack.push_back(worldToLocal);

		// save two polytopes in the vector  maybe have the better method
		// world plane to loacal plane 
		//  vertex is local coord
		Polytope wordplaneListSegment;
		for (int i = 0; i < _planeSegmentStack.at(_planeSegmentStack.size()-1).getPlaneList().size(); ++i)
		{
			auto planeList = _planeSegmentStack.at(_planeSegmentStack.size() - 1).getPlaneList();
			planeList.at(i) = planeList.at(i) * vsg::inverse(worldToLocal);
			double _length = 1.0 / sqrt(planeList.at(i).vec.x * planeList.at(i).vec.x
										+ planeList.at(i).vec.y * planeList.at(i).vec.y +
										planeList.at(i).vec.z * planeList.at(i).vec.z);

			planeList.at(i).vec.x *= _length;
			planeList.at(i).vec.y *= _length;
			planeList.at(i).vec.z *= _length;
			planeList.at(i).p *= _length;
			wordplaneListSegment.add(planeList.at(i));

		}
		_planeSegmentStack.emplace_back(std::move(wordplaneListSegment));
	}

	void PolytopeIntersector::popTransform()
	{
		_planeSegmentStack.pop_back();
		localToWorldStack().pop_back();
		worldToLocalStack().pop_back();
	}

	bool PolytopeIntersector::intersects(const dsphere &bs)
	{
		if (!bs.valid()) return false;
		auto &PlaneSegment = _planeSegmentStack.at(_planeSegmentStack.size()-1);
		bool inter = PlaneSegment.contains(bs);
		return inter;
	}

	bool PolytopeIntersector::intersectDraw(uint32_t firstVertex, uint32_t vertexCount, uint32_t firstInstance, uint32_t instanceCount)
	{
		auto &arrayState = *arrayStateStack.back();
		auto &polytope = _planeSegmentStack.back();
		if (arrayState.topology == VK_PRIMITIVE_TOPOLOGY_POINT_LIST && vertexCount >= 1)
		{
			uint32_t lastIndex = instanceCount > 1 ? (firstInstance + instanceCount) : firstInstance + 1;
			for (uint32_t instanceIndex = firstInstance; instanceIndex < lastIndex; ++instanceIndex)
			{
				bool isIntersect = polytope.contains(arrayState.vertexArray(instanceIndex), isCatchOnePoint);
				if (isIntersect)
				{
					add(polytope, instanceIndex);
					if (isCatchOnePoint)
					{
						return true;
					}
				}
			}
		}
		
		else if (arrayState.topology == VK_PRIMITIVE_TOPOLOGY_LINE_LIST ||
				 arrayState.topology == VK_PRIMITIVE_TOPOLOGY_LINE_STRIP ||
				 arrayState.topology == VK_PRIMITIVE_TOPOLOGY_LINE_LIST_WITH_ADJACENCY ||
				 arrayState.topology == VK_PRIMITIVE_TOPOLOGY_LINE_STRIP_WITH_ADJACENCY
				 && vertexCount >= 2)
		{
			uint32_t lastIndex = instanceCount > 1 ? (firstInstance + instanceCount) : firstInstance + 1;
			for (uint32_t instanceIndex = firstInstance; instanceIndex < lastIndex; ++instanceIndex)
			{
				bool isIntersect = polytope.contains(arrayState.vertexArray(instanceIndex), false);
				if (isIntersect)
				{
					add(polytope, instanceIndex);
				}
			}
		}
		else
		{
			return false;
		}
		return !polytope.getVertexList().empty();
	}

	bool PolytopeIntersector::intersectDrawIndexed(uint32_t firstIndex, uint32_t indexCount, uint32_t firstInstance, uint32_t instanceCount)
	{
		auto &arrayState = *arrayStateStack.back();
		auto &polytope = _planeSegmentStack.back();
		if (arrayState.topology == VK_PRIMITIVE_TOPOLOGY_POINT_LIST && indexCount >= 1)
		{
			uint32_t lastIndex = instanceCount > 1 ? (firstInstance + instanceCount) : firstInstance + 1;
			for (uint32_t instanceIndex = firstInstance; instanceIndex < lastIndex; ++instanceIndex)
			{
				bool isIntersect = polytope.contains(arrayState.vertexArray(instanceIndex), true);
				if (isIntersect)
				{
					add(polytope, instanceIndex);
				}
			}
		}
		else if (arrayState.topology == VK_PRIMITIVE_TOPOLOGY_LINE_LIST ||
				 arrayState.topology == VK_PRIMITIVE_TOPOLOGY_LINE_STRIP ||
				 arrayState.topology == VK_PRIMITIVE_TOPOLOGY_LINE_LIST_WITH_ADJACENCY ||
				 arrayState.topology == VK_PRIMITIVE_TOPOLOGY_LINE_STRIP_WITH_ADJACENCY
				 && indexCount >= 2)
		{
			uint32_t lastIndex = instanceCount > 1 ? (firstInstance + instanceCount) : firstInstance + 1;
			for (uint32_t instanceIndex = firstInstance; instanceIndex < lastIndex; ++instanceIndex)
			{
				bool isIntersect = polytope.contains(arrayState.vertexArray(instanceIndex), false);
				if (isIntersect)
				{
					add(polytope, instanceIndex);
				}
			}
		}
		else
		{
			return false;
		}
		return !polytope.getVertexList().empty();
	}

	PolytopeIntersector::Intersection::Intersection(const std::vector<dvec3> &in_localIntersection, const std::vector<dvec3> &in_worldIntersection, const dmat4 &in_localToWorld, const NodePath &in_nodePath, const DataList &in_arrays, uint32_t in_instanceIndex, std::vector<uint32_t> index) :
		localIntersection(in_localIntersection),
		worldIntersection(in_worldIntersection),

		localToWorld(in_localToWorld),
		nodePath(in_nodePath),
		arrays(in_arrays),

		m_instanceIndex(in_instanceIndex),
		m_pointIndices(index)
	{
	}

}
