#include "PolytopeIntersector.h"

namespace vsg {
    PolytopeIntersector::PolytopeIntersector( vsg::Polytope& polytope,Camera*camera):_camera(camera)
    {
        
        _polytope.set (polytope.getPlaneList());
        auto viewport = _camera->getViewport();
        auto projectionMatrix = _camera->projectionMatrix->transform();
        auto viewMatrix = _camera->viewMatrix->transform();
        auto pv =   projectionMatrix * viewMatrix;
        _polytope.setAndTransformProvidingInverse(pv);
       
        _PlaneSegmentStack.push_back(_polytope);

    }
    //input srceen coord from -1 to 1
    PolytopeIntersector::PolytopeIntersector(Camera *camera, float xMin, float yMin, float xMax, float yMax):_camera(camera)
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
        _PlaneSegmentStack.emplace_back(_polytope); 
    
    }
    PolytopeIntersector::Intersections PolytopeIntersector::add( vsg::Polytope& _poly, uint32_t instanceIndex)
    {
        ref_ptr<Intersection> intersection;

        auto localToWorld = computeTransform(_nodePath);
        std::vector<dvec3>worldCoords;
        for (size_t i = 0; i < _poly.getVertexList().size(); i++)
        {
            worldCoords.emplace_back(localToWorld * _poly.getVertexList().at(i));
        }
        intersection = Intersection::create(_poly.getVertexList(), worldCoords, localToWorld, _nodePath, arrayStateStack.back()->arrays, instanceIndex, _poly.getVertexIndex());
        intersections.emplace_back(intersection);

        return intersections;
    }
    PolytopeIntersector::Intersections PolytopeIntersector::add(const std::vector<dvec3>& coord, uint32_t instanceIndex, std::vector<uint32_t>& index)
    {
        ref_ptr<Intersection> intersection;

        auto localToWorld = computeTransform(_nodePath);
        std::vector<dvec3>worldCoords;
        for (size_t i = 0; i < coord.size(); i++)
        {
            worldCoords.emplace_back(localToWorld * coord.at(i));
        }
        intersection = Intersection::create(coord, worldCoords, localToWorld, _nodePath, arrayStateStack.back()->arrays, instanceIndex, index);
        intersections.emplace_back(intersection);

        return intersections;
        
    }
  
    void PolytopeIntersector::pushTransform(const Transform& mtransform)
    {
        auto& l2wStack = localToWorldStack();
        auto& w2lStack = worldToLocalStack();
        dmat4 localToWorld = mtransform.transform(dmat4{});
        
        dmat4 worldToLocal = inverse(localToWorld);

        l2wStack.push_back(localToWorld);
        w2lStack.push_back(worldToLocal);

        // save two polytopes in the vector  maybe have the better method
        // world plane to loacal plane 
        //  vertex is local coord
        Polytope wordplaneListSegment;
        for (int i = 0; i < _PlaneSegmentStack.at(0).getPlaneList().size(); ++i)
        {
            auto _planeList =_PlaneSegmentStack.at(0).getPlaneList();
            _planeList.at(i) =   _planeList.at(i)*vsg::inverse(worldToLocal);
            double _length = 1.0 / sqrt(_planeList.at(i).vec.x * _planeList.at(i).vec.x
                + _planeList.at(i).vec.y * _planeList.at(i).vec.y +
                _planeList.at(i).vec.z * _planeList.at(i).vec.z);

            _planeList.at(i).vec.x *= _length;
            _planeList.at(i).vec.y *= _length;
            _planeList.at(i).vec.z *= _length;
            _planeList.at(i).p *= _length;
            wordplaneListSegment.add(_planeList.at(i));
           
        }
        _PlaneSegmentStack.emplace_back(wordplaneListSegment);
    }

    void PolytopeIntersector::popTransform()
    {
        
        _PlaneSegmentStack.pop_back();
        localToWorldStack().pop_back();
        worldToLocalStack().pop_back();
    }
	bool PolytopeIntersector::intersects(const dsphere& bs)
	{
        if (!bs.valid()) return false;
        auto& PlaneSegment = _PlaneSegmentStack.back();
        bool inter= PlaneSegment.contains(bs);
        return inter;
	}

	bool PolytopeIntersector::intersectDraw(uint32_t firstVertex, uint32_t vertexCount, uint32_t firstInstance, uint32_t instanceCount)
	{
        auto& arrayState = *arrayStateStack.back();
        auto& polytope = _PlaneSegmentStack.back();
        if (arrayState.topology == VK_PRIMITIVE_TOPOLOGY_POINT_LIST && vertexCount >= 1)
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
             /*是否可以合并于上面*/
        else if(arrayState.topology == VK_PRIMITIVE_TOPOLOGY_LINE_LIST|| 
            arrayState.topology == VK_PRIMITIVE_TOPOLOGY_LINE_STRIP|| 
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
        auto& arrayState = *arrayStateStack.back();
        auto& polytope = _PlaneSegmentStack.back();
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


    PolytopeIntersector::Intersection::Intersection(const std::vector<dvec3>& in_localIntersection, const std::vector<dvec3>& in_worldIntersection, const dmat4& in_localToWorld, const NodePath& in_nodePath, const DataList& in_arrays, uint32_t in_instanceIndex, std::vector<uint32_t> index)
        :localIntersection(in_localIntersection),
        worldIntersection(in_worldIntersection),

        localToWorld(in_localToWorld),
        nodePath(in_nodePath),
        arrays(in_arrays),

        instanceIndex(in_instanceIndex),
        pointIndex(index) 
    {
    }

}