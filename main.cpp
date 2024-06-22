#pragma once 
#include <iostream>
#include <vsg/all.h>
#include "PolytopeIntersector.h"

using namespace vsg;
class IntersectionHandler : public vsg::Inherit<vsg::Visitor, IntersectionHandler>
{
public:
    vsg::GeometryInfo geom;
    vsg::StateInfo state;

    vsg::ref_ptr<vsg::Builder> builder;
    vsg::ref_ptr<vsg::Options> options;
    vsg::ref_ptr<vsg::Camera> camera;
    vsg::ref_ptr<vsg::Group> scenegraph;
    vsg::ref_ptr<vsg::EllipsoidModel> ellipsoidModel;
    double scale = 1.0;
    bool verbose = true;

    IntersectionHandler(vsg::ref_ptr<vsg::Builder> in_builder, vsg::ref_ptr<vsg::Camera> in_camera, vsg::ref_ptr<vsg::Group> in_scenegraph, vsg::ref_ptr<vsg::EllipsoidModel> in_ellipsoidModel, double in_scale, vsg::ref_ptr<vsg::Options> in_options) :
        builder(in_builder),
        options(in_options),
        camera(in_camera),
        scenegraph(in_scenegraph),
        ellipsoidModel(in_ellipsoidModel),
        scale(in_scale)
    {
        builder->verbose = verbose;
        if (scale > 10.0) scale = 10.0;
    }

    void apply(vsg::KeyPressEvent& keyPress) override
    {
        if (lastPointerEvent)
        {
            intersection(*lastPointerEvent);
            

            vsg::info("keyPress.keyModifier = ", keyPress.keyModifier, " keyPress.keyBase = ", keyPress.keyBase);

            geom.dx.set(scale, 0.0f, 0.0f);
            geom.dy.set(0.0f, scale, 0.0f);
            geom.dz.set(0.0f, 0.0f, scale);

            if (keyPress.keyModifier & vsg::MODKEY_Control)
            {
                // when we press the ctrl key we want to enable billboarding of the created shapes
                state.billboard = true;

                // when billboarding the position is the pivot point in local coordinates
                geom.position.set(0.0f, 0.0f, 0.0f);

                // the position is set by positions data, in this case just one position so use a vec4Value, but if needed we can use an array of positions
                auto pos = woldPosition;
                geom.positions = vsg::vec4Value::create(vsg::vec4(pos.x, pos.y, pos.z, scale * 5.0)); // x,y,z and scaleDistance
            }
            else
            {
                geom.position = woldPosition;
            }

            if (keyPress.keyBase == 'b')
            {
                scenegraph->addChild(builder->createBox(geom, state));
            }
            else if (keyPress.keyBase == 'q')
            {
                scenegraph->addChild(builder->createQuad(geom, state));
            }
            else if (keyPress.keyBase == 'c')
            {
                scenegraph->addChild(builder->createCylinder(geom, state));
            }
            else if (keyPress.keyBase == 'p')
            {
                scenegraph->addChild(builder->createCapsule(geom, state));
            }
            else if (keyPress.keyBase == 's')
            {
                scenegraph->addChild(builder->createSphere(geom, state));
            }
            else if (keyPress.keyBase == 'n')
            {
                scenegraph->addChild(builder->createCone(geom, state));
            }
            else if (keyPress.keyBase == 'o')
            {
                vsg::write(scenegraph, "builder.vsgt");
            }
        }

        if (state.billboard)
        {
            // switch off billboarding so other shapes aren't affected.
            state.billboard = false;
            geom.positions = {};
        }
    }

    void apply(vsg::ButtonPressEvent& buttonPressEvent) override
    {
        lastPointerEvent = &buttonPressEvent;

        if (buttonPressEvent.button == 1)
        {
                pointxy=vsg::vec2(buttonPressEvent.x, buttonPressEvent.y);
               
                intersection(buttonPressEvent);
        }
    }

    void apply(vsg::PointerEvent& pointerEvent) override
    {
        lastPointerEvent = &pointerEvent;
    }

    void intersection(vsg::PointerEvent& pointerEvent)
    {
       //project Model
        auto width=camera->getViewport().width;
        auto height = camera->getViewport().height;
        auto x = camera->getViewport().x;
        auto y = camera->getViewport().y;
        auto xNormal = 2.0f * (pointxy.x - x) / ( width) - 1.0f;
        auto yNormal = 2.0f * (pointxy.y - y) / (height) - 1.0f ;
        auto intersector = vsg::PolytopeIntersector::create(camera, xNormal -0.05, yNormal -0.05, xNormal +0.05, yNormal +0.05);
        
        scenegraph->accept(*intersector);
      
        auto intersize=intersector->intersections.size();
        for (int i = 0; i < intersize; i++)
        {
            for (int j = 0; j < intersector->intersections.at(i)->localIntersection.size(); j++)
            {
                std::cout << "vertex: (" << intersector->intersections.at(i)->localIntersection.at(j).x << "," << intersector->intersections.at(i)->localIntersection.at(j).y<< "," << intersector->intersections.at(i)->localIntersection.at(j).z<< ")" << std::endl;
                std::cout << "world vertex: (" << intersector->intersections.at(i)->worldIntersection.at(j).x<<"."<< intersector->intersections.at(i)->worldIntersection.at(j).y << "," << intersector->intersections.at(i)->worldIntersection.at(j).z << ")" << std::endl;
                woldPosition = vsg::vec3(intersector->intersections.at(i)->worldIntersection.at(j));
                std::cout << "local to world :" << intersector->intersections.at(i)->localToWorld; 
                std::cout << "node path :" << std::endl;
                for (int s=0;s< intersector->intersections.at(i)->nodePath.size();++s )
                {
                        
                        std::cout<< intersector->intersections.at(i)->nodePath.at(s)->className() << std::endl;
                }
                
                for (auto& array : intersector->intersections.at(i)->arrays)
                {
                    std::cout << array << " " << std::endl;
                }
            }
        }
        if (intersector->intersections.empty())
        {
            std::cout << "no vertex" << std::endl;
        }
    }
    
protected:
    vsg::ref_ptr<vsg::PointerEvent> lastPointerEvent;
   //vsg::ref_ptr<vsg::PolytopeIntersector::Intersection> lastIntersections;
   
    vsg::vec3 woldPosition;
    vsg::vec2 pointxy;
    int times=0;
};


vsg::ref_ptr<vsg::Camera> createCameraForScene(vsg::Node* scenegraph, int32_t x, int32_t y, uint32_t width, uint32_t height)
{
    // compute the bounds of the scene graph to help position the camera
    vsg::ComputeBounds computeBounds;
    scenegraph->accept(computeBounds);
    vsg::dvec3 centre = (computeBounds.bounds.min + computeBounds.bounds.max) * 0.5;
    double radius = vsg::length(computeBounds.bounds.max - computeBounds.bounds.min) * 0.6;
    double nearFarRatio = 0.001;

    // set up the camera
    auto lookAt = vsg::LookAt::create(centre + vsg::dvec3(0.0, -radius * 3.5, 0.0),
        centre, vsg::dvec3(0.0, 0.0, 1.0));

    auto perspective = vsg::Perspective::create(30.0, static_cast<double>(width) / static_cast<double>(height),
        nearFarRatio * radius, radius * 4.5);

    auto viewportstate = vsg::ViewportState::create(x, y, width, height);

    return vsg::Camera::create(perspective, lookAt, viewportstate);
}
vsg::ref_ptr<vsg::Commands> createhight()
{
    auto command = vsg::Commands::create();
    vsg::ref_ptr<vsg::ShaderStage> vertexShader = vsg::ShaderStage::read(VK_SHADER_STAGE_VERTEX_BIT, "main", "vert.spv");
    vsg::ref_ptr<vsg::ShaderStage> fragmentShader = vsg::ShaderStage::read(VK_SHADER_STAGE_FRAGMENT_BIT, "main", "frag.spv");

    vsg::DescriptorSetLayoutBindings descriptorBindings{
       {0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr} // { binding, descriptorType, descriptorCount, stageFlags, pImmutableSamplers}
    };

    vsg::PushConstantRanges pushConstantRanges{
        {VK_SHADER_STAGE_VERTEX_BIT, 0, 128} // projection, view, and model matrices, actual push constant calls automatically provided by the VSG's RecordTraversal
    };

    vsg::VertexInputState::Bindings vertexBindingsDescriptions{
        VkVertexInputBindingDescription{0, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX}, // vertex data
        VkVertexInputBindingDescription{1, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX} // colour data
    };


    return command;
}
int main(int argc, char** argv)
{
    // set up defaults and read command line arguments to override them
    vsg::CommandLine arguments(&argc, argv);

    auto windowTraits = vsg::WindowTraits::create();
    windowTraits->debugLayer = true;
    //windowTraits->apiDumpLayer = true;
    windowTraits->debugUtils = true;
    windowTraits->width = 900;
        windowTraits->height =600;

    if (arguments.errors()) return arguments.writeErrorMessages(std::cerr);

    // set up search paths to SPIRV shaders and textures
    vsg::Paths searchPaths = vsg::getEnvPaths("VSG_FILE_PATH");
    auto options = vsg::Options::create();
    auto builder = vsg::Builder::create();
    builder->options = options;
    // load shaders
    vsg::ref_ptr<vsg::ShaderStage> vertexShader = vsg::ShaderStage::read(VK_SHADER_STAGE_VERTEX_BIT, "main", "vert.spv");
    vsg::ref_ptr<vsg::ShaderStage> fragmentShader = vsg::ShaderStage::read(VK_SHADER_STAGE_FRAGMENT_BIT, "main", "frag.spv");
    if (!vertexShader || !fragmentShader)
    {
        std::cout << "Could not create shaders." << std::endl;
        return 1;
    }
    // set up graphics pipeline
    vsg::DescriptorSetLayoutBindings descriptorBindings{
        {0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr} // { binding, descriptorType, descriptorCount, stageFlags, pImmutableSamplers}
    };

    auto descriptorSetLayout = vsg::DescriptorSetLayout::create(descriptorBindings);

    vsg::PushConstantRanges pushConstantRanges{
        {VK_SHADER_STAGE_VERTEX_BIT, 0, 128} // projection, view, and model matrices, actual push constant calls automatically provided by the VSG's RecordTraversal
    };

    vsg::VertexInputState::Bindings vertexBindingsDescriptions{
        VkVertexInputBindingDescription{0, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX}, // vertex data
        VkVertexInputBindingDescription{1, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX} // colour data
    };
    //VK_VERTEX_INPUT_RATE_VERTEX 每一个绑定
    //
    vsg::VertexInputState::Attributes vertexAttributeDescriptions{
        VkVertexInputAttributeDescription{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0}, // vertex data
        VkVertexInputAttributeDescription{1, 1, VK_FORMAT_R32G32B32_SFLOAT, 0}, // colour data
                                          
    };
    //VK_PRIMITIVE_TOPOLOGY_LINE_STRIP
    vsg::GraphicsPipelineStates pipelineStates{
        vsg::VertexInputState::create(vertexBindingsDescriptions, vertexAttributeDescriptions),
        vsg::InputAssemblyState::create(VK_PRIMITIVE_TOPOLOGY_POINT_LIST),
        vsg::RasterizationState::create(),
        vsg::MultisampleState::create(),
        vsg::ColorBlendState::create(),
        vsg::DepthStencilState::create() };

    auto pipelineLayout = vsg::PipelineLayout::create(vsg::DescriptorSetLayouts{ descriptorSetLayout }, pushConstantRanges);
    auto graphicsPipeline = vsg::GraphicsPipeline::create(pipelineLayout, vsg::ShaderStages{ vertexShader, fragmentShader }, pipelineStates);
    auto bindGraphicsPipeline = vsg::BindGraphicsPipeline::create(graphicsPipeline);

    // create texture image and associated DescriptorSets and binding
    //auto texture = vsg::DescriptorImage::create(vsg::Sampler::create(), textureData, 0, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);

    auto descriptorSet = vsg::DescriptorSet::create(descriptorSetLayout, vsg::Descriptors{});
    auto bindDescriptorSet = vsg::BindDescriptorSet::create(VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, descriptorSet);

    // create StateGroup as the root of the scene/command graph to hold the GraphicsPipeline, and binding of Descriptors to decorate the whole graph
    auto scenegraph = vsg::StateGroup::create();
    //scenegraph->className()=
    scenegraph->add(bindGraphicsPipeline);
    scenegraph->add(bindDescriptorSet);

    // set up model transformation node
    auto transform = vsg::MatrixTransform::create(); // VK_SHADER_STAGE_VERTEX_BIT

    // add transform to root of the scene graph
    scenegraph->addChild(transform);
 
    auto vertices = vsg::vec3Array::create(
        { {-0.5f, -0.5f, 0.0f},
         {0.5f, -0.5f, 0.0f},
         {0.5f, 0.5f, 0.0f},
         {-0.5f, 0.5f, 0.0f},
         });
    auto colors = vsg::vec3Array::create(
        {
            {1.0f, 0.0f, 0.0f},
            {0.0f, 1.0f, 0.0f},
            {0.0f, 0.0f, 1.0f},
            {1.0f, 1.0f, 1.0f},
            
        }); // VK_FORMAT_R32G32B32_SFLOAT, VK_VERTEX_INPUT_RATE_VERTEX, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VK_SHARING_MODE_EXCLUSIVE
    //VK_FORMAT_R32G32B32_SFLOAT, VK_VERTEX_INPUT_RATE_VERTEX, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VK_SHARING_MODE_EXCLUSIVE
 
    
    vsg::ref_ptr<vsg::EllipsoidModel> ellipsoidModel;

    // setup geometry
    auto drawCommands = vsg::Commands::create();
    drawCommands->addChild(vsg::BindVertexBuffers::create(0, vsg::DataList{ vertices, colors }));
    //drawCommands->addChild(vsg::BindIndexBuffer::create(indices));
    drawCommands->addChild(vsg::Draw::create(4, 1, 0, 0));

    ellipsoidModel = drawCommands->getRefObject<vsg::EllipsoidModel>("EllipsoidModel");
    //perspective = vsg::Perspective::create(30.0, static_cast<double>(window->extent2D().width) / static_cast<double>(window->extent2D().height), nearFarRatio * radius, radius * 4.5);
    vsg::Allocator::instance()->setMemoryTracking(vsg::MemoryTracking::MEMORY_TRACKING_DEFAULT);
    vsg::Logger::instance()->level = vsg::Logger::Level::LOGGER_ERROR;
    // add drawCommands to transform

    transform->addChild(drawCommands);
    transform->matrix = transform->transform(vsg::translate(30.0, 2.0, 3.0));
   
   
    auto viewer = vsg::Viewer::create();

    auto window = vsg::Window::create(windowTraits);
    if (!window)
    {
        std::cout << "Could not create window." << std::endl;
        return 1;
    }

    viewer->addWindow(window);



    auto camera = createCameraForScene(transform, 0, 0, window->extent2D().width, window->extent2D().height);
    auto commandGraph = vsg::createCommandGraphForView(window, camera, scenegraph);
    viewer->assignRecordAndSubmitTaskAndPresentation({ commandGraph });


    // assign a CloseHandler to the Viewer to respond to pressing Escape or the window close button
    viewer->addEventHandlers({ vsg::CloseHandler::create(viewer) });
    viewer->addEventHandler(vsg::Trackball::create(camera, ellipsoidModel));
    auto bounds = vsg::visit<vsg::ComputeBounds>(transform).bounds;

    double radius = vsg::length(bounds.max - bounds.min) * 0.5;
    /*====================================================================*/
    auto intersectionHandler = IntersectionHandler::create(builder, camera, scenegraph, ellipsoidModel, radius * 0.1, options);
    //intersectionHandler->state = stateInfo;
    viewer->addEventHandler(intersectionHandler);
    builder->assignCompileTraversal(vsg::CompileTraversal::create(*viewer));
    // main frame loop


    // compile the Vulkan objects
    viewer->compile();
    while (viewer->advanceToNextFrame())
    {
        // pass any events into EventHandlers assigned to the Viewer
        viewer->handleEvents();
        viewer->update();

        viewer->recordAndSubmit();

        viewer->present();
    }
    
    // clean up done automatically thanks to ref_ptr<>
    return 0;
}
