


#include "IMeshGeneratorDetails.h"
#include "IMeshGenerator.h"


namespace IEngine
{

    namespace MeshGenerator
    {


            static void BuildFace(  IMeshModel& mesh,
                                    const IQuaternion& rotation,
                                    float sizeHorz,
                                    float sizeVert,
                                    float sizeOffsetZ,
                                    std::uint32_t segsHorz,
                                    std::uint32_t segsVert,
                                    bool alternateGrid)
            {
                sizeOffsetZ /= 2;

                const auto invHorz = float(1) / static_cast<float>(segsHorz);
                const auto invVert = float(1) / static_cast<float>(segsVert);

                auto idxOffset = mesh.VertexCount();


                auto AddQuad = [&](std::uint32_t u, std::uint32_t v, VertexIndex i0, VertexIndex i1, VertexIndex i2, VertexIndex i3)
                {
                    AddTriangulatedQuad(mesh, alternateGrid, u, v, i0, i1, i2, i3, idxOffset);
                };

                /* Generate vertices */
                for (std::uint32_t i = 0; i <= segsVert; ++i)
                {
                    for (std::uint32_t j = 0; j <= segsHorz; ++j)
                    {
                        auto u = invHorz * static_cast<float>(j);
                        auto v = invVert * static_cast<float>(i);

                        auto x = sizeHorz * u - sizeHorz/float(2);
                        auto y = sizeVert * v - sizeVert/float(2);

                        VertexInfo InfoVrtx(rotation.GetRotMatrix() * IVector3(x, y, sizeOffsetZ),
                                            rotation.GetRotMatrix() * IVector3(0, 0, -1),
                                            rotation.GetRotMatrix() * IVector3(1, 0, 0),
                                            IVector2(u, float(1) - v));

                        mesh.AppendVertex(InfoVrtx);
                    }
                }

                /* Generate indices */
                const auto strideHorz = segsHorz + 1;

                for (std::uint32_t v = 0; v < segsVert; ++v)
                {
                    for (std::uint32_t u = 0; u < segsHorz; ++u)
                    {
                        AddQuad(
                                    u, v,
                                    (  v   *strideHorz + u   ),
                                    ( (v+1)*strideHorz + u   ),
                                    ( (v+1)*strideHorz + u+1 ),
                                    (  v   *strideHorz + u+1 )
                                    );
                    }
                }

            }



            void GenerateMesh(const CuboidDescriptor& desc, IMeshModel& mesh)
            {
                auto segsX = std::max(1, desc.segments.x);
                auto segsY = std::max(1, desc.segments.y);
                auto segsZ = std::max(1, desc.segments.z);

                /* Generate faces */
                // front
                BuildFace(
                    mesh, IQuaternion::IDENTITY,
                    desc.size.x, desc.size.y, -desc.size.z, segsX, segsY, desc.alternateGrid
                );

                // back
                BuildFace(
                    mesh, IQuaternion::FromEulerAngles(0, pi, 0),
                    desc.size.x, desc.size.y, -desc.size.z, segsX, segsY, desc.alternateGrid
                );

                // left
                BuildFace(
                    mesh, IQuaternion::FromEulerAngles(0, -pi_0_5, 0),
                    desc.size.z, desc.size.y, -desc.size.x, segsZ, segsY, desc.alternateGrid
                );

                // right
                BuildFace(
                    mesh, IQuaternion::FromEulerAngles(0, pi_0_5, 0),
                    desc.size.z, desc.size.y, -desc.size.x, segsZ, segsY, desc.alternateGrid
                );

                // top
                BuildFace(
                    mesh, IQuaternion::FromEulerAngles(pi_0_5, 0, 0),
                    desc.size.x, desc.size.z, -desc.size.y, segsX, segsZ, desc.alternateGrid
                );

                // bottom
                BuildFace(
                    mesh, IQuaternion::FromEulerAngles(-pi_0_5, 0, 0),
                    desc.size.x, desc.size.z, -desc.size.y, segsX, segsZ, desc.alternateGrid
                );
            }

            IMeshModel GenerateMeshT(const CuboidDescriptor& desc)
            {
                IMeshModel mesh(1,0,0,0);
                GenerateMesh(desc, mesh);
                return mesh;
            }




    } // /namespace MeshGenerator

} // /namespace Gm
