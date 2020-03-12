
#define SCYTHER_COLLISION_TRIANGLE_MODEL_CPP
#include "ScytherTriangleModel.inl"
#include <sofa/core/ObjectFactory.h>


namespace SofaInterface {

int ScytherTriangleCollisionModelClass =
    sofa::core::RegisterObject("collision model using a triangular mesh, as described in BaseMeshTopology, for Scyther")
        .add<ScytherTriangleCollisionModel<sofa::defaulttype::Vec3Types>>()
        .addAlias("ScytherTriangleModel");
;

template class SOFA_SCYTHER_API ScytherTriangleCollisionModel<sofa::defaulttype::Vec3Types>;

} // namespace SofaInterface
