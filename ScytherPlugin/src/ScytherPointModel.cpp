
#define SCYTHER_POINT_COLLISION_MODEL_CPP
#include "ScytherPointModel.inl"
#include <sofa/core/ObjectFactory.h>

//#include <SofaBaseCollision/RigidCapsuleModel.h>
//#include <SofaMeshCollision/BarycentricContactMapper.h>
//#include <SofaMeshCollision/RigidContactMapper.inl>

//#include <sofa/core/collision/Contact.h>



namespace SofaInterface {

int ScytherPointCollisionModelClass = sofa::core::RegisterObject("Collision model which represents a set of points")
                                          .add<ScytherPointCollisionModel<sofa::defaulttype::Vec3Types>>()
                                          .addAlias("ScytherPointModel");

template class SOFA_SCYTHER_API ScytherPointCollisionModel<sofa::defaulttype::Vec3Types>;

} // namespace SofaInterface
