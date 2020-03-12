
#ifndef SCYTHER_DETECTION_H
#define SCYTHER_DETECTION_H
#include <scyther.h>

//#include <SofaBaseCollision/CubeModel.h>
#include <sofa/core/CollisionElement.h>
#include <sofa/core/collision/BroadPhaseDetection.h>
#include <sofa/core/collision/NarrowPhaseDetection.h>
#include <sofa/defaulttype/Vec.h>

#include <SofaMeshCollision/BarycentricPenalityContact.h>
#include <SofaMeshCollision/BarycentricContactMapper.h>
#include <SofaBaseCollision/BaseContactMapper.h>


#include <ScytherCubeModel.h>
#include <ScytherPointModel.h>
#include <ScytherTriangleModel.h>


#include <sofa/helper/Factory.h>
#include <SofaBaseMechanics/BarycentricMapping.h>
#include <SofaBaseMechanics/IdentityMapping.h>
#include <SofaRigid/RigidMapping.h>
#include <SofaBaseMechanics/SubsetMapping.h>
#include <SofaBaseMechanics/MechanicalObject.h>
#include <sofa/simulation/Node.h>
#include <sofa/simulation/Simulation.h>
#include <SofaBaseCollision/BaseContactMapper.h>
#include <SofaBaseCollision/SphereModel.h>
#include <SofaMeshCollision/TriangleModel.h>
#include <SofaMeshCollision/LineModel.h>
#include <SofaMeshCollision/PointModel.h>
#include <SofaBaseMechanics/IdentityMapping.h>


namespace SofaInterface {

class SOFA_SCYTHER_API ScytherDetection : public sofa::core::collision::BroadPhaseDetection,
                                                 public sofa::core::collision::NarrowPhaseDetection
{
    template <class T>
    using Data = sofa::core::objectmodel::Data<T>;

public:
    SOFA_CLASS2(
        ScytherDetection, sofa::core::collision::BroadPhaseDetection, sofa::core::collision::NarrowPhaseDetection);

private:
    bool                                              _is_initialized;
    sofa::helper::vector<sofa::core::CollisionModel*> collisionModels;

    Data<sofa::helper::fixed_array<sofa::defaulttype::Vector3, 2>>
        box; ///< if not empty, objects that do not intersect this bounding-box will be ignored

    ScytherCubeModel::SPtr boxModel;

protected:
    ScytherDetection();

    ~ScytherDetection() override;

    // virtual bool keepCollisionBetween(sofa::core::CollisionModel* cm1, sofa::core::CollisionModel* cm2);

public:
    void init() override;
    void reinit() override;

    void addCollisionModel(sofa::core::CollisionModel* cm) override;
    void addCollisionPair(const std::pair<sofa::core::CollisionModel*, sofa::core::CollisionModel*>& cmPair) override;

    void beginBroadPhase() override;
    void endBroadPhase() override;
    void beginNarrowPhase() override;
    void endNarrowPhase() override;

    void draw(const sofa::core::visual::VisualParams* /* vparams */) override {}

    inline bool needsDeepBoundingTree() const override { return false; }
};

} // namespace SofaInterface


 namespace sofa {
 namespace component {
 namespace collision {


     /// Mapper for SofaInterface::ScytherPointCollisionModel<sofa::defaulttype::Vec3Types>
     template<class DataTypes>
     class ContactMapper<SofaInterface::ScytherPointCollisionModel<sofa::defaulttype::Vec3Types>, DataTypes> : public IdentityContactMapper<SofaInterface::ScytherPointCollisionModel<sofa::defaulttype::Vec3Types>, DataTypes>
     {
     public:
         typedef typename DataTypes::Real Real;
         typedef typename DataTypes::Coord Coord;
         //int addPoint(const Coord& P, int index, Real&)
         //{
         //    return 1;// this->mapper->createPointInLine(P, this->model->getElemEdgeIndex(index), &this->model->getMechanicalState()->read(core::ConstVecCoordId::position())->getValue());
         //}
         //int addPointB(const Coord& /*P*/, int index, Real& /*r*/, const defaulttype::Vector3& baryP)
         //{
         //    return 1;// this->mapper->addPointInLine(this->model->getElemEdgeIndex(index), baryP.ptr());
         //}

         //inline int addPointB(const Coord& P, int index, Real& r) { return addPoint(P, index, r); }
     };

     /// Mapper for SofaInterface::ScytherTriangleCollisionModel<sofa::defaulttype::Vec3Types>
     //template<class DataTypes>
     //class ContactMapper<SofaInterface::ScytherTriangleCollisionModel<sofa::defaulttype::Vec3Types>, DataTypes> : public BarycentricContactMapper<SofaInterface::ScytherTriangleCollisionModel<sofa::defaulttype::Vec3Types>, DataTypes>
     //{
     //public:
     //    typedef typename DataTypes::Real Real;
     //    typedef typename DataTypes::Coord Coord;
     //    //int addPoint(const Coord& P, int index, Real&)
     //    //{
     //    //    return 1;// this->mapper->createPointInLine(P, this->model->getElemEdgeIndex(index), &this->model->getMechanicalState()->read(core::ConstVecCoordId::position())->getValue());
     //    //}
     //    //int addPointB(const Coord& /*P*/, int index, Real& /*r*/, const defaulttype::Vector3& baryP)
     //    //{
     //    //    return 1;// this->mapper->addPointInLine(this->model->getElemEdgeIndex(index), baryP.ptr());
     //    //}

     //    //inline int addPointB(const Coord& P, int index, Real& r) { return addPoint(P, index, r); }
     //};


//#if !defined(SCYTHER_DETECTION_CPP) 
 extern template class SOFA_MESH_COLLISION_API ContactMapper<SofaInterface::ScytherPointCollisionModel<sofa::defaulttype::Vec3Types>, sofa::defaulttype::Vec3Types>;
 //extern template class SOFA_MESH_COLLISION_API ContactMapper<SofaInterface::ScytherTriangleCollisionModel<sofa::defaulttype::Vec3Types>, sofa::defaulttype::Vec3Types>;

 //extern template class SOFA_MESH_COLLISION_API BarycentricPenalityContact<SofaInterface::ScytherPointCollisionModel<sofa::defaulttype::Vec3Types>, SofaInterface::ScytherPointCollisionModel<sofa::defaulttype::Vec3Types>>;
 //extern template class SOFA_MESH_COLLISION_API BarycentricPenalityContact<SofaInterface::ScytherTriangleCollisionModel<sofa::defaulttype::Vec3Types>, SofaInterface::ScytherPointCollisionModel<sofa::defaulttype::Vec3Types>>;
//#  ifdef _MSC_VER
// // Manual declaration of non-specialized members, to avoid warnings from MSVC.
 extern template SOFA_MESH_COLLISION_API void IdentityContactMapper<SofaInterface::ScytherPointCollisionModel<sofa::defaulttype::Vec3Types>, defaulttype::Vec3Types>::cleanup();
 extern template SOFA_MESH_COLLISION_API core::behavior::MechanicalState<defaulttype::Vec3Types>* IdentityContactMapper<SofaInterface::ScytherPointCollisionModel<sofa::defaulttype::Vec3Types>, defaulttype::Vec3Types>::createMapping(const char*);
 //extern template SOFA_MESH_COLLISION_API void BarycentricContactMapper<SofaInterface::ScytherTriangleCollisionModel<sofa::defaulttype::Vec3Types>, defaulttype::Vec3Types>::cleanup();
 //extern template SOFA_MESH_COLLISION_API core::behavior::MechanicalState<defaulttype::Vec3Types>* BarycentricContactMapper<SofaInterface::ScytherTriangleCollisionModel<sofa::defaulttype::Vec3Types>, defaulttype::Vec3Types>::createMapping(const char*);
//#  endif // _MSC_VER

 //#endif

 }
 }
 }

//

#endif
