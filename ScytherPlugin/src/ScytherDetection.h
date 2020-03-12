
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
    };


    /// Mapper for SofaInterface::ScytherPointCollisionModel<sofa::defaulttype::Vec3Types>
    template<class DataTypes>
    class ContactMapper<SofaInterface::ScytherTriangleCollisionModel<sofa::defaulttype::Vec3Types>, DataTypes> : public BarycentricContactMapper<SofaInterface::ScytherTriangleCollisionModel<sofa::defaulttype::Vec3Types>, DataTypes>
    {
    public:
        typedef typename DataTypes::Real Real;
        typedef typename DataTypes::Coord Coord;
        int addPoint(const Coord& P, int index, Real&)
        {
            int nbt = this->model->getCollisionTopology()->getNbTriangles();
            if (index < nbt)
                return this->mapper->createPointInTriangle(P, index, &this->model->getMechanicalState()->read(core::ConstVecCoordId::position())->getValue());
            else
            {
                int qindex = (index - nbt) / 2;
                int nbq = this->model->getCollisionTopology()->getNbQuads();
                if (qindex < nbq)
                    return this->mapper->createPointInQuad(P, qindex, &this->model->getMechanicalState()->read(core::ConstVecCoordId::position())->getValue());
                else
                {
                    msg_error("ContactMapper<ScytherTriangleModel>") << "Invalid contact element index " << index << " on a topology with " << nbt << " triangles and " << nbq << " quads." << msgendl
                        << "model=" << this->model->getName() << " size=" << this->model->getSize();
                    return -1;
                }
            }
        }
        int addPointB(const Coord& P, int index, Real& /*r*/, const defaulttype::Vector3& baryP)
        {
    
            int nbt = this->model->getCollisionTopology()->getNbTriangles();
            if (index < nbt)
                return this->mapper->addPointInTriangle(index, baryP.ptr());
            else
            {
                // TODO: barycentric coordinates usage for quads
                int qindex = (index - nbt) / 2;
                int nbq = this->model->getCollisionTopology()->getNbQuads();
                if (qindex < nbq)
                    return this->mapper->createPointInQuad(P, qindex, &this->model->getMechanicalState()->read(core::ConstVecCoordId::position())->getValue());
                else
                {
                    msg_error("ContactMapper<ScytherTriangleModel>") << "Invalid contact element index " << index << " on a topology with " << nbt << " triangles and " << nbq << " quads." << msgendl
                        << "model=" << this->model->getName() << " size=" << this->model->getSize();
                    return -1;
                }
            }
        }
    
        inline int addPointB(const Coord& P, int index, Real& r) { return addPoint(P, index, r); }
    };


#if !defined(SCYTHER_DETECTION_CPP) 
    extern template class SOFA_SCYTHER_API ContactMapper<SofaInterface::ScytherPointCollisionModel<sofa::defaulttype::Vec3Types>, sofa::defaulttype::Vec3Types>;
    extern template class SOFA_SCYTHER_API ContactMapper<SofaInterface::ScytherTriangleCollisionModel<sofa::defaulttype::Vec3Types>, sofa::defaulttype::Vec3Types>;
#endif

 }
 }
 }

//

#endif
