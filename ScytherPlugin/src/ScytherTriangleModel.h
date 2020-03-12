
#ifndef SCYTHER_COLLISION_TRIANGLE_MODEL_H
#define SCYTHER_COLLISION_TRIANGLE_MODEL_H
//#include <SofaMeshCollision/config.h>
#include <scyther.h>

#include <SofaBaseMechanics/MechanicalObject.h>
#include <SofaBaseTopology/TopologyData.h>
#include <SofaMeshCollision/LocalMinDistanceFilter.h>
#include <map>
#include <sofa/core/CollisionModel.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/defaulttype/VecTypes.h>

#include "ScytherPointModel.h"

namespace SofaInterface {

template <class DataTypes>
class ScytherTriangleCollisionModel;

class TriangleLocalMinDistanceFilter;

template <class TDataTypes>
class TScytherTriangle : public sofa::core::TCollisionElementIterator<ScytherTriangleCollisionModel<TDataTypes>>
{
public:
    typedef TDataTypes                               DataTypes;
    typedef typename DataTypes::Coord                Coord;
    typedef typename DataTypes::Deriv                Deriv;
    typedef ScytherTriangleCollisionModel<DataTypes> ParentModel;
    typedef typename DataTypes::Real                 Real;

    TScytherTriangle(ParentModel* model, int index);
    TScytherTriangle() {}
    explicit TScytherTriangle(const sofa::core::CollisionElementIterator& i);
    TScytherTriangle(ParentModel* model, int index, sofa::helper::ReadAccessor<typename DataTypes::VecCoord>& /*x*/);

    const Coord& p1() const;
    const Coord& p2() const;
    const Coord& p3() const;

    const Coord& p(int i) const;

    int p1Index() const;
    int p2Index() const;
    int p3Index() const;

    const Coord& p1Free() const;
    const Coord& p2Free() const;
    const Coord& p3Free() const;

    const Coord& operator[](int i) const;

    const Deriv& v1() const;
    const Deriv& v2() const;
    const Deriv& v3() const;
    const Deriv& v(int i) const;

    const Deriv& n() const;
    Deriv&       n();

    /// Return true if the element stores a free position vector
    bool hasFreePosition() const;

    int flags() const;

    TScytherTriangle&       shape() { return *this; }
    const TScytherTriangle& shape() const { return *this; }

    Coord interpX(sofa::defaulttype::Vec<2, Real> bary) const
    {
        return (p1() * (1 - bary[0] - bary[1])) + (p2() * bary[0]) + (p3() * bary[1]);
    }
};

/**
 * This class will create collision elements based on a triangle and/or quad mesh.
 * It uses directly the information of the topology and the dof to compute the triangle normals, BB and BoundingTree.
 * The class \sa TScytherTriangle is used to access specific triangle of this collision Model.
 */
template <class TDataTypes>
class SOFA_SCYTHER_API ScytherTriangleCollisionModel : public sofa::core::CollisionModel
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(ScytherTriangleCollisionModel, TDataTypes), sofa::core::CollisionModel);

    typedef TDataTypes                   DataTypes;
    typedef DataTypes                    InDataTypes;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::Coord    Coord;
    typedef typename DataTypes::Deriv    Deriv;
    typedef TScytherTriangle<DataTypes>  Element;
    friend class TScytherTriangle<DataTypes>;

    template <class T>
    using Data = sofa::core::objectmodel::Data<T>;

    enum TriangleFlag
    {
        FLAG_P1     = 1 << 0, ///< Point 1  is attached to this triangle
        FLAG_P2     = 1 << 1, ///< Point 2  is attached to this triangle
        FLAG_P3     = 1 << 2, ///< Point 3  is attached to this triangle
        FLAG_E23    = 1 << 3, ///< Edge 2-3 is attached to this triangle
        FLAG_E31    = 1 << 4, ///< Edge 3-1 is attached to this triangle
        FLAG_E12    = 1 << 5, ///< Edge 1-2 is attached to this triangle
        FLAG_BE23   = 1 << 6, ///< Edge 2-3 is attached to this triangle and is a boundary
        FLAG_BE31   = 1 << 7, ///< Edge 3-1 is attached to this triangle and is a boundary
        FLAG_BE12   = 1 << 8, ///< Edge 1-2 is attached to this triangle and is a boundary
        FLAG_POINTS = FLAG_P1 | FLAG_P2 | FLAG_P3,
        FLAG_EDGES  = FLAG_E12 | FLAG_E23 | FLAG_E31,
        FLAG_BEDGES = FLAG_BE12 | FLAG_BE23 | FLAG_BE31,
    };

    enum
    {
        NBARY = 2
    };

    Data<bool> d_bothSide;       ///< to activate collision on both side of the triangle model
    Data<bool> d_computeNormals; ///< set to false to disable computation of triangles normal

        /// Link to be set to the topology container in the component graph.
    sofa::core::topology::BaseMeshTopology* getCollisionTopology() override
    {
        return m_topology;
    }
protected:
    sofa::core::behavior::MechanicalState<DataTypes>* m_mstate;   ///< Pointer to the corresponding MechanicalState
    sofa::core::topology::BaseMeshTopology*           m_topology; ///< Pointer to the corresponding Topology

    VecDeriv m_normals; ///< Vector of normal direction per triangle.

    /** Pointer to the triangle array of this collision model.
     * Will point directly to the topology triangle buffer if only triangles are present. If topology is using/mixing
     * quads and triangles, This pointer will target \sa m_internalTriangles
     * @brief m_triangles
     */
    const sofa::core::topology::BaseMeshTopology::SeqTriangles* m_triangles;

    sofa::core::topology::BaseMeshTopology::SeqTriangles
        m_internalTriangles; ///< Internal Buffer of triangles to combine quads splitted and other triangles.

    bool m_needsUpdate;      ///< parameter storing the info boundingTree has to be recomputed.
    int  m_topologyRevision; ///< internal revision number to check if topology has changed.

    ScytherPointModel* m_pointModels;

    // TriangleLocalMinDistanceFilter* m_lmdFilter;

protected:
    ScytherTriangleCollisionModel();

    virtual void updateFromTopology();
    virtual void updateFlags(int ntri = -1);
    virtual void updateNormals();

public:
    void init() override;

    // -- CollisionModel interface

    void resize(int size) override;

    void computeBoundingTree(int maxDepth = 0) override;

    void computeContinuousBoundingTree(double dt, int maxDepth = 0) override;

    void draw(const sofa::core::visual::VisualParams*, int index) override;

    void draw(const sofa::core::visual::VisualParams* vparams) override;

    bool canCollideWithElement(int index, CollisionModel* model2, int index2) override;

    sofa::core::behavior::MechanicalState<DataTypes>*       getMechanicalState() { return m_mstate; }
    const sofa::core::behavior::MechanicalState<DataTypes>* getMechanicalState() const { return m_mstate; }

    const VecCoord& getX() const
    {
        return (getMechanicalState()->read(sofa::core::ConstVecCoordId::position())->getValue());
    }
    const sofa::core::topology::BaseMeshTopology::SeqTriangles& getTriangles() const { return *m_triangles; }
    const VecDeriv&                                             getNormals() const { return m_normals; }
    int getTriangleFlags(sofa::core::topology::BaseMeshTopology::TriangleID i);

    TriangleLocalMinDistanceFilter* getFilter() const;

    void setFilter(TriangleLocalMinDistanceFilter* /*lmdFilter*/);

    Deriv velocity(int index) const;

    /// Pre-construction check method called by ObjectFactory.
    /// Check that DataTypes matches the MechanicalState.
    template <class T>
    static bool canCreate(
        T*& obj, sofa::core::objectmodel::BaseContext* context, sofa::core::objectmodel::BaseObjectDescription* arg)
    {
        if (dynamic_cast<sofa::core::behavior::MechanicalState<DataTypes>*>(context->getMechanicalState()) == NULL)
            return false;
        return BaseObject::canCreate(obj, context, arg);
    }

    virtual std::string getTemplateName() const override { return templateName(this); }

    static std::string templateName(const ScytherTriangleCollisionModel<DataTypes>* = NULL)
    {
        return DataTypes::Name();
    }

    void computeBBox(const sofa::core::ExecParams* params, bool onlyVisible = false) override;
};

template <class DataTypes>
inline TScytherTriangle<DataTypes>::TScytherTriangle(ParentModel* model, int index)
    : sofa::core::TCollisionElementIterator<ParentModel>(model, index)
{
}

template <class DataTypes>
inline TScytherTriangle<DataTypes>::TScytherTriangle(const sofa::core::CollisionElementIterator& i)
    : sofa::core::TCollisionElementIterator<ParentModel>(static_cast<ParentModel*>(i.getCollisionModel()), i.getIndex())
{
}

template <class DataTypes>
inline TScytherTriangle<DataTypes>::TScytherTriangle(
    ParentModel* model, int index, sofa::helper::ReadAccessor<typename DataTypes::VecCoord>& x)
    : TScytherTriangle(model, index)
{
    SOFA_UNUSED(x);
}

template <class DataTypes>
inline const typename DataTypes::Coord& TScytherTriangle<DataTypes>::p1() const
{
    return this->model->m_mstate->read(sofa::core::ConstVecCoordId::position())
        ->getValue()[(*(this->model->m_triangles))[this->index][0]];
}
template <class DataTypes>
inline const typename DataTypes::Coord& TScytherTriangle<DataTypes>::p2() const
{
    return this->model->m_mstate->read(sofa::core::ConstVecCoordId::position())
        ->getValue()[(*(this->model->m_triangles))[this->index][1]];
}
template <class DataTypes>
inline const typename DataTypes::Coord& TScytherTriangle<DataTypes>::p3() const
{
    return this->model->m_mstate->read(sofa::core::ConstVecCoordId::position())
        ->getValue()[(*(this->model->m_triangles))[this->index][2]];
}
template <class DataTypes>
inline const typename DataTypes::Coord& TScytherTriangle<DataTypes>::p(int i) const
{
    return this->model->m_mstate->read(sofa::core::ConstVecCoordId::position())
        ->getValue()[(*(this->model->m_triangles))[this->index][i]];
}
template <class DataTypes>
inline const typename DataTypes::Coord& TScytherTriangle<DataTypes>::operator[](int i) const
{
    return this->model->m_mstate->read(sofa::core::ConstVecCoordId::position())
        ->getValue()[(*(this->model->m_triangles))[this->index][i]];
}

template <class DataTypes>
inline const typename DataTypes::Coord& TScytherTriangle<DataTypes>::p1Free() const
{
    return (*this->model->m_mstate->read(sofa::core::ConstVecCoordId::freePosition())
                 ->getValue())[(*(this->model->m_triangles))[this->index][0]];
}
template <class DataTypes>
inline const typename DataTypes::Coord& TScytherTriangle<DataTypes>::p2Free() const
{
    return (*this->model->m_mstate->read(sofa::core::ConstVecCoordId::freePosition())
                 ->getValue())[(*(this->model->m_triangles))[this->index][1]];
}
template <class DataTypes>
inline const typename DataTypes::Coord& TScytherTriangle<DataTypes>::p3Free() const
{
    return (*this->model->m_mstate->read(sofa::core::ConstVecCoordId::freePosition())
                 ->getValue())[(*(this->model->m_triangles))[this->index][2]];
}

template <class DataTypes>
inline int TScytherTriangle<DataTypes>::p1Index() const
{
    return (*(this->model->m_triangles))[this->index][0];
}
template <class DataTypes>
inline int TScytherTriangle<DataTypes>::p2Index() const
{
    return (*(this->model->m_triangles))[this->index][1];
}
template <class DataTypes>
inline int TScytherTriangle<DataTypes>::p3Index() const
{
    return (*(this->model->m_triangles))[this->index][2];
}

template <class DataTypes>
inline const typename DataTypes::Deriv& TScytherTriangle<DataTypes>::v1() const
{
    return (this->model->m_mstate->read(sofa::core::ConstVecDerivId::velocity())
                ->getValue())[(*(this->model->m_triangles))[this->index][0]];
}
template <class DataTypes>
inline const typename DataTypes::Deriv& TScytherTriangle<DataTypes>::v2() const
{
    return this->model->m_mstate->read(sofa::core::ConstVecDerivId::velocity())
        ->getValue()[(*(this->model->m_triangles))[this->index][1]];
}
template <class DataTypes>
inline const typename DataTypes::Deriv& TScytherTriangle<DataTypes>::v3() const
{
    return this->model->m_mstate->read(sofa::core::ConstVecDerivId::velocity())
        ->getValue()[(*(this->model->m_triangles))[this->index][2]];
}
template <class DataTypes>
inline const typename DataTypes::Deriv& TScytherTriangle<DataTypes>::v(int i) const
{
    return this->model->m_mstate->read(sofa::core::ConstVecDerivId::velocity())
        ->getValue()[(*(this->model->m_triangles))[this->index][i]];
}

template <class DataTypes>
inline const typename DataTypes::Deriv& TScytherTriangle<DataTypes>::n() const
{
    return this->model->m_normals[this->index];
}
template <class DataTypes>
inline typename DataTypes::Deriv& TScytherTriangle<DataTypes>::n()
{
    return this->model->m_normals[this->index];
}

template <class DataTypes>
inline int TScytherTriangle<DataTypes>::flags() const
{
    return this->model->getTriangleFlags(this->index);
}

template <class DataTypes>
inline bool TScytherTriangle<DataTypes>::hasFreePosition() const
{
    return this->model->m_mstate->read(sofa::core::ConstVecCoordId::freePosition())->isSet();
}

template <class DataTypes>
inline typename DataTypes::Deriv ScytherTriangleCollisionModel<DataTypes>::velocity(int index) const
{
    return (m_mstate->read(sofa::core::ConstVecDerivId::velocity())->getValue()[(*(m_triangles))[index][0]] +
            m_mstate->read(sofa::core::ConstVecDerivId::velocity())->getValue()[(*(m_triangles))[index][1]] +
            m_mstate->read(sofa::core::ConstVecDerivId::velocity())->getValue()[(*(m_triangles))[index][2]]) /
           ((Real)(3.0));
}
typedef ScytherTriangleCollisionModel<sofa::defaulttype::Vec3Types> ScytherTriangleModel;
typedef TScytherTriangle<sofa::defaulttype::Vec3Types>              ScytherTriangle;

#if !defined(SCYTHER_COLLISION_TRIANGLE_MODEL_CPP)
extern template class SOFA_SCYTHER_API ScytherTriangleCollisionModel<sofa::defaulttype::Vec3Types>;
#endif

} // namespace SofaInterface

#endif