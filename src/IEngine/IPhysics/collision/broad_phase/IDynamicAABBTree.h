#ifndef IDYNAMICAABBTREE_H
#define IDYNAMICAABBTREE_H

#include "../../common/ISettings.h"
#include "../IAABB.h"

namespace IPhysics
{


// Declarations
//class IBroadPhaseAlgorithm;
//class IBroadPhaseRaycastTestCallback;
class IDynamicAABBTreeOverlapCallback;

struct IRaycastTest;

// Structure TreeNode
/**
 * This structure represents a node of the dynamic AABB tree.
 */
struct ITreeNode
{

    // -------------------- Constants -------------------- //

    /// Null tree node constant
    const static i32 NULL_TREE_NODE;

    // -------------------- Attributes -------------------- //

        // A node is either in the tree (has a parent) or in the free nodes list
        // (has a next node)
        union
        {

          /// Parent node ID
          i32 parentID;

          /// Next allocated node ID
          i32 nextNodeID;
        };

        // A node is either a leaf (has data) or is an internal node (has children)
         union
    {

        /// Left and right child of the node (children[0] = left child)
        i32 children[2];

        /// Two pieces of data stored at that node (in case the node is a leaf)
        union
        {
            i32   dataInt[2];
            void* dataPointer;
        };
    };

    /// Height of the node in the tree
    i16 height;

    /// Fat axis aligned bounding box (AABB) corresponding to the node
    IAABB aabb;

    // -------------------- Methods -------------------- //

    /// Return true if the node is a leaf of the tree
    bool isLeaf() const;
};




// Class DynamicAABBTreeOverlapCallback
/**
 * Overlapping callback method that has to be used as parameter of the
 * reportAllShapesOverlappingWithNode() method.
 */
class IDynamicAABBTreeOverlapCallback
{

    public :

        // Called when a overlapping node has been found during the call to
        // DynamicAABBTree:reportAllShapesOverlappingWithAABB()
        virtual void notifyOverlappingNode(i32 nodeId)=0;
};




// Class DynamicAABBTreeRaycastCallback
/**
 * Raycast callback in the Dynamic AABB Tree called when the AABB of a leaf
 * node is hit by the ray.
 */
class IDynamicAABBTreeRaycastCallback
{

    public:

        // Called when the AABB of a leaf node is hit by a ray
        virtual scalar raycastBroadPhaseShape(i32 nodeId, const IRay& ray)=0;

};

// Class DynamicAABBTree
/**
 * This class implements a dynamic AABB tree that is used for broad-phase
 * collision detection. This data structure is inspired by Nathanael Presson's
 * dynamic tree implementation in BulletPhysics. The following implementation is
 * based on the one from Erin Catto in Box2D as described in the book
 * "Introduction to Game Physics with Box2D" by Ian Parberry.
 */
class IDynamicAABBTree
{

    private:

        // -------------------- Attributes -------------------- //

        /// Pointer to the memory location of the nodes of the tree
        ITreeNode* mNodes;

        /// ID of the root node of the tree
        i32 mRootNodeID;

        /// ID of the first node of the list of free (allocated) nodes in the tree that we can use
        i32 mFreeNodeID;

        /// Number of allocated nodes in the tree
        i32 mNbAllocatedNodes;

        /// Number of nodes in the tree
        i32 mNbNodes;

        /// Extra AABB Gap used to allow the collision shape to move a little bit
        /// without triggering a large modification of the tree which can be costly
        scalar mExtraAABBGap;

        // -------------------- Methods -------------------- //

        /// Allocate and return a node to use in the tree
        u32 allocateNode();

        /// Release a node
        void releaseNode(i32 nodeID);

        /// Insert a leaf node in the tree
        void insertLeafNode(i32 nodeID);

        /// Remove a leaf node from the tree
        void removeLeafNode(i32 nodeID);

        /// Balance the sub-tree of a given node using left or right rotations.
        i32 balanceSubTreeAtNode(i32 nodeID);

        /// Compute the height of a given node in the tree
        i32 computeHeight(i32 nodeID);

        /// Internally add an object into the tree
        i32 addObjectInternal(const IAABB& aabb);

        /// Initialize the tree
        void init();

#ifndef NDEBUG

        /// Check if the tree structure is valid (for debugging purpose)
        void check() const;

        /// Check if the node structure is valid (for debugging purpose)
        void checkNode(i32 nodeID) const;

#endif

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        IDynamicAABBTree(scalar extraAABBGap = scalar(0.0));

        /// Destructor
        ~IDynamicAABBTree();

        /// Add an object into the tree (where node data are two integers)
        i32 addObject(const IAABB& aabb, i32 data1, i32 data2);

        /// Add an object into the tree (where node data is a pointer)
        i32 addObject(const IAABB& aabb, void* data);

        /// Remove an object from the tree
        void removeObject(u32 nodeID);

        /// Update the dynamic tree after an object has moved.
        bool updateObject(i32 nodeID, const IAABB& newAABB, const IVector3& displacement, bool forceReinsert = false);

        /// Return the fat AABB corresponding to a given node ID
        const IAABB& getFatAABB(i32 nodeID) const;

        /// Return the pointer to the data array of a given leaf node of the tree
        i32* getNodeDataInt(i32 nodeID) const;

        /// Return the data pointer of a given leaf node of the tree
        void* getNodeDataPointer(i32 nodeID) const;

        /// Report all shapes overlapping with the AABB given in parameter.
        void reportAllShapesOverlappingWithAABB(const IAABB& aabb, IDynamicAABBTreeOverlapCallback& callback) const;

        /// Ray casting method
        void raycast(const IRay& ray, IDynamicAABBTreeRaycastCallback& callback) const;

        /// Compute the height of the tree
        i32 computeHeight();

        /// Return the root AABB of the tree
        IAABB getRootAABB() const;

        /// Clear all the nodes and reset the tree
        void reset();
};

// Return true if the node is a leaf of the tree
SIMD_INLINE bool ITreeNode::isLeaf() const
{
    return (height == 0);
}

// Return the fat AABB corresponding to a given node ID
SIMD_INLINE const IAABB& IDynamicAABBTree::getFatAABB(i32 nodeID) const
{
    assert(nodeID >= 0 && nodeID < mNbAllocatedNodes);
    return mNodes[nodeID].aabb;
}

// Return the pointer to the data array of a given leaf node of the tree
SIMD_INLINE i32* IDynamicAABBTree::getNodeDataInt(i32 nodeID) const
{
    assert(nodeID >= 0 && nodeID < mNbAllocatedNodes);
    assert(mNodes[nodeID].isLeaf());
    return mNodes[nodeID].dataInt;
}

// Return the pointer to the data pointer of a given leaf node of the tree
SIMD_INLINE void* IDynamicAABBTree::getNodeDataPointer(i32 nodeID) const
{
    assert(nodeID >= 0 && nodeID < mNbAllocatedNodes);
    assert(mNodes[nodeID].isLeaf());
    return mNodes[nodeID].dataPointer;
}

// Return the root AABB of the tree
SIMD_INLINE IAABB IDynamicAABBTree::getRootAABB() const
{
    return getFatAABB(mRootNodeID);
}

// Add an object into the tree. This method creates a new leaf node in the tree and
// returns the ID of the corresponding node.
SIMD_INLINE i32 IDynamicAABBTree::addObject(const IAABB& aabb, i32 data1, i32 data2)
{

    i32 nodeId = addObjectInternal(aabb);

    mNodes[nodeId].dataInt[0] = data1;
    mNodes[nodeId].dataInt[1] = data2;

    return nodeId;
}

// Add an object into the tree. This method creates a new leaf node in the tree and
// returns the ID of the corresponding node.
SIMD_INLINE i32 IDynamicAABBTree::addObject(const IAABB& aabb, void* data)
{

    i32 nodeId = addObjectInternal(aabb);

    mNodes[nodeId].dataPointer = data;

    return nodeId;
}



}
#endif // IDYNAMICAABBTREE_H
