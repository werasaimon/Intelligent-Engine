#ifndef MEMORY_ILIST_H_
#define MEMORY_ILIST_H_



//Libraries
#include "../ISettings.h"

namespace IPhysics
{


template<class T>
class  IListElement
{
  public:

    IListElement()
    : mNext(0),
      mPrev(0)
    {
    }

    IListElement( T* element ,IListElement *next, IListElement *prev)
    : mNext(next),
      mPrev(prev),
      mPointer(element)
    {
    }


    IListElement *getNext() const { return mNext; }
    IListElement *getPrev() const { return mPrev; }


    bool isHead() const { return mPrev == 0; }
    bool isTail() const { return mNext == 0; }


    void insertBefore(IListElement *link)
    {
        mNext         = link;
        mPrev         = link->mPrev;
        mNext->mPrev = this;
        mPrev->mNext = this;
    }


    void insertAfter(IListElement *link)
    {
        mNext         = link->mNext;
        mPrev         = link;
        mNext->mPrev = this;
        mPrev->mNext = this;
    }


    void remove()
    {
        mNext->mPrev = mPrev;
        mPrev->mNext = mNext;
    }



    ///Get real value
    T *getPointer() const;

    void setPointer(T *pointer);

private:

    IListElement  *mNext;
    IListElement  *mPrev;

    ///pointer value
    T *mPointer;
};


///Get real value
template< class T >
T *IListElement<T>::getPointer() const
{
   return mPointer;
}

template< class T >
void IListElement<T>::setPointer(T *pointer)
{
   mPointer = pointer;
}


template<class T>
class IList
{
  public:

    IList()
    : mHead(&mTail, 0),
      mTail(0, &mHead)
    {
    }

    IListElement<T> *getHead() const { return mHead.getNext(); }
    IListElement<T> *getTail() const { return mTail.getPrev(); }

    void addHead(IListElement<T> *link) { link->insertAfter(&mHead); }
    void addTail(IListElement<T> *link) { link->insertBefore(&mTail); }

 private:

    IListElement<T> mHead;
    IListElement<T> mTail;
};




} /* namespace real_physics */

#endif /* MEMORY_ILIST_H_ */
